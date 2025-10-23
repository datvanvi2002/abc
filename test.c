// feeder_server.c — JSON: {"group": <int>, "ledData": [32 x "15HEX"]} -> gửi 20 đầu
#include "mongoose.h"
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define TBL_USE_COUNT   20      // chỉ gửi 20
#define TBL_TOTAL_RECV  32      // web có thể gửi 32
#define POS_PER_TBL     15
#define UART_DEV       "/dev/serial0"
#define UART_BAUD       B9600
#define END_CHAR        '!'     // nếu muốn CR thì đổi thành '\r'

static int g_uart_fd = -1;

static int uart_open(const char *dev, speed_t baud) {
  int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) return -1;
  struct termios tio; memset(&tio, 0, sizeof(tio));
  if (tcgetattr(fd, &tio) != 0) { close(fd); return -2; }
  cfsetospeed(&tio, baud); cfsetispeed(&tio, baud);
  tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(PARENB | PARODD);
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_iflag = tio.c_oflag = tio.c_lflag = 0;
  tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 10;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return -3; }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

static unsigned char xorsum(const unsigned char *p, size_t n) {
  unsigned char x = 0; for (size_t i = 0; i < n; i++) x ^= p[i]; return x;
}

// Lấy số nguyên từ JSON: hỗ trợ cả number (12) lẫn string ("12")
static int json_get_int_loose(struct mg_str json, const char *path, int *out) {
  // Thử lấy số trực tiếp (nếu bản Mongoose hỗ trợ)
  #ifdef MG_ENABLE_PACKED
  #endif
  // Một số bản có mg_json_get_num, một số không — thử dùng nếu có:
  #ifdef MG_JSON_GET_NUM_HAS_DEFAULT
    double d = mg_json_get_num(json, path, -1);
    if (d >= 0) { *out = (int) d; return 1; }
  #else
  {
    // Có bản chỉ có mg_json_get_num(json, path)
    extern double mg_json_get_num(struct mg_str, const char *);
    // dùng try-catch compile không được, nên bao bằng weak assumption:
    // nếu hàm không tồn tại, linker sẽ lỗi — khi đó hãy dùng nhánh dưới.
  }
  #endif
  // Fallback: lấy chuỗi rồi atoi
  char *s = mg_json_get_str(json, path);
  if (s) { *out = atoi(s); free(s); return 1; }
  return 0;
}

// Frame: '@' + BCD(group 3 chữ số) + 20*15 HEX + CS(2 HEX) + END_CHAR
static size_t build_frame(unsigned char *out, int group,
                          const char tables[TBL_USE_COUNT][POS_PER_TBL + 1]) {
  unsigned char *w = out;
  *w++ = '@';

  if (group < 0) group = 0; if (group > 999) group = 999;
  int g2 = (group / 100) % 10, g1 = (group / 10) % 10, g0 = group % 10;
  *w++ = '0' + g2; *w++ = '0' + g1; *w++ = '0' + g0;

  for (int t = 0; t < TBL_USE_COUNT; t++) {
    for (int i = 0; i < POS_PER_TBL; i++) {
      char c = tables[t][i];
      if (!isxdigit((unsigned char)c)) c = '0';
      *w++ = (unsigned char) toupper((unsigned char)c);
    }
  }

  unsigned char cs = xorsum(out + 1, 3 + TBL_USE_COUNT * POS_PER_TBL);
  static const char HEX[] = "0123456789ABCDEF";
  *w++ = HEX[(cs >> 4) & 0xF];
  *w++ = HEX[cs & 0xF];
  *w++ = END_CHAR;
  return (size_t)(w - out);
}

static void send_to_uart(int group, const char tables[TBL_USE_COUNT][POS_PER_TBL + 1]) {
  if (g_uart_fd < 0) {
    g_uart_fd = uart_open(UART_DEV, UART_BAUD);
    if (g_uart_fd < 0) { fprintf(stderr, "UART open failed: %s\n", strerror(errno)); return; }
  }
  unsigned char frame[4 + TBL_USE_COUNT * POS_PER_TBL + 2 + 1];
  size_t n = build_frame(frame, group, tables);
  fprintf(stderr, "TX group=%03d, len=%zu\n", group, n);
  int w = write(g_uart_fd, frame, n);
  fprintf(stderr, "write()=%d, errno=%d\n", w, errno);
  tcdrain(g_uart_fd);
}

// Lấy chuỗi JSON an toàn
static void json_get_str_buf(struct mg_str json, const char *path,
                             char *dst, size_t dstsz) {
  char *s = mg_json_get_str(json, path);
  if (dstsz) dst[0] = 0;
  if (s) { strncpy(dst, s, dstsz ? dstsz - 1 : 0); if (dstsz) dst[dstsz - 1] = 0; free(s); }
}

// Handler: chỉ gửi UART khi POST /API/setdata hợp lệ
static void http_cb(struct mg_connection *c, int ev, void *ev_data) {
  if (ev != MG_EV_HTTP_MSG) return;
  struct mg_http_message *hm = (struct mg_http_message *) ev_data;

  // CORS preflight
  if (mg_strcmp(hm->method, mg_str("OPTIONS")) == 0 &&
      mg_strcmp(hm->uri, mg_str("/API/setdata")) == 0) {
    mg_http_reply(c, 204,
      "Access-Control-Allow-Origin: *\r\n"
      "Access-Control-Allow-Methods: POST, OPTIONS\r\n"
      "Access-Control-Allow-Headers: Content-Type\r\n", "");
    return;
  }

  if (mg_strcmp(hm->method, mg_str("POST")) != 0 ||
      mg_strcmp(hm->uri, mg_str("/API/setdata")) != 0) {
    mg_http_reply(c, 404, "", "Not found\n");
    return;
  }

  // ----- Parse group -----
  int group = -1;
  if (!json_get_int_loose(hm->body, "$.group", &group)) {
    mg_http_reply(c, 400, "", "Missing group\n");
    return;
  }

  // ----- Parse ledData[0..31], dùng 0..19 -----
  char use[TBL_USE_COUNT][POS_PER_TBL + 1];
  memset(use, 0, sizeof(use));
  int any = 0;
  for (int i = 0; i < TBL_TOTAL_RECV; i++) {
    char path[32]; snprintf(path, sizeof(path), "$.ledData[%d]", i);
    char tmp[POS_PER_TBL + 1] = {0};
    json_get_str_buf(hm->body, path, tmp, sizeof(tmp));   // có thể rỗng nếu không có phần tử
    if (i < TBL_USE_COUNT) {
      for (int k = 0; k < POS_PER_TBL; k++) use[i][k] = tmp[k] ? tmp[k] : '0';
      if (use[i][0]) any = 1;
    }
  }
  if (!any) { mg_http_reply(c, 400, "", "Missing ledData\n"); return; }

  // ----- Chỉ tại đây mới gửi UART -----
  send_to_uart(group, use);

  mg_http_reply(c, 200,
    "Content-Type: text/plain\r\n"
    "Access-Control-Allow-Origin: *\r\n", "OK\n");
}

int main(void) {
  struct mg_mgr mgr; mg_mgr_init(&mgr);
  printf("HTTP server on http://0.0.0.0:8080\nUART dev: %s (9600-8N1)\n", UART_DEV);
  if (mg_http_listen(&mgr, "http://0.0.0.0:8080", http_cb, NULL) == NULL) {
    fprintf(stderr, "Listen failed\n"); return 1;
  }
  for (;;) mg_mgr_poll(&mgr, 100);
  mg_mgr_free(&mgr);
  if (g_uart_fd >= 0) close(g_uart_fd);
  return 0;
}
