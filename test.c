// feeder_server.c  — bản sửa: dùng "group" + "ledData", pad group 3 chữ số
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

#define TBL_COUNT     20
#define POS_PER_TBL   15
#define UART_DEV     "/dev/serial0"
#define UART_BAUD     B9600

static int g_uart_fd = -1;

static int uart_open(const char *dev, speed_t baud) {
  int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) return -1;
  struct termios tio; memset(&tio, 0, sizeof(tio));
  if (tcgetattr(fd, &tio) != 0) { close(fd); return -2; }
  cfsetospeed(&tio, baud); cfsetispeed(&tio, baud);
  tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;   // 8 data
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(PARENB | PARODD);            // N
  tio.c_cflag &= ~CSTOPB;                       // 1 stop
  tio.c_cflag &= ~CRTSCTS;                      // no HW flow
  tio.c_iflag = tio.c_oflag = tio.c_lflag = 0;
  tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 10;     // read timeout 1s
  if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return -3; }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

static unsigned char xorsum(const unsigned char *p, size_t n) {
  unsigned char x = 0; for (size_t i = 0; i < n; i++) x ^= p[i]; return x;
}

// Frame: '@' + BCD(group hundreds,tens,units) + 20*15 HEX + CS(2 HEX) + '\r'
static size_t build_frame(unsigned char *out, int group,
                          const char tables[TBL_COUNT][POS_PER_TBL + 1]) {
  unsigned char *w = out;
  *w++ = '@';

  if (group < 0) group = 0; if (group > 999) group = 999;
  int g2 = (group / 100) % 10;    // hundreds
  int g1 = (group / 10)  % 10;    // tens
  int g0 = group % 10;            // units
  *w++ = '0' + g2;
  *w++ = '0' + g1;
  *w++ = '0' + g0;

  for (int t = 0; t < TBL_COUNT; t++) {
    for (int i = 0; i < POS_PER_TBL; i++) {
      char c = tables[t][i];
      if (!isxdigit((unsigned char)c)) c = '0';
      *w++ = (unsigned char) toupper((unsigned char) c);
    }
  }

  unsigned char cs = xorsum(out + 1, 3 + TBL_COUNT * POS_PER_TBL);
  static const char HEX[] = "0123456789ABCDEF";
  *w++ = HEX[(cs >> 4) & 0xF];
  *w++ = HEX[cs & 0xF];
  *w++ = '\r';

  return (size_t)(w - out);
}

static void send_to_uart(int group, const char tables[TBL_COUNT][POS_PER_TBL + 1]) {
  if (g_uart_fd < 0) {
    g_uart_fd = uart_open(UART_DEV, UART_BAUD);
    if (g_uart_fd < 0) { fprintf(stderr, "UART open failed: %s\n", strerror(errno)); return; }
  }
  unsigned char frame[4 + TBL_COUNT * POS_PER_TBL + 2 + 1];
  size_t n = build_frame(frame, group, tables);

  // Log khung để bạn xác nhận group đã pad đúng
  fprintf(stderr, "TX group=%03d, len=%zu\n", group, n);

  int w = write(g_uart_fd, frame, n);
  fprintf(stderr, "write()=%d, errno=%d\n", w, errno);
  tcdrain(g_uart_fd);
}

// helper: copy chuỗi JSON vào buffer an toàn
static void json_get_str_buf(struct mg_str json, const char *path,
                             char *dst, size_t dstsz) {
  char *s = mg_json_get_str(json, path);   // malloc'ed (hoặc NULL)
  if (dstsz) dst[0] = 0;
  if (s) {
    strncpy(dst, s, dstsz ? dstsz - 1 : 0);
    if (dstsz) dst[dstsz - 1] = 0;
    free(s);
  }
}

// Handler 3 tham số; chỉ GỬI UART khi có POST /API/setdata
static void http_cb(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;

    if (mg_strcmp(hm->uri, mg_str("/API/setdata")) == 0) {
      // Lấy group (số)
      char buf[16];
      json_get_str_buf(hm->body, "$.group", buf, sizeof(buf));
      int group = atoi(buf);                  // 0..999 (sẽ clamp khi build_frame)

      // Đọc mảng ledData[0..19] (mỗi phần tử 15 HEX)
      char tables[TBL_COUNT][POS_PER_TBL + 1];
      memset(tables, 0, sizeof(tables));
      for (int i = 0; i < TBL_COUNT; i++) {
        char path[32]; snprintf(path, sizeof(path), "$.ledData[%d]", i);
        json_get_str_buf(hm->body, path, tables[i], sizeof(tables[i]));
        // đảm bảo đủ 15 ký tự
        for (int k = 0; k < POS_PER_TBL; k++)
          if (tables[i][k] == 0) tables[i][k] = '0';
      }

      // Gửi UART NGAY KHI có POST hợp lệ
      send_to_uart(group, tables);

      mg_http_reply(c, 200, "Content-Type: text/plain\r\n", "OK\n");
    } else {
      mg_http_reply(c, 404, "", "Not found\n");
    }
  }
}

int main(void) {
  struct mg_mgr mgr; mg_mgr_init(&mgr);
  printf("HTTP server on http://0.0.0.0:8080\nUART dev: %s (9600-8N1)\n", UART_DEV);

  // Chỉ listen; KHÔNG gửi UART nếu không có POST
  if (mg_http_listen(&mgr, "http://0.0.0.0:8080", http_cb, NULL) == NULL) {
    fprintf(stderr, "Listen failed\n"); return 1;
  }
  for (;;) mg_mgr_poll(&mgr, 100);
  mg_mgr_free(&mgr);
  if (g_uart_fd >= 0) close(g_uart_fd);
  return 0;
}
