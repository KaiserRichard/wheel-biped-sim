/* Host simulator: runs the same bench logic as the F446 firmware behind a
 * pseudo-terminal, so pi/link_bench.py can be tested without hardware.
 * Prints the pty path on stdout, then serves until killed. Optional argv[1]:
 * write a final counter line to that file on SIGTERM/SIGINT. */
#define _XOPEN_SOURCE 600
#include "bench.h"
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }
static uint64_t micros(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u; }
static void tx(void *ctx, const uint8_t *d, size_t n) { int fd = *(int *)ctx; while (n) { ssize_t w = write(fd, d, n); if (w <= 0) return; d += w; n -= (size_t)w; } }

int main(int argc, char **argv) {
 int fd = posix_openpt(O_RDWR | O_NOCTTY); bench_port p; struct termios t; uint64_t t0;
 if (fd < 0 || grantpt(fd) || unlockpt(fd)) { perror("pty"); return 1; }
 tcgetattr(fd, &t);
 t.c_iflag = 0; t.c_oflag = 0; t.c_lflag = 0; t.c_cflag = (t.c_cflag & ~(tcflag_t)(CSIZE | PARENB)) | CS8 | CREAD | CLOCAL;
 t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
 tcsetattr(fd, TCSANOW, &t);
 signal(SIGTERM, on_signal); signal(SIGINT, on_signal);
 printf("%s\n", ptsname(fd)); fflush(stdout);
 t0 = micros();
 bench_init(&p, 0x51A1u, tx, &fd);
 while (!stop) {
  struct pollfd pf = { fd, POLLIN, 0 }; uint8_t buf[256]; ssize_t n;
  if (poll(&pf, 1, 1) > 0 && (pf.revents & POLLIN)) {
   n = read(fd, buf, sizeof(buf));
   if (n > 0) bench_rx(&p, buf, (size_t)n, micros() - t0);
  }
  bench_poll(&p, micros() - t0);
 }
 if (argc > 1) {
  FILE *f = fopen(argv[1], "w");
  if (f) { fprintf(f, "accepted=%u rejected=%u wire_errors=%u status_sent=%u tx_dropped=%u advisory_accepted=%u advisory_rejected=%u advisory_stale=%u\n", p.accepted, p.rejected, p.stream.errors, p.status_sent, p.tx_dropped, p.advisory_accepted, p.advisory_rejected, p.advisory_stale); fclose(f); }
 }
 return 0;
}
