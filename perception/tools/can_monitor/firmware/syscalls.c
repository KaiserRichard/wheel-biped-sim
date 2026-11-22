#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>

extern char __heap_start, __heap_limit;
extern int clock_gettime(clockid_t clock_id, struct timespec *ts);
static char *heap_cur;

void *_sbrk(ptrdiff_t increment) {
  if (heap_cur == NULL) heap_cur = &__heap_start;
  char *prev = heap_cur;
  if (increment < 0 || heap_cur + increment > &__heap_limit) {
    errno = ENOMEM;
    return (void *)-1;
  }
  heap_cur += increment;
  return prev;
}

int _write(int fd, const void *buf, size_t count) { (void)fd; (void)buf; (void)count; errno = EIO; return -1; }
int _read(int fd, void *buf, size_t count) { (void)fd; (void)buf; (void)count; errno = EIO; return -1; }
int _close(int fd) { (void)fd; errno = EBADF; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
off_t _lseek(int fd, off_t offset, int whence) { (void)fd; (void)offset; (void)whence; errno = ESPIPE; return (off_t)-1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int status) { (void)status; for (;;) { __asm volatile("wfi"); } }
int _gettimeofday(struct timeval *tv, void *tz) {
  (void)tz;
  struct timespec ts;
  if (clock_gettime(CLOCK_REALTIME, &ts) != 0) return -1;
  tv->tv_sec = ts.tv_sec;
  tv->tv_usec = (suseconds_t)(ts.tv_nsec / 1000L);
  return 0;
}
