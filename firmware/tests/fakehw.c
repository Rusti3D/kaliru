// Test tool: an LD_PRELOAD shim that fakes the Linux hardware interfaces a
// linux MCU process uses (gpiochip line handles, spidev, i2c-dev, iio adc
// files) and logs every pin write and bus transfer. Run the C and the Rust
// MCU under it and compare the logs (mcu_run.py --fakehw).
//
// Inputs: FAKEHW_LOG=<file> (the event log), FAKEHW_GPIO_IN=<value>
// (what every input line reads, default 0). SPI/I2C reads return a byte
// pattern that counts up per device and transfer, ADC files read "2048".
//
// gcc -shared -fPIC -O2 -o fakehw.so fakehw.c -ldl
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/spi/spidev.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

enum { F_NONE, F_CHIP, F_LINE, F_SPI, F_I2C, F_ADC };
struct fake { int kind, chip, line, bus, dev, addr, counter; };
static struct fake fakes[4096];
static FILE *logf;

static int (*real_open)(const char *, int, ...);
static int (*real_ioctl)(int, unsigned long, ...);
static ssize_t (*real_write)(int, const void *, size_t);
static ssize_t (*real_pread)(int, void *, size_t, off_t);
static int (*real_access)(const char *, int);

static void
init(void)
{
    if (real_open)
        return;
    real_open = dlsym(RTLD_NEXT, "open");
    real_ioctl = dlsym(RTLD_NEXT, "ioctl");
    real_write = dlsym(RTLD_NEXT, "write");
    real_pread = dlsym(RTLD_NEXT, "pread");
    real_access = dlsym(RTLD_NEXT, "access");
    const char *p = getenv("FAKEHW_LOG");
    if (p)
        logf = fopen(p, "w");
}

static void
logev(const char *fmt, ...)
{
    if (!logf)
        return;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fprintf(logf, "%ld.%09ld ", (long)ts.tv_sec, ts.tv_nsec);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(logf, fmt, ap);
    va_end(ap);
    fputc('\n', logf);
    fflush(logf);
}

static int
new_fake(struct fake f)
{
    int fd = real_open("/dev/null", O_RDWR | O_CLOEXEC);
    if (fd >= 0 && fd < 4096)
        fakes[fd] = f;
    return fd;
}

static struct fake *
lookup(int fd)
{
    if (fd < 0 || fd >= 4096 || fakes[fd].kind == F_NONE)
        return NULL;
    return &fakes[fd];
}

static int
fake_path(const char *path, struct fake *f)
{
    memset(f, 0, sizeof(*f));
    if (sscanf(path, "/dev/gpiochip%d", &f->chip) == 1)
        f->kind = F_CHIP;
    else if (sscanf(path, "/dev/spidev%d.%d", &f->bus, &f->dev) == 2)
        f->kind = F_SPI;
    else if (sscanf(path, "/dev/i2c-%d", &f->bus) == 1)
        f->kind = F_I2C;
    else if (sscanf(path, "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw", &f->line) == 1)
        f->kind = F_ADC;
    return f->kind != F_NONE;
}

int
access(const char *path, int mode)
{
    init();
    struct fake f;
    if (fake_path(path, &f))
        return 0;
    return real_access(path, mode);
}

int
open(const char *path, int flags, ...)
{
    init();
    va_list ap;
    va_start(ap, flags);
    int mode = va_arg(ap, int);
    va_end(ap);
    struct fake f;
    if (fake_path(path, &f)) {
        logev("open %s", path);
        return new_fake(f);
    }
    return real_open(path, flags, mode);
}

int open64(const char *path, int flags, ...) __attribute__((alias("open")));

static void
hexlog(const char *what, struct fake *f, const unsigned char *d, int len)
{
    char buf[1024];
    int n = 0;
    for (int i = 0; i < len && n < 1000; i++)
        n += sprintf(&buf[n], "%02x", d[i]);
    buf[n] = '\0';
    logev("%s bus=%d dev=%d addr=%d %s", what, f->bus, f->dev, f->addr, buf);
}

int
ioctl(int fd, unsigned long req, ...)
{
    init();
    va_list ap;
    va_start(ap, req);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    struct fake *f = lookup(fd);
    if (!f)
        return real_ioctl(fd, req, arg);
    if (f->kind == F_CHIP && req == GPIO_GET_LINEHANDLE_IOCTL) {
        struct gpiohandle_request *r = arg;
        struct fake line = { .kind = F_LINE, .chip = f->chip, .line = r->lineoffsets[0] };
        r->fd = new_fake(line);
        logev("line chip=%d line=%d flags=%u value=%d", f->chip, line.line, r->flags, r->default_values[0]);
        return 0;
    }
    if (f->kind == F_LINE && req == GPIOHANDLE_SET_LINE_VALUES_IOCTL) {
        struct gpiohandle_data *d = arg;
        logev("set chip=%d line=%d value=%d", f->chip, f->line, d->values[0]);
        return 0;
    }
    if (f->kind == F_LINE && req == GPIOHANDLE_GET_LINE_VALUES_IOCTL) {
        struct gpiohandle_data *d = arg;
        const char *v = getenv("FAKEHW_GPIO_IN");
        d->values[0] = v ? atoi(v) : 0;
        return 0;
    }
    if (f->kind == F_SPI && (req == SPI_IOC_WR_MODE || req == SPI_IOC_WR_MAX_SPEED_HZ)) {
        logev("spi_setup bus=%d dev=%d req=%lx value=%u", f->bus, f->dev, req
              , req == SPI_IOC_WR_MODE ? *(unsigned char *)arg : *(unsigned *)arg);
        return 0;
    }
    if (f->kind == F_SPI && req == SPI_IOC_MESSAGE(1)) {
        struct spi_ioc_transfer *t = arg;
        unsigned char *d = (void *)(uintptr_t)t->tx_buf;
        hexlog("spi_tx", f, d, t->len);
        unsigned char *rx = (void *)(uintptr_t)t->rx_buf;
        for (unsigned i = 0; i < t->len; i++)
            rx[i] = f->counter + i;
        f->counter++;
        return t->len;
    }
    if (f->kind == F_I2C && req == I2C_FUNCS) {
        *(unsigned long *)arg = I2C_FUNC_I2C;
        return 0;
    }
    if (f->kind == F_I2C && req == I2C_SLAVE) {
        f->addr = (int)(uintptr_t)arg;
        logev("i2c_slave bus=%d addr=%d", f->bus, f->addr);
        return 0;
    }
    if (f->kind == F_I2C && req == I2C_RDWR) {
        struct i2c_rdwr_ioctl_data *x = arg;
        for (unsigned m = 0; m < x->nmsgs; m++) {
            struct i2c_msg *msg = &x->msgs[m];
            if (msg->flags & I2C_M_RD) {
                for (unsigned i = 0; i < msg->len; i++)
                    msg->buf[i] = f->counter + i;
                f->counter++;
                logev("i2c_read bus=%d addr=%d len=%d", f->bus, msg->addr, msg->len);
            } else {
                f->addr = msg->addr;
                hexlog("i2c_write", f, msg->buf, msg->len);
            }
        }
        return x->nmsgs;
    }
    logev("ioctl kind=%d req=%lx", f->kind, req);
    return 0;
}

ssize_t
write(int fd, const void *buf, size_t len)
{
    init();
    struct fake *f = lookup(fd);
    if (!f)
        return real_write(fd, buf, len);
    if (f->kind == F_SPI || f->kind == F_I2C)
        hexlog(f->kind == F_SPI ? "spi_write" : "i2c_write_raw", f, buf, len);
    return len;
}

ssize_t
pread(int fd, void *buf, size_t len, off_t off)
{
    init();
    struct fake *f = lookup(fd);
    if (!f)
        return real_pread(fd, buf, len, off);
    const char *v = "2048\n";
    size_t n = strlen(v) < len ? strlen(v) : len;
    memcpy(buf, v, n);
    logev("adc_read line=%d", f->line);
    return n;
}

ssize_t pread64(int fd, void *buf, size_t len, off_t off) __attribute__((alias("pread")));

int
close(int fd)
{
    init();
    static int (*real_close)(int);
    if (!real_close)
        real_close = dlsym(RTLD_NEXT, "close");
    struct fake *f = lookup(fd);
    if (f) {
        if (f->kind == F_LINE)
            logev("release chip=%d line=%d", f->chip, f->line);
        f->kind = F_NONE;
    }
    return real_close(fd);
}
