/*
 * TAP Interface for CYW43 WiFi Bridge
 *
 * Linux TAP device creation and non-blocking I/O.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "tapif.h"

#ifdef __linux__
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/if_tun.h>

int tapif_open(const char *name) {
    int fd = open("/dev/net/tun", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[TAP] Failed to open /dev/net/tun: %s\n", strerror(errno));
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    if (name && name[0])
        strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);

    if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
        fprintf(stderr, "[TAP] TUNSETIFF failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Set non-blocking */
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    fprintf(stderr, "[TAP] Interface '%s' opened (fd=%d)\n", ifr.ifr_name, fd);
    return fd;
}

#else /* !__linux__ */

int tapif_open(const char *name) {
    (void)name;
    fprintf(stderr, "[TAP] TAP interface only supported on Linux\n");
    return -1;
}

#endif /* __linux__ */

void tapif_close(int fd) {
    if (fd >= 0) close(fd);
}

int tapif_read(int fd, uint8_t *buf, int maxlen) {
    if (fd < 0) return -1;
    ssize_t n = read(fd, buf, maxlen);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    return (int)n;
}

int tapif_write(int fd, const uint8_t *buf, int len) {
    if (fd < 0) return -1;
    ssize_t n = write(fd, buf, len);
    return (int)n;
}
