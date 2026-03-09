/*
 * Bramble Wire Protocol - Inter-Instance Communication
 *
 * Uses Unix domain sockets for low-latency IPC between emulator instances.
 * First instance to access a socket path creates it (server).
 * Second instance connects (client). Both are then peers.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "wire.h"
#include "uart.h"
#include "gpio.h"

wire_state_global_t wire_state;

static void set_nonblock(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

int wire_add_link(const char *path, uint8_t type, uint8_t channel) {
    if (wire_state.link_count >= WIRE_MAX_LINKS) return -1;
    wire_link_t *l = &wire_state.links[wire_state.link_count++];
    l->state = WIRE_NONE;
    l->listen_fd = -1;
    l->peer_fd = -1;
    strncpy(l->path, path, sizeof(l->path) - 1);
    l->path[sizeof(l->path) - 1] = '\0';
    l->type = type;
    l->channel = channel;
    return 0;
}

static int try_connect(const char *path) {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    set_nonblock(fd);
    return fd;
}

static int create_listen(const char *path) {
    /* Remove stale socket file */
    unlink(path);

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    if (listen(fd, 1) < 0) {
        close(fd);
        unlink(path);
        return -1;
    }

    set_nonblock(fd);
    return fd;
}

int wire_init(void) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];

        /* Try to connect first (peer already listening?) */
        l->peer_fd = try_connect(l->path);
        if (l->peer_fd >= 0) {
            l->state = WIRE_CONNECTED;
            fprintf(stderr, "[Wire] %s: connected to peer\n", l->path);
            continue;
        }

        /* No peer yet — create server socket */
        l->listen_fd = create_listen(l->path);
        if (l->listen_fd < 0) {
            fprintf(stderr, "[Wire] %s: failed to create socket: %s\n",
                    l->path, strerror(errno));
            return -1;
        }
        l->state = WIRE_LISTEN;
        fprintf(stderr, "[Wire] %s: waiting for peer\n", l->path);
    }
    return 0;
}

void wire_cleanup(void) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->peer_fd >= 0) {
            close(l->peer_fd);
            l->peer_fd = -1;
        }
        if (l->listen_fd >= 0) {
            close(l->listen_fd);
            l->listen_fd = -1;
            unlink(l->path);
        }
    }
}

static void wire_handle_message(wire_link_t *l, wire_msg_t *msg, uint8_t *payload) {
    switch (msg->type) {
    case WIRE_MSG_UART_DATA:
        if (msg->len >= 1) {
            uart_rx_push(msg->channel, payload[0]);
        }
        break;

    case WIRE_MSG_GPIO_PIN:
        if (msg->len >= 2) {
            gpio_set_input_pin(payload[0], payload[1]);
        }
        break;

    default:
        break;
    }
    (void)l;
}

void wire_poll(void) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];

        /* Accept pending connections */
        if (l->state == WIRE_LISTEN && l->listen_fd >= 0) {
            int cfd = accept(l->listen_fd, NULL, NULL);
            if (cfd >= 0) {
                set_nonblock(cfd);
                l->peer_fd = cfd;
                l->state = WIRE_CONNECTED;
                fprintf(stderr, "[Wire] %s: peer connected\n", l->path);
            }
        }

        /* Read messages from peer */
        if (l->state == WIRE_CONNECTED && l->peer_fd >= 0) {
            struct pollfd pfd = { .fd = l->peer_fd, .events = POLLIN };
            while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
                wire_msg_t msg;
                ssize_t n = read(l->peer_fd, &msg, sizeof(msg));
                if (n == sizeof(msg) && msg.len <= 8) {
                    uint8_t payload[8];
                    if (msg.len > 0) {
                        ssize_t pn = read(l->peer_fd, payload, msg.len);
                        if (pn != msg.len) break;
                    }
                    wire_handle_message(l, &msg, payload);
                } else if (n <= 0) {
                    fprintf(stderr, "[Wire] %s: peer disconnected\n", l->path);
                    close(l->peer_fd);
                    l->peer_fd = -1;
                    l->state = WIRE_LISTEN;
                    break;
                }
            }
        }
    }
}

static wire_link_t *find_link(uint8_t type, uint8_t channel) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->type == type && l->channel == channel && l->state == WIRE_CONNECTED) {
            return l;
        }
    }
    return NULL;
}

static void wire_send(wire_link_t *l, uint8_t type, uint8_t channel,
                       const uint8_t *data, uint8_t len) {
    if (!l || l->peer_fd < 0) return;

    wire_msg_t msg = { .type = type, .channel = channel, .len = len, .reserved = 0 };
    /* Send header + payload atomically (small enough for pipe buffer) */
    uint8_t buf[sizeof(wire_msg_t) + 8];
    memcpy(buf, &msg, sizeof(msg));
    if (len > 0 && data) memcpy(buf + sizeof(msg), data, len);

    ssize_t n = write(l->peer_fd, buf, sizeof(msg) + len);
    if (n < 0 && errno != EAGAIN) {
        fprintf(stderr, "[Wire] Write error, peer disconnected\n");
        close(l->peer_fd);
        l->peer_fd = -1;
        l->state = WIRE_LISTEN;
    }
}

void wire_send_uart(int uart_num, uint8_t byte) {
    wire_link_t *l = find_link(WIRE_MSG_UART_DATA, (uint8_t)uart_num);
    if (l) wire_send(l, WIRE_MSG_UART_DATA, (uint8_t)uart_num, &byte, 1);
}

void wire_send_gpio(uint8_t pin, uint8_t value) {
    uint8_t payload[2] = { pin, value };
    /* Send to all GPIO wire links */
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->type == WIRE_MSG_GPIO_PIN && l->state == WIRE_CONNECTED) {
            wire_send(l, WIRE_MSG_GPIO_PIN, 0, payload, 2);
        }
    }
}

int wire_uart_active(int uart_num) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->type == WIRE_MSG_UART_DATA && l->channel == (uint8_t)uart_num) {
            return 1;
        }
    }
    return 0;
}
