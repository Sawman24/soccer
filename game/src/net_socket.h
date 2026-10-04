#ifndef NET_SOCKET_H
#define NET_SOCKET_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #ifdef DrawText
    #undef DrawText
  #endif
  #ifdef CloseWindow
    #undef CloseWindow
  #endif
  #ifdef ShowCursor
    #undef ShowCursor
  #endif
  #ifdef PlaySound
    #undef PlaySound
  #endif
  #ifdef Rectangle
    #undef Rectangle
  #endif
  typedef SOCKET NetSocket;
  #define NET_INVALID_SOCKET INVALID_SOCKET
  #define NET_SOCKET_ERROR   SOCKET_ERROR
  typedef int net_socklen_t;
#else
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <errno.h>
  typedef int NetSocket;
  #define NET_INVALID_SOCKET (-1)
  #define NET_SOCKET_ERROR   (-1)
  #define closesocket(s)     close(s)
  typedef socklen_t net_socklen_t;
#endif

typedef struct NetAddr {
    struct sockaddr_in in;
} NetAddr;

static inline int net_init(void)
{
#ifdef _WIN32
    WSADATA wsa;
    return WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
#else
    return 1;
#endif
}

static inline void net_shutdown(void)
{
#ifdef _WIN32
    WSACleanup();
#endif
}

static inline NetAddr net_addr_create(const char *ip_str, int port)
{
    NetAddr addr;
    memset(&addr, 0, sizeof(addr));
    addr.in.sin_family = AF_INET;
    addr.in.sin_port = htons((unsigned short)port);
    if (!ip_str || strcmp(ip_str, "") == 0 || strcmp(ip_str, "0.0.0.0") == 0) {
        addr.in.sin_addr.s_addr = INADDR_ANY;
    } else if (strcmp(ip_str, "localhost") == 0) {
        addr.in.sin_addr.s_addr = inet_addr("127.0.0.1");
    } else {
        addr.in.sin_addr.s_addr = inet_addr(ip_str);
    }
    return addr;
}

static inline void net_addr_to_string(const NetAddr *addr, char *out, int maxlen)
{
    if (!addr || !out || maxlen <= 0) return;
    char ip[INET_ADDRSTRLEN] = { 0 };
    const char *p = inet_ntop(AF_INET, &addr->in.sin_addr, ip, sizeof(ip));
    if (!p) p = "unknown";
    snprintf(out, maxlen, "%s:%d", p, ntohs(addr->in.sin_port));
}

static inline int net_addr_equal(const NetAddr *a, const NetAddr *b)
{
    if (!a || !b) return 0;
    return a->in.sin_port == b->in.sin_port &&
           a->in.sin_addr.s_addr == b->in.sin_addr.s_addr;
}

static inline NetSocket net_socket_create(int port, int non_blocking)
{
    NetSocket s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == NET_INVALID_SOCKET) return NET_INVALID_SOCKET;

#ifdef _WIN32
    u_long mode = non_blocking ? 1 : 0;
    if (ioctlsocket(s, FIONBIO, &mode) != 0) {
        closesocket(s);
        return NET_INVALID_SOCKET;
    }
#else
    int flags = fcntl(s, F_GETFL, 0);
    if (non_blocking) flags |= O_NONBLOCK; else flags &= ~O_NONBLOCK;
    if (fcntl(s, F_SETFL, flags) != 0) {
        closesocket(s);
        return NET_INVALID_SOCKET;
    }
#endif

    /* Enable broadcast if needed */
    int broadcast = 1;
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, (const char *)&broadcast, sizeof(broadcast));

    /* Bind if port specified */
    if (port > 0) {
        struct sockaddr_in bind_addr;
        memset(&bind_addr, 0, sizeof(bind_addr));
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_addr.s_addr = INADDR_ANY;
        bind_addr.sin_port = htons((unsigned short)port);
        if (bind(s, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == NET_SOCKET_ERROR) {
            closesocket(s);
            return NET_INVALID_SOCKET;
        }
    }

    return s;
}

static inline void net_socket_close(NetSocket s)
{
    if (s != NET_INVALID_SOCKET) {
        closesocket(s);
    }
}

static inline int net_socket_send(NetSocket s, const NetAddr *to, const void *buf, int len)
{
    if (s == NET_INVALID_SOCKET || !to || !buf || len <= 0) return -1;
    return sendto(s, (const char *)buf, len, 0, (const struct sockaddr *)&to->in, sizeof(to->in));
}

static inline int net_socket_recv(NetSocket s, NetAddr *from, void *buf, int max_len)
{
    if (s == NET_INVALID_SOCKET || !buf || max_len <= 0) return -1;
    net_socklen_t from_len = sizeof(from->in);
    int res = recvfrom(s, (char *)buf, max_len, 0, (struct sockaddr *)&from->in, &from_len);
    if (res < 0) {
#ifdef _WIN32
        int err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK || err == WSAECONNRESET) return 0;
#else
        if (errno == EWOULDBLOCK || errno == EAGAIN) return 0;
#endif
        return -1;
    }
    return res;
}

#endif /* NET_SOCKET_H */
