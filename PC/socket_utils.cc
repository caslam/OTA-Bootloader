#include "socket_utils.h"

#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include <iostream>

using std::cerr;
using std::endl;

int Listen(char *portnum, int *sock_family) {
  // Populate the "hints" addrinfo structure for getaddrinfo().
  struct addrinfo hints;
  memset(&hints, 0, sizeof(struct addrinfo));
  hints.ai_family = AF_INET6;       // IPv6 (also handles IPv4 clients)
  hints.ai_socktype = SOCK_STREAM;  // stream
  hints.ai_flags = AI_PASSIVE;      // use wildcard "in6addr_any" address
  hints.ai_flags |= AI_V4MAPPED;    // use v4-mapped v6 if no v6 found
  hints.ai_protocol = IPPROTO_TCP;  // TCP protocol
  hints.ai_canonname = nullptr;
  hints.ai_addr = nullptr;
  hints.ai_next = nullptr;

  struct addrinfo *result;
  int res = getaddrinfo(nullptr, portnum, &hints, &result);

  if (res != 0) {
    cerr << "getaddrinfo() failed: ";
    cerr << gai_strerror(res) << endl;
    return -1;
  }

  // Loop through the returned address structures until we are able
  // to create a socket and bind to one. The address structures are
  // linked in a list through the "ai_next" field of result.
  int listen_fd = -1;
  for (struct addrinfo *rp = result; rp != nullptr; rp = rp->ai_next) {
    listen_fd = socket(rp->ai_family,
                       rp->ai_socktype,
                       rp->ai_protocol);
    if (listen_fd == -1) {
      // Creating this socket failed. So, loop to the next returned
      // result and try again.
      cerr << "socket() failed: " << strerror(errno) << endl;
      continue;
    }

    // Configure the socket.
    int optval = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR,
               &optval, sizeof(optval));

    // Try binding the socket to the address and port number returned
    // by getaddrinfo().
    if (bind(listen_fd, rp->ai_addr, rp->ai_addrlen) == 0) {
      // Return to the caller the address family.
      *sock_family = rp->ai_family;
      break;
    }

    // The bind failed. Close the socket, then loop back around and
    // try the next address/port returned by getaddrinfo().
    close(listen_fd);
    listen_fd = -1;
  }
  
  freeaddrinfo(result);

  if (listen_fd == -1) {
    cerr << "Failed to bind." << endl;
    return -1;
  }

  // Successful bind. Mark as a listening socket.
  if (listen(listen_fd, SOMAXCONN) != 0) {
    cerr << "Failed to mark socket as listening: ";
    cerr << strerror(errno) << endl;
    close(listen_fd);
    return -1;
  }

  // Return to the client the listening socket file descriptor.
  return listen_fd;
}

int WrappedRead(int fd, unsigned char *buf, int readlen) {
  int res;
  while (1) {
    res = read(fd, buf, readlen);
    if (res == -1) {
      if ((errno == EAGAIN) || (errno == EINTR))
        continue;
    }
    break;
  }
  return res;
}

int WrappedWrite(int fd, unsigned char *buf, int writelen) {
  int res, written_so_far = 0;
  while (written_so_far < writelen) {
    res = write(fd, buf + written_so_far, writelen - written_so_far);
    if (res == -1) {
      if ((errno == EAGAIN) || (errno == EINTR)) {
        continue;
      }
      break;
    }
    if (res == 0)
      break;
    written_so_far += res;
  }
  return written_so_far;
}
