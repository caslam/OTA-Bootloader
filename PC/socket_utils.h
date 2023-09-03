#ifndef SOCKET_UTILS_H_
#define SOCKET_UTILS_H_

// Takes a string representation of a port number to use for a TCP
// connection and attempts to open a listening socket on the network at that
// port, and an int * as a return parameter representing the family of the
// address bound to the returned listening socket. Returns the file
// descriptor of the listening socket if created successfully, or -1 otherwise. 
int Listen(char *portnum, int *sock_family);

// Wraps read() and write() calls from the given file descriptor into "buf"
// in a loop to handle recoverable errors. Returns the number of bytes written
// or read, or -1 on error.
int WrappedRead(int fd, unsigned char *buf, int readlen);
int WrappedWrite(int fd, unsigned char *buf, int writelen);

#endif