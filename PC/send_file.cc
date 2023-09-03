#include <fcntl.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>

#include "socket_utils.h"

// Size of the reading/writing buffer.
#define BUF_SIZE 1024

// Maximum file size of the program to be loaded on the STM32F103.
#define MAX_FILE_SIZE 114688

// Byte sent by the ESP32 to request data.
#define RX_BYTE 0xEE

// Byte sent to the ESP32 to indicate a file is ready to be sent.
#define TX_BYTE 0xEE

using std::cerr;
using std::cout;
using std::endl;

// Prints usage information about this program and exits with EXIT_FAILURE.
void Usage(char *progname);

// Takes 3 ints representing the fd of a client socket, the fd of a local file
// to write to the client, and the size local file, and sends the file size and
// contents to the client upon receipt of RX_BYTE until the connection is 
// dropped or EOF is reached. Returns false on failure to send the full file
// contents, or true on success.
bool HandleClient(int client_fd, int file_fd, int file_size);

// This program takes 2 arguments on the command line: a port number and a
// local file name (the program to be sent to the STM32F103). The file is
// expected to be a raw binary file (no debugging info) and no more than 112kB
// in size. Creates a listening socket, accepting one connection from a client
// at a time, and writes data from the provided file to the ESP32.
int main(int argc, char **argv) {
    // Verify correct number of arguments.
    if (argc != 3) {
        Usage(argv[0]);
    }

    // Verify that a valid port argument was given.
    unsigned short port = 0;
    if (sscanf(argv[1], "%hu", &port) != 1) {
        Usage(argv[0]);
    }

    struct stat file_stat;
    if (stat(argv[2], &file_stat)) {
        cerr << "stat() failed: " << strerror(errno) << endl;
        return EXIT_FAILURE;
    }
    int file_size = file_stat.st_size;
    if (file_size > MAX_FILE_SIZE) {
        cerr << "Maximum file size: " << MAX_FILE_SIZE << endl;
        return EXIT_FAILURE;
    }

    // Attempt to open the given local file.
    int file_fd;
    file_fd = open(argv[2], O_RDONLY);
    if (file_fd == -1) {
        cerr << "open() failed: " << strerror(errno) << endl;
        return EXIT_FAILURE;
    }

    // Attempt to create and bind a socket for a local IP address at
    // the given port and set it up to listen for client connections.
    int sock_family;
    int listen_fd = Listen(argv[1], &sock_family);
    if (listen_fd == -1) {
        return EXIT_FAILURE;
    }

    // Loop forever, accepting a connection from a client.
    while (1) {
        struct sockaddr_storage caddr;
        socklen_t caddr_len = sizeof(caddr);
        int client_fd = accept(listen_fd,
                               reinterpret_cast<struct sockaddr*>(&caddr),
                               &caddr_len);
        if (client_fd < 0) {
            if ((errno == EINTR) || (errno == EAGAIN) || (errno == EWOULDBLOCK)) {
                continue;
            }
            break;
        }
	    cout << "Client connected." << endl;
        bool sent = HandleClient(client_fd, file_fd, file_size);
        close(listen_fd);
        close(client_fd);
        close(file_fd);
        if (sent) {
            cout << "Done sending." << endl;
            return EXIT_SUCCESS;
        } else {
            return EXIT_FAILURE;
        }
    }
    close(listen_fd);
    close(file_fd);
    return EXIT_FAILURE;
}

void Usage(char *progname) {
    cerr << "Usage: " << progname << " <port> <file>" << endl;
    exit(EXIT_FAILURE);
}

bool HandleClient(int client_fd, int file_fd, int file_size) {
    unsigned char buf[BUF_SIZE];
    unsigned char rec_byte;
    int rres, wres;
    int left_to_write = file_size;

    rec_byte = TX_BYTE;

    // Send "TX_BYTE" when a client connects. At the first "RX_BYTE" received,
    // send the file size as a 4-byte int in network order. 
    while (1) {
        if (WrappedWrite(client_fd, &rec_byte, 1) != 1) {
            return false;
        }
        if (WrappedRead(client_fd, &rec_byte, 1) < 0) {
            return false;
        }
        if (rec_byte == RX_BYTE) {
            break;
        }
    }
    ((int *)buf)[0] = htonl(file_size);
    wres = WrappedWrite(client_fd, buf, sizeof(int));
    if (wres != sizeof(int)) {
        return false;
    }

    // For subsequent requests, send the file in "BUF_SIZE" sections.
    while (left_to_write > 0) {
        while (1) {
            if (WrappedRead(client_fd, &rec_byte, 1) < 0) {
                return false;
            }
            if (rec_byte == RX_BYTE) {
                break;
            }
        }
        rres = WrappedRead(file_fd, buf, BUF_SIZE);
        if (rres == 0)   // eof
            break;
        if (rres < 0) {  // error
            return false;
        }

        wres = WrappedWrite(client_fd, buf, rres);
        if (wres != rres) {
            return false;
        }
        left_to_write -= wres; 
    }
    return true;
}
