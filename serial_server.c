#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>

#define IS_CONNECTION(x) if ((x) != -1) continue;

#define ERROR_RETURN -1
#define PORT 8080
#define MAX_BUF_SIZE 5000

static int set_socket(){

    int sock_fd;
    struct sockaddr_in serv_addr, client_info;

    if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket()");
        return ERROR_RETURN;
    }

    bzero(&serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(PORT);

    if ((bind(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) < 0)
    {
        perror("bind()");
        return ERROR_RETURN;
    }

    if ((listen(sock_fd, 1)) < 0)
    {
        perror("listen()");
        return ERROR_RETURN;
    }

    return sock_fd;
}
static int set_serial(){

    int serial_fd;
    struct termios options;

    if((serial_fd = open("/dev/ttyUSB0", O_RDWR)) < 0)
    {
        perror("open()");
        return ERROR_RETURN;
    }
    else
    {
        fcntl(serial_fd, F_SETFL, 0);
    }

    struct serial_struct serial;
    ioctl(serial_fd, TIOCGSERIAL, &serial); 
    serial.flags |= ASYNC_LOW_LATENCY; // (0x2000)
    ioctl(serial_fd, TIOCSSERIAL, &serial);

    /* Setting I/O Baud Rate */
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);
    /* Setting control fleid */
    options.c_cflag |= CREAD; /* Open receive */
    options.c_cflag |= CLOCAL; /* Ignore control line(Avoid occupy port) */
    options.c_cflag &= ~PARENB; /* Disable parity */
    options.c_cflag &= ~CSTOPB; /* Disable two stop bits(only one)*/
    options.c_cflag &= ~CSIZE; /* Clear bit size*/
    options.c_cflag |= CS8; /* Set  8 bits per byte */
    options.c_cflag |= CRTSCTS; /* Hardware flow control */
    /* Setting local field */
    options.c_lflag &= ~ICANON; /* Disable canonical mode*/
    options.c_lflag &= ~ECHO; /* Disable echo */
    options.c_lflag &= ~ECHOE; /* Disable erase */
    options.c_lflag &= ~ECHONL; /* Disable new line echo */
    options.c_lflag &= ~ISIG; /* Disable interpretation of INTR, QUIT, SUSP and DSUSP */

    // options.c_iflag |= (IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~ONLCR;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    if (tcsetattr(serial_fd, TCSANOW, &options) < 0)
    {
        perror("tcsetattr()");
        return ERROR_RETURN;
    }

    return serial_fd;
}
int main(int argc, char **argv)
{
    int sock_fd, serial_fd, conn_fd;
    int is_connected, fdmax, recv_bytes, client_info_len;
    struct sockaddr_in client_info;
    fd_set read_fds_master, read_fds;
    char buf_to_tcp[MAX_BUF_SIZE], buf_to_serial[MAX_BUF_SIZE];
    int window_size = MAX_BUF_SIZE;
    
    if((sock_fd = set_socket()) == ERROR_RETURN)
    {
        goto fd_close;
    }
    if((serial_fd = set_serial()) == ERROR_RETURN)
    {
        goto fd_close;
    }
    
    FD_ZERO(&read_fds_master);
    FD_ZERO(&read_fds);

    /* Add "std_input" and "sock_fd" to read_fd_master set */
    FD_SET(sock_fd, &read_fds_master);
    FD_SET(serial_fd, &read_fds_master);
    /* Initial select() numfds */
    fdmax = serial_fd; 

    memset(buf_to_tcp, 0, MAX_BUF_SIZE);
    memset(buf_to_serial, 0, MAX_BUF_SIZE);
    conn_fd = -1;
    while(1){
        /* Select active fds into read_fds */
        read_fds = read_fds_master;
        if (select(fdmax+1, &read_fds, NULL, NULL, NULL) == -1) {
            perror("select()");
            goto fd_close;
        }
        if (FD_ISSET(sock_fd, &read_fds)) 
        {
            IS_CONNECTION(conn_fd);
            client_info_len = sizeof(client_info);
            if ((conn_fd = accept(sock_fd, (struct sockaddr *)&client_info, &client_info_len)) < 0)
            {
                perror("accept()");
            }
            else 
            {   
                printf("Connection accpet(%d)\n", conn_fd);
                /* Add new "conn_fd" into read_fds */
                FD_SET(conn_fd, &read_fds_master); 
                if (conn_fd > fdmax) 
                {
                    /* Update select() numfds */
                    fdmax = conn_fd;
                }
            }
        }
        if (FD_ISSET(serial_fd, &read_fds))
        {
            
            if ((recv_bytes = read(serial_fd, buf_to_tcp, MAX_BUF_SIZE)) <= 0) 
            {
                if (recv_bytes == 0) /* Connection closed */
                {
                    printf("Serial closed\n");
                    FD_CLR(serial_fd, &read_fds_master); 
                    close(serial_fd);
                } 
                else 
                {
                    perror("Serial read()");
                    goto fd_close;
                }
            }
            else
            {   
                if (write(conn_fd, buf_to_tcp, recv_bytes) == -1)
                {
                    perror("Socket write()");
                    goto fd_close;
                }
                else
                {   
                    window_size += recv_bytes;
                    memset(buf_to_tcp, 0, recv_bytes);
                }
            }
        }
        if (FD_ISSET(conn_fd, &read_fds) && (window_size > 0))
        {   
            if ((recv_bytes = read(conn_fd, buf_to_serial, window_size)) <= 0)
            {
                if (recv_bytes == 0) /* Connection closed */
                {
                    printf("<Socket %d Closed>\n", conn_fd);
                    FD_CLR(conn_fd, &read_fds_master); 
                    close(conn_fd);
                    conn_fd = -1;
                } 
                else 
                {
                    perror("Socket read()");
                    goto fd_close;
                }
            }
            else
            {
                if (write(serial_fd, buf_to_serial, recv_bytes) < 0)
                {
                    perror("Serial write()");
                    goto fd_close;
                }
                else
                {
                    window_size -= recv_bytes;
                    memset(buf_to_serial, 0, recv_bytes);
                    if(window_size < 0)
                    {
                        fprintf(stderr, "window size error\n");
                        goto fd_close;
                    }
                }
            }
        }
    }
fd_close:
    if (sock_fd)
    {
        close(sock_fd);
    }
    if (serial_fd)
    {
        close(serial_fd);
    }
    if (conn_fd)
    {
        close(conn_fd);
    }
    
    return 0;
}
