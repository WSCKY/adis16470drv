/*
 * uart.c
 *
 *  Created on: Feb 18, 2019
 *      Author: kychu
 */

#include "uart.h"

#define NUM_SUPPORT_BAUD               8
int org_baud[NUM_SUPPORT_BAUD] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000};
int cvt_baud[NUM_SUPPORT_BAUD] = {B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000};

static int uart_fd = -1;

static long block_s = 1, block_us = 0;

//static pthread_t recv_t;
//static int auto_recv_flag = 0;
//static int auto_recv_exit = 0;
//static void recv_thread(void);
//static UartRecvByteCallback pCallback = 0;

static int SetOption(int fd, int nSpeed, int nBits, char nEvent, int nStop);
static int BaudrateConvert(int speed);

int uart_open(const char *dev, const char *baud)
{
	int baudrate = B115200;
//	printf("baud: %d\n", atoi(baud));
//	fflush(stdout);
	baudrate = BaudrateConvert(atoi(baud));
	uart_fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(uart_fd == -1) { perror("Can not open Serial Port!"); return EXIT_FAILURE; }
	if(SetOption(uart_fd, baudrate, 8, 'N', 1) != 0) { return EXIT_FAILURE; };
	return EXIT_SUCCESS;
}

int uart_baudrate(const char *baud)
{
	int baudrate = BaudrateConvert(atoi(baud));
	if(SetOption(uart_fd, baudrate, 8, 'N', 1) != 0) { return EXIT_FAILURE; };
	return EXIT_SUCCESS;
}

int uart_write(char *p, size_t l)
{
	if(uart_fd == -1) return -1;
	return write(uart_fd, p, l);
}

void uart_flush_read(void)
{
	tcflush(uart_fd, TCIFLUSH); /* flushes data received but not read. */
}

void uart_flush_write(void)
{
	tcflush(uart_fd, TCOFLUSH); /* flushes data written but not transmitted. */
}

//int uart_read(char *p, size_t l)
//{
//	return read(uart_fd, p, l);
//}

int uart_read(char *p, size_t l)
{
	if(uart_fd == -1) return -1;
	int ret;
	fd_set rfds;
	struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(uart_fd, &rfds);
    tv.tv_sec = block_s;
    tv.tv_usec = block_us;
    ret = select(uart_fd + 1, &rfds ,NULL, NULL, &tv);
    switch(ret) {
    case -1:
    	perror("Read Serial Port Failed!");
    	return -1;
    case 0:
    	return 0;
    default:
    	return read(uart_fd, p, l);
    }
}

int uart_block_time(long sec, long us)
{
  block_s = sec;
  block_us = us;
  return 0;
}

void uart_close(void)
{
//	if(auto_recv_flag != 0) { // auto recv enabled.
//		if(auto_recv_exit == 0) {
//			auto_recv_exit = 1; // exit recv thread.
//			pthread_join(recv_t, NULL); // wait for thread exit.
//		}
//	}
	close(uart_fd);
	uart_fd = -1;
}

int uart_isopen(void)
{
	return (uart_fd != -1);
}

//int uart_auto_recv_enable(void)
//{
//	if(auto_recv_flag != 0) return 0; /* already initialized. */
//	if(pthread_create(&recv_t, NULL, (void *)recv_thread, NULL) != 0) {
//		return -1;
//	}
//	auto_recv_flag = 1;
//	return 0;
//}

//void uart_set_recv_callback(UartRecvByteCallback p) {
//	if(p != 0) {
//		pCallback = p;
//	}
//}

//static void recv_thread(void)
//{
//	int idx = 0, len = 0;
//	char recv[512];
//	while(auto_recv_exit == 0) {
//		if((len = uart_read(recv, 512)) > 0) {
//			if(pCallback != 0) {
//				for(idx = 0; idx < len; idx ++) {
//					pCallback(recv[idx]);
////					printf("0x%2x, ", (unsigned char)recv[idx]);
//				}
//			}
//		}
//	}
//}

static int SetOption(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios uart_config;
	/* fill the structure for the new configuration */
	if(tcgetattr(fd, &uart_config) != 0) {
		perror("get serial port configuration failed!");
		return -1;
	}
	bzero(&uart_config, sizeof(uart_config));
	uart_config.c_cflag |= CLOCAL | CREAD;
	uart_config.c_cflag &= ~CSIZE;
	switch( nBits ) { /* set Bits */
		case 7: uart_config.c_cflag |= CS7; break;
		case 8: uart_config.c_cflag |= CS8; break;
	}
	switch( nEvent ) { /* set Event */
		case 'o':
		case 'O':
			uart_config.c_cflag |= PARENB;
			uart_config.c_cflag |= PARODD;
			uart_config.c_iflag |= (INPCK | ISTRIP);
		break;
		case 'e':
		case 'E':
			uart_config.c_iflag |= (INPCK | ISTRIP);
			uart_config.c_cflag |= PARENB;
			uart_config.c_cflag &= ~PARODD;
		break;
		case 'n':
		case 'N':
			uart_config.c_cflag &= ~PARENB;
		break;
		default: break;
	}
	/* set Speed */
	cfsetispeed(&uart_config, nSpeed);
	cfsetospeed(&uart_config, nSpeed);
	/* set StopBits */
	if( nStop == 1 ) uart_config.c_cflag &= ~CSTOPB;
	else if ( nStop == 2 ) uart_config.c_cflag |= CSTOPB;
	uart_config.c_cc[VTIME]  = 0;
	uart_config.c_cc[VMIN] = 0;
	/* flush unprocessed characters */
	tcflush(fd, TCIFLUSH);
	if((tcsetattr(fd, TCSANOW, &uart_config)) != 0) {
		perror("configure serial port failed!");
		return -1;
	}
	return 0;
}

static int BaudrateConvert(int speed)
{
	for(int i = 0; i < NUM_SUPPORT_BAUD; i ++) {
		if(speed == org_baud[i]) return cvt_baud[i];
	}
	printf("Warning: Unsupported Speed! Set to default: B115200.\n");
	return B115200;
}
