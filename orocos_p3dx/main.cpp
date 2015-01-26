#include "packet.h"
#include <fcntl.h>    

#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <errno.h>
#include <assert.h>
#include <cctype>

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

}



int main(int argc, char** argv)
{

	char *portname = "/dev/ttyS0";


	Packet packet;    
	uint8_t data[1];
	uint8_t data_len = 1;
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

	if (fd < 0)
	{
		std::cerr << "error while opening serial device!";
		return -1;
	}

	set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 1);                // set no blocking


	uint8_t buf [6];

	*data = 0;
	packet.serialize(data, data_len);
	std::cout << std::endl;
	write(fd, packet.packet, packet.size);

	usleep((packet.size + 25) * 100);
	int n = 0;
	n = read(fd, buf, 6);
	assert(buf != 0);
	std::cout << n << std::endl;
	for(int i = 0; i < n; i++)
		std::cout << (unsigned int)buf[i] << " ";
	std::cout << std::endl;
	if(buf[0] != 250 || buf[1] != 251) 
	{
		std::cerr << "received incorrect response!" << std::endl;
		return -1;

	}
	*data = 1;
	packet.serialize(data, data_len);
	write(fd, packet.packet, packet.size);

	usleep((packet.size + 25) * 100);
	n = read(fd, buf, 6);

	for(int i = 0; i < n; i++)
		std::cout << (unsigned int)buf[i] << " ";
	std::cout << std::endl;

	*data = 2;
	packet.serialize(data, data_len);
	write(fd, packet.packet, packet.size);
	usleep((packet.size + 25) * 100);
	uint8_t buf2[100];
	n = read(fd, buf2, sizeof(buf2));
	for(int i = 0; i < n-1;i++){
		if(isdigit(buf2[i]))
			std::cout << (int)buf2[i] << " ";
		std::cout << buf2[i];

	}


	while(1){
		*data = 0;		
		packet.serialize(data, data_len);
		write(fd, packet.packet, packet.size);
		sleep(1);
	}

	return 0;
}
