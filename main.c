#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#include <termios.h>
#include <sys/time.h>

#if 1
// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(void)
{
    struct termios toptions;
    int fd;
    
	//fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }

    speed_t brate = B4800;
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    toptions.c_cflag |= PARENB;  // Enable parity
    toptions.c_cflag &= ~PARODD; // Even parity
    toptions.c_cflag |= CSTOPB;  // 2 stop bits
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;     // 8 data bits
    toptions.c_cflag &= ~CRTSCTS;  // Disable HW flow control
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable SW flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw

    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 1;
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}
#endif

void write_html(const char *msg)
{
	int fd = open("/var/www/vista.html", (O_CREAT | O_RDWR | O_TRUNC), (S_IRUSR | S_IWUSR | S_IROTH | S_IWOTH));

	char tmp[255];
	int len;
	
	const char hdr[] = "<html><body><head><meta http-equiv=\"refresh\" content=\".5\"></head><pre>\n";
	int rc = write(fd, hdr, (sizeof(hdr) - 1));
	if (rc == -1)
		printf("Log write failure\n");
		
	//len = snprintf(tmp, sizeof(tmp), "<p>%s</p>\n", msg);
	rc = write(fd, msg, 16);
	if (rc == -1)
		printf("Log write failure\n");
	rc = write(fd, "\n", 1);
	if (rc == -1)
		printf("Log write failure\n");
	rc = write(fd, &(msg[16]), 16);
	if (rc == -1)
		printf("Log write failure\n");
		
#if 0
	rc = write(fd, "<html><body>\n", 13);
	if (rc == -1)
		printf("Log write failure\n");
#endif
	
	rc = write(fd, "</pre></body></html>\n", 21);
	if (rc == -1)
		printf("Log write failure\n");
	
}

void dump_message(const uint8_t *msg, int len)
{
	int index;
	for (index = 0; index < len; index++)
		printf("%02X ", msg[index]);
	//printf("\n");
}

// http://www.alarmdecoder.com/wiki/index.php/Protocol#Format for some hints

// keypad message:
//   Byte 0 : 0xF7
//   Bytes 1-4 : keypad mask - which keypad this goes to [7-0,15-8,23-16,31-24]
//   Byte 5 : Numerical display for non-alpha keypads: Faulted zone / countdown / 08 for ready??
//     D1 - Busy standby (startup)
//     20 - Installer code (then digits of installer code)
//     Fx - Field (F for first otherwise first entered digit)
//   Byte 6 : Number of times for keypad to chime
//   Byte 7 : Armed status: 0C=Faulted, 1C=Ready, ??=Armed (Away), 8C=Armed (Stay), 80=Arming, 00=Busy standby
//     0x01=Program data output??
//     0x02=Program date output last byte??
//     0x04=??
//     0x08=??
//     0x0C=disarmed
//     0x10=ready
//     0x20=???
//     0x40=low battery (0x5C low bat + disarmed + ready)
//     0x80=arming/armed
//   Byte 8 : ???
//   Bytes 9-11 : "02 00 00"
//   Bytes 12-43 : Alpha keypad display, 16 characters per line
//   Byte 44 : two's compliment checksum
//
// backlight, programming mode, zone bypassed, AC power, chime enabled, alarm occured, alarm in progress, low battery, entry delay on/off, fire, system issue, Armed (home)

// Keypress (from keypad)
//   Bytes 0-2 : Address?  FF FF FE
//   Byte 3 : remaining bytes
//   Byte 4-N-1 : keypresses
//   Byte N : Two's compliment checksum

// 9E - No idea what this is for
//   Byte 0 : 9E
//   Byte 1 : Data Length?
//   Bytes 2- N-1 : Data??? The F1 seem to correlate to faults
//     25 81
//     25 F1
//     20 60
//     20 82
//     20 F1
//   Byte N : Checksum?  Looks like Two's compliment + 5

int main(int argc, char *argv[])
{
	uint8_t byte[100];
	uint8_t message[256];
	
	int fd = serialport_init();

	//write(fd, "X", 1);

	int state = 0; // Wait for a recognized pattern
	int msgOffset = 0;
	int needed = 0;
	int index;		
	
	int logFd = open("vistalog.bin", (O_CREAT | O_RDWR), (S_IRUSR | S_IWUSR));
	//int txtFd = open("vistalog.txt", (O_CREAT | O_RDWR), (S_IRUSR | S_IWUSR));
	
	while(1)
	{
		ssize_t size = read(fd, byte, 100);
		
		if (size < 0)
			printf("Read error\n");
		if (size == 0)
			continue;
		

		//printf("%zu:", size);
		int rc = write(logFd, byte, size);
		if (rc == -1)
			printf("Log write failure\n");
		
#if 0
		int rc = write(logFd, byte, size);
		if (rc == -1)
			printf("Log write failure\n");
#endif
		
#if 1
		struct timeval time;
		gettimeofday(&time, NULL);
		printf("[%ld.%06ld]", time.tv_sec, time.tv_usec);
		dump_message(byte, size);
		printf("\n");
#else
		for (index = 0; index < size; index++)
		{
			//printf(" %02X", byte[index]);

			switch (state)
			{
			case 0:  // No message
				if (byte[index] == 0xF7)
				{
					// Keypad message
					state = 1;  // Read message
					needed = 44;
					//printf("state = Read keypad message header (%d)\n", needed);
					message[0] = 0xF7;
					msgOffset = 1;
				}
				else if (byte[index] == 0x9E)
				{
					// 9E message, whatever that is
					state = 1;  // Read message
					needed = 4;
					//printf("state = Read 9E message (%d)\n", needed);
					message[0] = 0x9E;
					msgOffset = 1;
				}
				else if (byte[index] == 0xF6)
				{
					// Appears to be a keypress message but without any indication of what key was pressed, maybe byte 1 is keypad address
					state = 1;  // Read message
					needed = 3;
					message[0] = 0xF6;
					msgOffset = 1;
				}
				else if (byte[index] == 0x00)
				{
					// Keep looking
				}
				else
				{
					printf("Unknown message type (0x%02X)\n", byte[index]);					
				}
				break;
				
			case 1:
				message[msgOffset] = byte[index];
				msgOffset++;
				needed--;
				
				if (needed == 0)
				{
					//printf("Read needed bytes...\n");
					// Entire message received
					struct timeval time;
					gettimeofday(&time, NULL);
#if 0
					time_t ltime; /* calendar time */
					ltime=time(NULL); /* get current cal time */
					char timeStr[256];
					strftime(timeStr, sizeof(timeStr), "", localtime(&ltime)));
#endif
					printf("[%ld.%06ld]", time.tv_sec, time.tv_usec);

					// Handle message
					if (message[0] == 0xF7)
					{
						// keypad message
						printf("Keypad : ");
						dump_message(message, 12);
						message[msgOffset - 1] = 0x00;
						message[12] &= 0x7F;  // The upper bit has some unknown meaning, masked for display
						printf("'%s'\n", &(message[12]));
						write_html((char *)&(message[12]));
					}
					else if (message[0] == 0x9E)
					{
						printf("9E : ");
						dump_message(message, msgOffset);
						printf("\n");
					}
					else if (message[0] == 0xF6)
					{
						printf("Keypress : ");
						dump_message(message, msgOffset);
						printf("\n");
					}
					else
					{
						printf("Trying to process unknown message type (0x%02X)\n", message[0]);
					}
					
					state = 0; // Wait for known message pattern
					msgOffset = 0;
					needed = 0;
				}
			}
			
			//printf("\n");
		}
#endif
	}

	return EXIT_SUCCESS;
}
