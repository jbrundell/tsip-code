// C library headers
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define BUFFER 4096


#define getsb(buf, off)	((int8_t)buf[off])
#define getub(buf, off)	((uint8_t)buf[off])
#define getbes16(buf, off)	((int16_t)(((uint16_t)getub(buf, (off)) << 8) | (uint16_t)getub(buf, (off)+1)))
#define getbeu16(buf, off)	((uint16_t)(((uint16_t)getub(buf, (off)) << 8) | (uint16_t)getub(buf, (off)+1)))
#define getbeu24(buf, off)	((uint32_t)(((uint16_t)getbeu16(buf, (off)) << 8) | getub(buf, (off)+2)))
#define getbes32(buf, off)	((int32_t)(((uint16_t)getbeu16(buf, (off)) << 16) | getbeu16(buf, (off)+2)))
#define getbeu32(buf, off)	((uint32_t)(((uint16_t)getbeu16(buf, (off)) << 16) | getbeu16(buf, (off)+2)))
#define getbes64(buf, off)	((int64_t)(((uint64_t)getbeu32(buf, (off)) << 32) | getbeu32(buf, (off)+4)))
#define getbeu64(buf, off)	((uint64_t)(((uint64_t)getbeu32(buf, (off)) << 32) | getbeu32(buf, (off)+4)))

union int_float {
    int32_t i;
    float f;
};

union long_double {
    int64_t l;
    double d;
};
float getbef32(const char *buf, int off)
{
    union int_float i_f;

    i_f.i = getbes32(buf, off);
    return i_f.f;
}

double getbed64(const char *buf, int off)
{
    union long_double l_d;

    l_d.l = getbes64(buf, off);
    return l_d.d;
}

int gVerbose = 0;
//TODO
//baud rate and serial port options needed
//
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE B115200

int parse(unsigned char *buffer, int length)
{
unsigned int id;
uint8_t u1,u2,u3,u4,u5,u6;
int8_t b1,b2;
int16_t s1,s2;
uint32_t ul1;
uint16_t us1;
int32_t sl1;
double latitude,longitude,altitude,d1,d2;
float temperature;
float f1,f2,f3,f4;

int idx;
unsigned char *buf;
printf("Received %d byte message: [",length);

for(idx=0;idx<length;idx++)
	printf("%02x ",buffer[idx]);
printf("]\n");
if (buffer[0]!=0x10)
	return 1;

buf = buffer+1; //skip 0x10 start byte
id=buf[0];
printf("Packet ID 0x%02X",id);
switch (id) {
case 0x41: 
	printf(" GPS Time\n");
	//0-3 GPS time of week Single seconds
	//4-5 Extended GPS week number INT16 weeks
	//6-9 GPS UTC offset Single seconds
	f1 = getbef32(buf,1); //GPS time of week Single seconds
	printf("GPS time of week: %.2f s\n",f1);
	s1 = getbeu16(buf,5); //Extended GPS week number
	printf("Extended GPS week number: %d weeks\n",s1);
	f1 = getbef32(buf,7); //GPS UTC offset
	printf("GPS UTC offset: %.1f s\n",f1);
	break;

case 0x46:
	u1 = getub(buf,1); //status code
	//0x00 Doing position fixes
	//0x01 Don't have GPS time yet
	//0x02 Reserved
	//0x03 PDOP is too high
	//0x04 The chosen SV is unusable
	//0x08 No usable satellites
	//0x09 Only 1 usable satellite
	//0x0A Only 2 usable satellites
	//0x0B Only 3 usable satellites
	//0xBB Over Determined Mode
	printf(" Health of Receiver\nStatus: 0x%02x, ", u1);	
	switch (u1) {
		case 0: printf("Doing position fixes\n"); break;
		case 1: printf("Don't have GPS time yet\n"); break;
		case 2: printf("Reserved\n"); break;
		case 3: printf("PDOP is too high\n"); break;
		case 4: printf("The chosen SV is unusable\n"); break;
		case 8: printf("No usable satellites\n"); break;
		case 9: printf("Only 1 usable satellite\n"); break;
		case 0x0A: printf("Only 2 usable satellites\n"); break;
		case 0x0B: printf("Only 3 usable satellites\n"); break;
		case 0xBB: printf("Over Determined Mode\n"); break;
		default: printf("Unknown status\n");break;
		}

	u1 = getub(buf,2); 
	
// 0 Battery backup Bit 0 OK
//                      1 BBRAM was not available at start-up
// 4 Antenna feedline fault Bit 0 OK
//                              1 Short or open detected
// 5 Type of fault  Bit 0 Open detected
//                      1 Short detected
	printf("Battery backup: ");
	if (u1 & 0x01) 
		printf("BBRAM was not available at start-up\n");
	else	printf("OK\n");
	printf("Antenna feedline: ");

	if (u1 & 0x10) { //Short or open detected
		if (u1 & 0x20)
			printf("Short detected\n");
		else 
			printf("Open detected\n");
		}
	else	
		printf("OK\n");



	break;

case 0x4B:
	printf(" Machine/Code ID and Additional Status\n");
	break;

case 0x47:
	printf(" Signal Level for All Satellites Tracked\n");
	u1 = getub(buf,1); //satellite count
	printf("%d Satellites Tracked\nSatellite number, Signal level\n",u1);
	idx = 2;
	while (u1 > 0 && (idx < length)){
		printf("%d, %.2f\n",getub(buf,idx),getbef32(buf,idx+1));
		idx += 5; 
		u1--;
	}
		
	printf("\n");
	break;

case 0x54:
	printf(" One Satellite Bias and Bias Rate Report\n");
	f1 = getbef32(buf,1); //One satellite bias, in meters
	printf("One satellite bias: %.2f m\n",f1);
	f1 = getbef32(buf,5); //Clock bias rate, in meters per second
	printf("Clock bias rate: %.2f m/s\n",f1);
	f1 = getbef32(buf,9); //Time of position fix, in GPS seconds
	printf("GPS Time of position fix: %.2f s\n",f1);
	break;

case 0x56:
	printf(" Velocity Fix, East-North-Up (ENU)\n");
	break;

case 0x5F:
	printf(" Trimble Diagnostic Use Only. Please ignore.\n");
	break;

case 0x6C:
	printf(" Satellite Selection List\n");
	u1 = getub(buf,1); // fix flags 
	f1 = getbef32(buf,2); //PDOP
	printf("PDOP: %.2f, ",f1);
	f1 = getbef32(buf,6); //HDOP
	printf("HDOP: %.2f, ",f1);
	f1 = getbef32(buf,10); //VDOP
	printf("VDOP: %.2f, ",f1);
	f1 = getbef32(buf,14); //TDOP
	printf("TDOP: %.2f\n",f1);

	u1 = getub(buf,18);
	printf("%d SVs in fix ",u1);
	idx = 0;
	while (idx<u1 && (idx+19 < length))
		printf("%d ",getub(buf,19+idx++));
	printf("\n");
	
	break;

case 0x6D:
	printf(" All-In-View Satellite Selection Report\n");
	u1 = getub(buf,1); //mode byte
	
	b1 = (u1 & 0xF0)>>4; //number of SVs
	b2 = (u1 & 0x07); //fix dimension
	printf("Mode: 0x%02X (",u1);

	//1 1D clock fix
	//3 2D fix
	//4 3D fix
	//5 OD clock fix
	switch (b2) {
		case 1: printf("1D clock fix, "); break;
		case 3: printf("2D fix, "); break;
		case 4: printf("3D fix, "); break;
		case 5: printf("OD clock fix, "); break;
		default: printf("unknown fix dimension, "); break;
	}

	if (u1 & 0x80) 
		printf("Fix mode: manual, ");
	else
		printf("Fix mode: auto, ");

	printf("SVs %d)\n",b1);
	

	f1 = getbef32(buf,2); //PDOP
	printf("PDOP: %.2f, ",f1);
	f1 = getbef32(buf,6); //HDOP
	printf("HDOP: %.2f, ",f1);
	f1 = getbef32(buf,10); //VDOP
	printf("VDOP: %.2f, ",f1);
	f1 = getbef32(buf,14); //TDOP
	printf("TDOP: %.2f\n",f1);
	idx = 18;
	if (b1 > 0) {
		printf("SV PRNs: ");
		while (b1 > 0 && (idx < length)){
			printf("%d ", getsb(buf,idx) );
			idx += 1; 
			b1--;
			}
		printf("\n");
		}

	break;

case 0x82:
	printf(" Differential Position Fix Mode\n");
	u1 = getub(buf,1); // mode
	printf("Mode %d, ",u1);
	switch (u1) {
		case 0: printf("Differential off (Manual GPS)\n"); break;
		case 1: printf("Differential on (Manual DGPS)\n"); break;
		case 2: printf("Differential currently off (Auto DGPS)\n"); break;
		case 3: printf("Differential currently on (Auto DGPS)\n"); break;
		default: printf("Unknown mode\n");break;
		}

	break;

case 0x84:
	printf(" Double-precision LLA Position Fix and Bias Information\n");
	//0-7 latitude Double radians; + for north, - for south
	//8-15 longitude Double radians; + for east, - for west
	//16-23 altitude Double meters
	//24-31 clock bias Double meters
	//32-35 time-of-fix Single seconds
	latitude = getbed64((char *)buf, 1) * 180.0 / M_PI;
	longitude = getbed64((char *)buf, 9) * 180.0 / M_PI;
	altitude = getbed64((char *)buf, 17);
	d1 = getbed64((char *)buf, 25);
	printf("Latitude: %.6f \n",latitude);
	printf("Longitude: %.6f \n",longitude);
	printf("Altitude: %.2f m\n",altitude);
	printf("Clock Bias: %.2f m\n",d1);
	f1 = getbef32(buf,33); //time of fix
	printf("Time of fix: %.2f s\n",f1);

	break;

case 0x8f:			/* Super Packet.  Well...  */
	u1 = getub(buf, 1);
	printf("-%02X:",u1);
	switch (u1) {		/* sub-packet ID */
		case 0xab:	//0x8f 0xab primary timing packet
			printf(" Primary Timing packet\n");
			ul1 = getbeu32(buf, 2);	/* Time of week */
			s1 = getbeu16(buf, 6);	/* week number */
			s2 = getbes16(buf, 8);	/* UTC offset */
			printf("Time of week: %d s\n",ul1);
			printf("Week number: %d\n",s1);
			printf("UTC Offset: %d s\n",s2);
			u1 = getub(buf,10);//time flag
			printf("Time Flags: 0x%02X\n",u1);
			u2 = getub(buf,11);//seconds
			u3 = getub(buf,12);//minutes
			u4 = getub(buf,13);//hours
			u5 = getub(buf,14);//day of month
			u6 = getub(buf,15);//month
			us1 = getbeu16(buf,16);//four digt year
			printf("Time: %04d-%02d-%02dT%02d:%02d:%02d\n",us1,u6,u5,u4,u3,u2,u1);

			break;
		case 0xac:	//supplemental timing packet
			printf(" Supplemental Timing Packet\n");
			u1 = getub(buf,2); // receiver mode
			printf("Receiver Mode: %d, ",u1);
			switch (u1) {
				case 0: printf("Automatic (2D/3D)\n"); break;
				case 1: printf("Single Satellite (Time)\n");break;
				case 3: printf("Horizontal (2D)\n");break;
				case 4: printf("Full Position (3D)\n");break;
				case 7: printf("Over-determined Clock\n");break;
				default: printf("Unknown mode\n");break;
				}
			u1 = getub(buf,3);//disciplining mode
			if (u1>0)
				printf("Disciplining Mode: 0x%02X\n",u1);
			u1 = getub(buf,4);//self survey
			printf("Self-Survey Progress: %d %\n",u1);
			ul1 = getbeu32(buf,5); //holdover duration
			if (ul1>0)
				printf("Holdover Duration: %d s\n", ul1);
			us1 = getbeu16(buf,9);//critical alarms
			if (us1>0)
				printf("Critical Alarms: 0x%04X\n",us1);
			us1 = getbeu16(buf,11);//minor alarms
			printf("Minor Alarms: 0x%04X",us1);
			//decode minor alarms
			if (us1 & 0x0002)
				printf(", Antenna Open");
			if (us1 & 0x0004)
                                printf(", Antenna Shorted");
			if (us1 & 0x0008)
                                printf(", Not tracking satellites");
			if (us1 & 0x0020)
                                printf(", Survey-in progress");
			if (us1 & 0x0040)
                                printf(", No stored position");
			if (us1 & 0x0080)
                                printf(", Leap second pending");
			if (us1 & 0x0100)
                                printf(", In test mode");
			if (us1 & 0x0200)
                                printf(", Position is questionable");
			if (us1 & 0x0800)
                                printf(", Almanac not complete");
			if (us1 & 0x1000)
                                printf(", PPS not generated");

			printf("\n");
			u1 = getub(buf,13); 
			printf("GNSS Decoding Status: 0x%02X, ",u1);
			switch(u1) {
				case 0: printf("Doing fixes\n"); break;
				case 1: printf("Don't have GNSS time\n"); break;
				}
			u1 = getub(buf,14); //disciplining activity
			u1 = getub(buf,15);// PPS indication
			printf("PPS indication: %d, ",u1);
			switch (u1) {
				case 0: printf("PPS Good\n"); break;
				case 1: printf("PPS NOT Good\n");break;
				default: printf("Unknown\n");break;
				}
			temperature = getbef32(buf,33);
			printf("Temperature: %.2f deg C\n",temperature);
			latitude = getbed64((char *)buf, 37) * 180.0 / M_PI;
			longitude = getbed64((char *)buf, 45) * 180.0 / M_PI;
			altitude = getbed64((char *)buf, 53);
			printf("Latitude: %.6f \n",latitude);
			printf("Longitude: %.6f \n",longitude);
			printf("Altitude: %.2f m\n",altitude);
			break;
		default:
			printf(" undecoded packet\n");
			break;
			

	}
	break;
default:
	printf(" undecoded\n");
	break;
}

printf("\n");

return 0;

}

int main(int argc, char* argv[]) {

speed_t baudRate = DEFAULT_BAUD_RATE;
const char *serialPort = DEFAULT_SERIAL_PORT;
int oddParity = 1; //default to odd parity as most common
int enableTimingPackets = 0;

//parse command line options
int opt;
char *pch;
while((opt = getopt(argc, argv, "b:p:ntv")) != EOF)
	switch (opt) {
		case 'b':
			baudRate = atoi(optarg);
			switch(baudRate){
				case 9600: baudRate = B9600; break;
				case 38400: baudRate = B38400; break;
				case 57600: baudRate = B57600; break;
				case 115200: baudRate = B115200; break;
				default: 
					printf("Unknown baud rate %d.\n", baudRate);
					exit(1);
					break;
				}
			break;
		case 'p':
			serialPort = optarg;
			break;
		case 'n':
			oddParity = 0;
			break;
		case 't':
			enableTimingPackets = 1;
			break;

		case 'v':
			gVerbose = 1;
			break;

	}


printf("Opening port %s ...\n",serialPort);

  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open(serialPort, O_RDWR);

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

if (oddParity) {
	tty.c_cflag |= PARENB; // Odd parity bit, most common for TSIP
	tty.c_cflag |= PARODD;
	}
else { //no parity - some TSIP devices
	tty.c_cflag &= ~PARENB;
	}

  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 4;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, baudRate);
  cfsetospeed(&tty, baudRate);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Allocate memory for read buffer, set size according to your needs
  unsigned char read_buf [BUFFER];
  unsigned char tsip_buf [BUFFER];
  int bufferIdx = 0;
  int tsipIdx=0;
  unsigned char last=0; 
  int notFirstMessage = 0;

//
int msgLen = 0;
unsigned char hwBytes[9];
hwBytes[0]=0x10;
hwBytes[1]=0x8E;
hwBytes[2]=0xA5; //Command Packet 0x8E-A5: Packet Broadcast Mask
hwBytes[3]=0x00;
hwBytes[4]=0x45; // 0100 0101  Enable primary and secondary timing packets. Enable automatic output packets.
hwBytes[5]=0x00; //reserved
hwBytes[6]=0x00; //reserved
hwBytes[7]=0x10;
hwBytes[8]=0x03;

if (enableTimingPackets){
	int bw;
	msgLen = 9;  //will send message again  - see below. Not sure why? 
	printf("Enabling timing packet output... ");
	bw=write(serial_port,hwBytes,msgLen);
	printf("%d bytes sent.\n", bw);
}



  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME

  while(1) {
	if (bufferIdx >= BUFFER)
		bufferIdx = 0;

  	int num_bytes = read(serial_port, read_buf+bufferIdx, BUFFER-bufferIdx);

  	// n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
 	if (num_bytes < 0) {
      		printf("Error reading: %s", strerror(errno));
		return 1;
		}

	int idx=0;
	while (idx<num_bytes)	{
		unsigned char x=read_buf[bufferIdx++];
		idx++;
		//printf("%02x ",x);
		if (last != 0x10 || tsipIdx == 1)
			tsip_buf[tsipIdx++]=x; 
		else { //last byte was a <DLE> 
  		
			if (x == 0x03){ //end of TSIP message
				tsip_buf[tsipIdx++]=x;
				if (notFirstMessage)
					parse(tsip_buf, tsipIdx);
				else {
					notFirstMessage = 1;
					if (msgLen>0){ 
						int bw;
						printf("Enabling timing packet output... ");
						bw=write(serial_port,hwBytes,msgLen);
						printf("%d bytes sent.\n", bw);
						}

				}
				printf("Looking for next message...\n" );
				fflush(stdout);
				tsipIdx=0; //start collecting next TSIP message
				}
			if (x == 0x10){// last was 0x10 and this byte is 0x10 => stuff byte
				x=0; //becomes last so that if next byte is 0x10 then it will get saved
			}
		}
		last = x;
  	}
	//bufferIdx += num_bytes;

  }

  close(serial_port);
  return 0; // success
}
