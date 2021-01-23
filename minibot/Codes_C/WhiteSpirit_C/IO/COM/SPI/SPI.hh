#ifndef _SPI_HH_
#define _SPI_HH_

	#include <wiringPiSPI.h>

class SPI
{
	public:
		SPI(unsigned int CS, unsigned int baud);
		~SPI();
		void tobytes(int len, int val, unsigned char *bytes);
		unsigned int frombytes(int len, unsigned char *bytes);
		int bytesToInt(unsigned char* b, unsigned length);

	protected:
		unsigned int baud;
		unsigned int cs;
};


#endif
