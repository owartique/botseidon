#include "SPI.hh"


SPI::SPI(unsigned int cs, unsigned int baud)
{
	this->cs = cs;
	this->baud = baud;
	wiringPiSPISetup(cs, baud);	
}

SPI::~SPI()
{
}


// ------------- SPI functions -------------

void SPI::tobytes(int len, int val, unsigned char *bytes)
{
    	for(int i=0; i<len; ++i)
        	bytes[i] = ( val >> 8*(len-(i+1)) ) % (256);
}

unsigned int SPI::frombytes(int len, unsigned char *bytes)
{
    	unsigned int res = 0;
	for(int i=0; i<len; ++i)
        	res = res + bytes[len-i-1]*(2 << (8*i-1));
    	return res;
}