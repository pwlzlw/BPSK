/*
* Binary data Cubesat Space protocol mimic
*/
#include "stm32f4xx_sdio.h"
const int binarydata[8] = {	
	/*HEADER*/
	1,0,1,1,0,0,1,1
	/*
	//priority
	1,0,
	//source
	1,0,1,0,1,
	//destination
	0,1,0,1,0,
	//destination port
	1,0,1,0,1,0,
	//source port
	1,0,1,0,1,0, 
	//reserved
	1,0,1,0,
	//HMAC,XTEA,RDP,CRC
	1,0,1,0,
	
	//  DATA 
	1,0,1,0,1,0,1,0, 
	1,0,1,0,1,0,1,0, 	
	1,0,1,0,1,0,1,0, 
	1,0,1,0,1,0,1,0, 	
	*/
};
const int framesize = sizeof(binarydata)/sizeof(binarydata[0]);

