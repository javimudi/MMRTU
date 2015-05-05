#include "Arduino.h"
#include <AltSoftSerial.h>
#include "MMRTU.h"


MMRTU::MMRTU(unsigned int id, unsigned char function, unsigned int address, unsigned int no_of_registers, int TxEnablePin)
{
	_id = id;
	_function = function;
	_address = address;
	_no_of_registers = no_of_registers;
	unsigned char _frame[BUFFER_SIZE];
	_TxEnablePin = TxEnablePin;
	// _frame = MMRTU::constructPacket();
	constructPacket();
}


unsigned char * MMRTU::getFrame(void){
	return _frame;
}



void MMRTU::setup(long baud, uint8_t config, uint8_t rxPin, uint8_t txPin, int soft) {

    if(soft==1){
        AltSoftSerial _mySerial = AltSoftSerial(rxPin,txPin);
        _mySerial.begin(baud, config);
        pinMode(_TxEnablePin, OUTPUT);
        digitalWrite(_TxEnablePin, LOW);    
    }
    else {
        Serial.begin(baud,config);
    }
}


void MMRTU::constructPacket(void){

	// unsigned int crc16;
	// unsigned char _frame[BUFFER_SIZE];

	_frame[0] = _id;
	_frame[1] = _function;
	_frame[2] = _address >> 8; // address Hi
	_frame[3] = _address & 0xFF; // address Lo
	_frame[4] = _no_of_registers >> 8; // no_of_registers Hi
	_frame[5] = _no_of_registers & 0xFF; // no_of_registers Lo
	
	calculateCRC();

	
}



void MMRTU::calculateCRC(void) 
{ 
  unsigned int bufferSize = 6; // 6 for READ_HOLDING
  unsigned int crc16;
  unsigned int temp, temp2, flag;
  
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ _frame[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;

  _frame[6] = temp >> 8;
  _frame[7] = temp & 0xFF;
  // return temp; // the returned value is already swapped - crcLo byte is first & crcHi byte is last
}

void MMRTU::sendPacket(void)
{
    unsigned int bufferSize=7;
    digitalWrite(_TxEnablePin, HIGH);

    Serial.write(_frame,bufferSize);            
    Serial.flush();
    
    // if (TxEnablePin > 1)
    digitalWrite(_TxEnablePin, LOW);
        
}


void MMRTU::sendPacket(AltSoftSerial mySerial)
{
    unsigned int bufferSize=7;
	digitalWrite(_TxEnablePin, HIGH);
		
	mySerial.write(_frame,bufferSize);	
	mySerial.flush();
	
	// allow a frame delay to indicate end of transmission
	delayMicroseconds(500); 
	
	// if (TxEnablePin > 1)
	digitalWrite(_TxEnablePin, LOW);
		
}


// get the serial data from the buffer
unsigned char * MMRTU::getData(AltSoftSerial mySerial)
{
  	unsigned char buffer = 0;
	unsigned char overflowFlag = 0;
	unsigned char response[BUFFER_SIZE];
		
  while (mySerial.available())
  {
		// The maximum number of bytes is limited to the serial buffer size of 128 bytes
		// If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the 
		// serial buffer will be red untill all the data is cleared from the receive buffer,
		// while the slave is still responding.
		if (overflowFlag) 
			mySerial.read();
		else
		{
			if (buffer == BUFFER_SIZE)
				overflowFlag = 1;
				
			response[buffer] = mySerial.read();
			buffer++;
		}
      
    delayMicroseconds(2400); // inter character time out
  }
	

	
  return response;
}


// get the serial data from the buffer
unsigned char * MMRTU::getData(void)
{
  	unsigned char buffer = 0;
	unsigned char overflowFlag = 0;
	unsigned char response[BUFFER_SIZE];
		
  while (Serial.available())
  {
		// The maximum number of bytes is limited to the serial buffer size of 128 bytes
		// If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the 
		// serial buffer will be red untill all the data is cleared from the receive buffer,
		// while the slave is still responding.
		if (overflowFlag) 
			Serial.read();
		else
		{
			if (buffer == BUFFER_SIZE)
				overflowFlag = 1;
				
			response[buffer] = Serial.read();
			buffer++;
		}
      
    delayMicroseconds(2400); // inter character time out
  }
	

	
  return response;
}





