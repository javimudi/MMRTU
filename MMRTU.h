#ifndef MMRTU_H
#define MMRTU_H
#endif

#include <AltSoftSerial.h>

// ModbusMasterRTU Library

#define BUFFER_SIZE 128
class MMRTU { 
  private:
    unsigned char _frame[BUFFER_SIZE];
    unsigned int _id;
    unsigned char _function;
    unsigned int _address;
    unsigned int _no_of_registers;
    int _TxEnablePin;
    
  public:
    MMRTU(unsigned int id, unsigned char function, unsigned int address, unsigned int no_of_registers, int TxEnablePin);
    void setup(long baud, uint8_t config, uint8_t rxPin, uint8_t txPin);    
    void setup(long baud, uint8_t config);    
    void constructPacket(void);
    void calculateCRC(void);
    void sendPacket(void);
    void sendPacket(AltSoftSerial);
    unsigned char * getData(AltSoftSerial);
    unsigned char * getData(void);    
    unsigned char * getFrame(void);
};
