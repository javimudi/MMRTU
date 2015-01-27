#ifndef MMRTU_H
#define MMRTU_H
#endif

#include <MySoftwareSerial.h>

// ModbusMasterRTU Library

#define BUFFER_SIZE 128
class MMRTU { 
  private:
    unsigned char _frame[BUFFER_SIZE];
    unsigned int _id;
    unsigned char _function;
    unsigned int _address;
    unsigned int _no_of_registers;
    
  public:
    MMRTU(unsigned int id, unsigned char function, unsigned int address, unsigned int no_of_registers);
    void constructPacket(void);
    void calculateCRC(void);
    void sendPacket(MySoftwareSerial);
    unsigned char * getData(MySoftwareSerial);
    unsigned char * getFrame(void);
};
