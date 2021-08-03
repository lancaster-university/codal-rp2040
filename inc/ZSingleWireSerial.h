// keep the name ZSingleWireSerial since used by pxt-jacdac hw.cpp
#ifndef ZSINGLE_WIRE_SERIAL_H
#define ZSINGLE_WIRE_SERIAL_H

#include "Pin.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "DMASingleWireSerial.h"

namespace codal
{

class ZSingleWireSerial : public DMASingleWireSerial 
{
protected:
  uint32_t baudrate;
  uint8_t txprog, rxprog;
  uint8_t smtx = 0;
  uint8_t smrx = 1;
  int dmachTx = -1;
  int dmachRx = -1;

  virtual void configureRxInterrupt(int enable);
  virtual int configureTx(int);
  virtual int configureRx(int);
public:
  ZSingleWireSerial(Pin& p);

  virtual int putc(char c);
  virtual int getc();

  virtual int send(uint8_t* data, int len);
  virtual int receive(uint8_t* data, int len);

  virtual int setBaud(uint32_t baud);
  virtual uint32_t getBaud();

  virtual int sendDMA(uint8_t* data, int len);
  virtual int receiveDMA(uint8_t* data, int len);
  virtual int abortDMA();

  int setMode(SingleWireMode sw) override;

  virtual int sendBreak();

  int getBytesReceived() override;
  int getBytesTransmitted() override;

};


}


#endif
