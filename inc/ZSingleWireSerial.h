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
  uint8_t txprog, rxprog;
  uint8_t smtx = 0;
  uint8_t smrx = 1;
  int dmachTx = -1;
  int dmachRx = -1;
public:
  ZSingleWireSerial(Pin& p);
  virtual int sendDMA(uint8_t* data, int len);
  virtual int receiveDMA(uint8_t* data, int len);
  virtual int abortDMA();

  virtual int setMode(SingleWireMode sw);

};


}


#endif
