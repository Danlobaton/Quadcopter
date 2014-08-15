#include "quadcopter.h"
#include "simpletools.h"

void xbee_init()
{
  high(PIN_XBEE_OUT);
}

void xbee_run()
{
  while(1)
  {
    lastCommand = xbee_get_byte();
  }
}

unsigned int xbee_get_byte()
{
  int current = 1;

  while(current)
  {
    current = input(PIN_XBEE_IN);
  }

  waitcnt(CNT + CLKFREQ/(XBEE_BAUD_RATE*2));

  unsigned int currentByte = 0;
  unsigned long time = CNT;

  for (int i=0;i<8;i++)
  {
    waitcnt(time + CLKFREQ/XBEE_BAUD_RATE * (i+1));
    current = input(PIN_XBEE_IN);
    currentByte += current<<i;
  }
  
  return currentByte;
}

void xbee_send_val(double n)
{
  if (n<0)
  {
    xbee_send_byte('-');
    n = 0 - n;
  }
  xbee_send_byte(48+(int)(n/10)%10);
  xbee_send_byte(48+(int)(n)%10);
  xbee_send_byte('.');
  xbee_send_byte(48+(int)(n*10)%10);
  xbee_send_byte(48+(int)(n*100)%10);
  xbee_send_byte(' ');
}

void xbee_send_byte(unsigned int byte)
{
  waitcnt(CNT + CLKFREQ/XBEE_BAUD_RATE);
  low(PIN_XBEE_OUT);

  unsigned long time = CNT;

  for (int i=0;i<8;i++)
  {
    waitcnt(time + CLKFREQ/XBEE_BAUD_RATE * (i+1));
    int bit = byte>>i & 1;
    if (bit)
      high(PIN_XBEE_OUT);
    else
      low(PIN_XBEE_OUT);
  }

  waitcnt(CNT + CLKFREQ/XBEE_BAUD_RATE);
  high(PIN_XBEE_OUT);
}