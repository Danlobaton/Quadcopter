#include "quadcopter.h"
#include "simpletools.h"

void ultrasonic_run()
{
  configure_pid(&imu.height, 60);
  int tmp;

  while(1)
  {
    tmp = CNT;

    high(PIN_RANGE_TRIG);
    waitcnt(CNT + CLKFREQ/100000); // 10 uS
    low(PIN_RANGE_TRIG);

    int duration = pulse_in(PIN_RANGE_ECHO, 1)/58;

    if (duration > 400)
      duration = 400;
    range = duration;

    compute_pid(&imu.height, 0, 400);

    waitcnt(tmp + CLKFREQ/17);
  }
}