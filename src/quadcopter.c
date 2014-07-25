#include "quadcopter.h"
#include "simpletools.h"
#include "simplei2c.h"
#include <propeller.h>
#include <stdio.h>

int main()
{
  waitcnt(CNT + CLKFREQ); // Wait one second.

  imuInit();
  cog_run(&imuRun, 1);
  //cog_run(&xbeeRun, 2);
  xbeeRun();
  signed short gx,gy,gz,ax,ay,az;

  while(1)
  {
    waitcnt(CNT + CLKFREQ/10);
    getImuData(&gx, &gy, &gz, &ax, &ay, &az);
    printf("G: %5d %5d %5d\t", gx, gy, gz);
    printf("A: %5d %5d %5d\n", ax, ay, az);
  }
}