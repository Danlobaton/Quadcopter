#include "quadcopter.h"
#include "simpletools.h"

int main()
{
  waitcnt(CNT + CLKFREQ);
  print("INFO: Starting\n");
  
  xbee_init();
  motor_init();
  //imu_init();

  cog_run(&xbee_run, 10);
  cog_run(&motor_run, 100);
  //cog_run(&ultrasonic_run, 1);
  //imu_run();
}