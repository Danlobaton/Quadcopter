#include "quadcopter.h"
#include "simpletools.h"

int main()
{
  waitcnt(CNT + CLKFREQ);
  
  motor_init();
  //xbee_init();
  //imu_init();

  cog_run(&motor_run, 100);
  //cog_run(&xbee_run, 10);
  //cog_run(&ultrasonic_run, 1);
  //imu_run();
}