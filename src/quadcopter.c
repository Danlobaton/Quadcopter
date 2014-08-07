#include "quadcopter.h"
#include "simpletools.h"

int main()
{
  simpleterm_close();
  
  xbee_init();
  imu_init();
  motor_init();

  cog_run(&ultrasonic_run, 0);
  cog_run(&motor_run, 1);
  cog_run(&imu_run, 2);
}