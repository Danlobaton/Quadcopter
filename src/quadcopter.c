#include "quadcopter.h"
#include "simpletools.h"

int main()
{
  motor_init();
  //xbee_init();
  //imu_init();
  
  //imu_run();
  motor_run();
  //cog_run(&xbee_run, 10);
  //cog_run(&ultrasonic_run, 1);
}