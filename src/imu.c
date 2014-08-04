#include "quadcopter.h"
#include "simpletools.h"

i2c imuConn;

volatile IMU imu;

volatile int lock;
unsigned long last;

void imu_run()
{
  while(1)
  {
    last = CNT;
    waitcnt(CNT + CLKFREQ/1000*IMU_UPDATE_DELAY);
    imu_update();
  }
}

void imu_init()
{
  lock = 0;
  i2c_open(&imuConn, PIN_IMU_SCL, PIN_IMU_SDA, 0);

  // Gyro initialization.
  // 22 -> 11011  Set internal clock and scale.
  // 21 -> 9      Set sample rate.
  // 23 -> 101    Trigger interrupt when new data is ready (extra int pins).
  // 62 -> 1      Set clock source.
  write_to_register(&imuConn, GYRO_ADDR, 0x16, 0x1A);
  write_to_register(&imuConn, GYRO_ADDR, 0x15, 0x09);
  write_to_register(&imuConn, GYRO_ADDR, 0x17, 0x05);
  write_to_register(&imuConn, GYRO_ADDR, 0x3E, 1);
  
  // Accel initialization.
  // 45 -> 1000, 100  Wake it from sleep, keep it awake.
  // 49 -> 01    Set the data range. 00->2, 01->4, 10->8, 11->16 (+- g).
  write_to_register(&imuConn, ACCL_ADDR, 0x2D, 16);
  write_to_register(&imuConn, ACCL_ADDR, 0x2D, 8);
  write_to_register(&imuConn, ACCL_ADDR, 0x31, 0); // Might need to change this later.
  //writeToRegister(&imuConn, ACCL_ADDR, 0x1E, -25/-4); -25 // Tuning values.
  //writeToRegister(&imuConn, ACCL_ADDR, 0x1F, 12/-4);   12
  //writeToRegister(&imuConn, ACCL_ADDR, 0x20, 249/4);  249
}

void imu_update()
{
  while(lock==1);
  lock = 1;

  imu.g.x.raw = (signed short) read_value(&imuConn, GYRO_ADDR, GYRO_REG_X, 0);
  imu.g.y.raw = (signed short) read_value(&imuConn, GYRO_ADDR, GYRO_REG_Y, 0);
  imu.g.z.raw = (signed short) read_value(&imuConn, GYRO_ADDR, GYRO_REG_Z, 0);

  imu.a.x.raw = (signed short) read_value(&imuConn, ACCL_ADDR, ACCL_REG_X, 1);
  imu.a.y.raw = (signed short) read_value(&imuConn, ACCL_ADDR, ACCL_REG_Y, 1);
  imu.a.z.raw = (signed short) read_value(&imuConn, ACCL_ADDR, ACCL_REG_Z, 1);

  imu.a.x.filter = imu.a.x.raw*GYRO_FILTER_ALPHA + imu.a.x.filter*(1-GYRO_FILTER_ALPHA);
  imu.a.y.filter = imu.a.y.raw*GYRO_FILTER_ALPHA + imu.a.y.filter*(1-GYRO_FILTER_ALPHA);
  imu.a.z.filter = imu.a.z.raw*GYRO_FILTER_ALPHA + imu.a.z.filter*(1-GYRO_FILTER_ALPHA);
  
  imu.a.roll = atan2(-imu.a.y.filter, imu.a.z.filter);
  imu.a.pitch = atan2(imu.a.x.filter, sqrt(imu.a.y.filter*imu.a.y.filter + imu.a.z.filter*imu.a.z.filter));

  imu.pitch = 0.98*(imu.pitch + imu.g.z.raw*IMU_UPDATE_DELAY) + 0.02*imu.a.pitch;
  imu.roll = 0.98*(imu.roll + imu.g.x.raw*IMU_UPDATE_DELAY) + 0.02*imu.a.roll;

  lock = 0;
}

double errSum = 0;
double lastErr = 0;
double lastInput;
double kp = 1;
double ki = 1;
double kd = 1;
int compute_pid(int input, int setpoint) // Input, Setpoint
{
  double time = 10;
  
  double error = setpoint - input;
  errSum += (error * time);
  double dIn = input - lastInput;
  
  int output = kp*error + ki*errSum - kd*dIn;
  
  lastInput = input;
  lastErr = error;

  return output;
}