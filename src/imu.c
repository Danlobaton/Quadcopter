#include "quadcopter.h"
#include "simpletools.h"

i2c imuConn;

volatile IMU imu;

volatile int lock;

void imu_run()
{
  simpleterm_open();
  waitcnt(CNT + CLKFREQ);

  print("INFO: Starting IMU process.\n");
  while(1)
  {
    unsigned long last = CNT;
    imu_update();
    printf("%5d %5d\n", imu.a.pitch, imu.a.roll);
    waitcnt(last + CLKFREQ/1000*IMU_UPDATE_DELAY);
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

  imu.a.x.filter = imu.a.x.raw*ACCEL_FILTER_ALPHA + imu.a.x.filter*(1-ACCEL_FILTER_ALPHA);
  imu.a.y.filter = imu.a.y.raw*ACCEL_FILTER_ALPHA + imu.a.y.filter*(1-ACCEL_FILTER_ALPHA);
  imu.a.z.filter = imu.a.z.raw*ACCEL_FILTER_ALPHA + imu.a.z.filter*(1-ACCEL_FILTER_ALPHA);

  imu.a.roll = atan2(-imu.a.y.filter, imu.a.z.filter);
  imu.a.pitch = atan2(imu.a.x.filter, sqrt(imu.a.y.filter*imu.a.y.filter + imu.a.z.filter*imu.a.z.filter));

  imu.pitch.input = 0.98*(imu.pitch.input + imu.g.z.raw*IMU_UPDATE_DELAY) + 0.02*imu.a.pitch;
  imu.roll.input = 0.98*(imu.roll.input + imu.g.x.raw*IMU_UPDATE_DELAY) + 0.02*imu.a.roll;

  compute_pid(&imu.pitch, 0);
  compute_pid(&imu.roll, 0);

  lock = 0;
}

void compute_pid(volatile Axis* axis, int setpoint)
{
  double error = setpoint - axis->input;

  axis->errSum += error*IMU_UPDATE_DELAY;
  if (axis->errSum > MOTOR_HIGH) axis->errSum = MOTOR_HIGH;
  else if (axis->errSum < MOTOR_LOW) axis->errSum = MOTOR_LOW;

  double dIn = axis->input - axis->lastInput;

  axis->output = axis->kp*error + axis->ki*axis->errSum - axis->kd*dIn;
  if (axis->output > MOTOR_HIGH) axis->output = MOTOR_HIGH;
  else if (axis->output < MOTOR_LOW) axis->output = MOTOR_LOW;
  
  axis->lastInput = axis->input;
  axis->lastErr = error;
}