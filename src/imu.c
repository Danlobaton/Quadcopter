#include "quadcopter.h"
#include "simpletools.h"

i2c imuConn;

volatile int lock;
volatile int first = 1;

void imu_run()
{
  simpleterm_open();
  waitcnt(CNT + CLKFREQ);

  while(1)
  {
    unsigned long last = CNT;
    imu_update();
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

  configure_pid(&imu.pitch, IMU_UPDATE_DELAY);
  configure_pid(&imu.roll, IMU_UPDATE_DELAY);
}

void imu_update()
{
  while(lock==1);
  lock = 1;

  imu.g.x.raw = (signed short) read_value(&imuConn, GYRO_ADDR, GYRO_REG_X, 0)/(2000*IMU_UPDATE_DELAY) - imu.g.x.offset;
  imu.g.y.raw = (signed short) read_value(&imuConn, GYRO_ADDR, GYRO_REG_Y, 0)/(2000*IMU_UPDATE_DELAY) - imu.g.x.offset;
  imu.g.z.raw = (signed short) read_value(&imuConn, GYRO_ADDR, GYRO_REG_Z, 0)/(2000*IMU_UPDATE_DELAY) - imu.g.x.offset;

  imu.a.x.raw = (signed short) read_value(&imuConn, ACCL_ADDR, ACCL_REG_X, 1)/4;
  imu.a.y.raw = (signed short) read_value(&imuConn, ACCL_ADDR, ACCL_REG_Y, 1)/4;
  imu.a.z.raw = (signed short) read_value(&imuConn, ACCL_ADDR, ACCL_REG_Z, 1)/4;

  if (first)
  {
    first = 0;
    imu.g.x.offset = imu.g.x.raw;
    imu.g.y.offset = imu.g.y.raw;
    imu.g.z.offset = imu.g.z.raw;

    imu.g.x.raw = 0;
    imu.g.y.raw = 0;
    imu.g.z.raw = 0;
  }
  else
  {
    // y[n]=(1-alpha)y[n-1]+(1-alpha)(x[n]-x[n-1])
    imu.g.x.filter = 0.98*imu.g.x.filter + 0.02*(imu.g.x.raw);
    imu.g.y.filter = 0.98*imu.g.y.filter + 0.02*(imu.g.y.raw);
    imu.g.z.filter = 0.98*imu.g.z.filter + 0.02*(imu.g.z.raw);
  }
  imu.a.x.filter = imu.a.x.raw*ACCEL_FILTER_ALPHA + imu.a.x.filter*(1-ACCEL_FILTER_ALPHA);
  imu.a.y.filter = imu.a.y.raw*ACCEL_FILTER_ALPHA + imu.a.y.filter*(1-ACCEL_FILTER_ALPHA);
  imu.a.z.filter = imu.a.z.raw*ACCEL_FILTER_ALPHA + imu.a.z.filter*(1-ACCEL_FILTER_ALPHA);

  imu.a.roll = atan2(imu.a.x.filter, sqrt(imu.a.y.filter*imu.a.y.filter + imu.a.z.filter*imu.a.z.filter))*180/PI;
  imu.a.pitch = atan2(-imu.a.y.filter, imu.a.z.filter)*180/PI;

  imu.roll.input = 0.90*(imu.roll.input + imu.g.x.raw*IMU_UPDATE_DELAY) + 0.1*imu.a.roll;
  imu.pitch.input = 0.90*(imu.pitch.input + imu.g.z.raw*IMU_UPDATE_DELAY) + 0.1*imu.a.pitch;

  compute_pid(&imu.pitch, MOTOR_LOW, MOTOR_HIGH);
  compute_pid(&imu.roll, MOTOR_LOW, MOTOR_HIGH);

  //printf("%.2f\t%.2f\t%.2f\n", imu.a.pitch, imu.a.z.filter, imu.a.z.raw);
  xbee_send_val(imu.pitch.input);
  xbee_send_val(imu.roll.input);
  xbee_send_byte('\n');

  lock = 0;
}

void configure_pid(volatile Axis* axis, int sample)
{
  axis->ki *= sample/1000;
  axis->kd /= sample/1000;
}

void compute_pid(volatile Axis* axis, int low, int high)
{
  double error = axis->setpoint - axis->input;

  axis->errSum += error;
  if (axis->errSum > high) axis->errSum = high;
  else if (axis->errSum < low) axis->errSum = low;

  double dIn = axis->input - axis->lastInput;

  axis->output = axis->kp*error + axis->ki*axis->errSum - axis->kd*dIn;
  if (axis->output > high) axis->output = high;
  else if (axis->output < low) axis->output = low;
  
  axis->lastInput = axis->input;
  axis->lastErr = error;
}