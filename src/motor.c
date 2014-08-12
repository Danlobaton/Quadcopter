#include "quadcopter.h"
#include "simpletools.h"
#include "servo.h"

#define SWAP(x,y) \
    if (newMotors[y]->current_val < newMotors[x]->current_val) \
    {                                                          \
      volatile Motor* tmp = newMotors[x];                      \
      newMotors[x] = newMotors[y];                             \
      newMotors[y] = tmp;                                      \
    }
#define PWM_CYCLE 20000

void quad_power(int, int);
void quad_power(int);

void pwm_run();

volatile Motor fr = { .pin = PIN_MOTOR_FR, .current_val = 0 };
volatile Motor fl = { .pin = PIN_MOTOR_FL, .current_val = 0 };
volatile Motor br = { .pin = PIN_MOTOR_BR, .current_val = 0 };
volatile Motor bl = { .pin = PIN_MOTOR_BL, .current_val = 0 };

volatile Motor* motors[4];

void motor_init()
{
  motors[0] = &fr;
  motors[1] = &fl;
  motors[2] = &br;
  motors[3] = &bl;
}

void motor_run()
{
  cog_run(&pwm_run, 100);
  quad_power(1000, 10000);
  quad_power(1100, 2000);
  quad_power(1200, 3000);
  quad_power(1100, 2000);
  quad_power(1000, 1000);
}

void pwm_run()
{
  servo_start();
  quad_wakeup();
  while(1)
  {
    for (int i=0;i<4;i++)
      servo_set(motors[i]->pin, motors[i]->current_val);
  }
}

void quad_power(int power, int time) {
  int tmp = CNT;
  quad_power(power);
  waitcnt(tmp + CLKFREQ*time/1000)
}

void quad_power(int power)
{
  for (int i=0;i<4;i++)
    motors[i]->current_val = power;
}