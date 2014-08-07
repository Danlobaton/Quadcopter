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

void quad_wakeup();
void quad_takeoff();
void quad_land();
void quad_shutdown();
void quad_hover();

void pwm_run();
void pwm_rune();

volatile Motor fr = { .pin = PIN_MOTOR_FR, .current_val = 0 };
volatile Motor fl = { .pin = PIN_MOTOR_FL, .current_val = 0 };
volatile Motor br = { .pin = PIN_MOTOR_BR, .current_val = 0 };
volatile Motor bl = { .pin = PIN_MOTOR_BL, .current_val = 0 };

volatile Motor* motors[4];
volatile Motor* newMotors[4];

void motor_init()
{
  motors[0] = &fr;
  motors[1] = &fl;
  motors[2] = &br;
  motors[3] = &bl;
}

void motor_run()
{
  pwm_start(20000);
  pwm_set(PIN_MOTOR_BL, 0, 1000);
  waitcnt(CNT + CLKFREQ);
  while (1)
  {
    pwm_set(PIN_MOTOR_BL, 0, 1500);
  }
}

void quad_hover()
{
  
}

void quad_land()
{
  for (int i=0;i<4;i++)
    motors[i]->current_val = MOTOR_HOVER - 50;
}

void quad_takeoff()
{
  for (int i=0;i<4;i++)
    motors[i]->current_val = MOTOR_HOVER + 50;
}

void quad_wakeup()
{
  for (int i=0;i<4;i++)
    motors[i]->current_val = 1000;
}

void quad_shutdown()
{
  for (int i=0;i<4;i++)
    motors[i]->current_val = 1000;
}

void pwm_rune()
{
  servo_start();
  while(1)
  {
    for (int i=0;i<4;i++)
      servo_set(motors[i]->pin, motors[i]->current_val);
  }
}

void pwm_run()
{
  while(1)
  {
    unsigned long startTmp = CNT;

    for (int i=0;i<4;i++)
      high(motors[i]->pin);

    waitcnt(CNT + CLKFREQ/1000000*(motors[0]->current_val));

    for (int i=0;i<4;i++)
      low(motors[i]->pin);

    waitcnt(startTmp + CLKFREQ/50);
  }
}

int clamp(int l, int n, int h)
{
  return n<l?l:(n>h?h:n);
}