#include "quadcopter.h"
#include "simpletools.h"

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
  cog_run(&pwm_run, 3);
  while(1)
  {
    int signal = lastCommand;
    if (signal == COMMAND_WAKEUP)
      quad_wakeup();
    else if (signal == COMMAND_TAKEOFF)
      quad_takeoff();
    else if (signal == COMMAND_LAND)
      quad_land();
    else if (signal == COMMAND_SHUTDOWN)
      quad_shutdown();
    else if (signal == COMMAND_HOVER)
      quad_hover();
  }
}

void quad_hover()
{
  
}

void quad_land()
{
  for (int i=0;i<4;i++)
    motors[i]->current_val = MOTOR_HOVER - 5*clamp(0, 100-range, 100);
}

void quad_takeoff()
{
  for (int i=0;i<4;i++)
    motors[i]->current_val = MOTOR_HOVER + 5*clamp(0, 100-range, 100);
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

void pwm_run()
{
  while(1)
  {
    int startTmp = CNT;
    memcpy(&newMotors, &motors, sizeof(motors));

    // Sort.
    SWAP(0, 1);
    SWAP(2, 3);
    SWAP(0, 2);
    SWAP(1, 3);
    SWAP(1, 2);

    int tmp = CNT;

    for (int i=0;i<4;i++)
      high(newMotors[i]->pin);

    for (int i=0;i<4;i++)
    {
      waitcnt(tmp + CLKFREQ/1000000*(newMotors[i]->current_val));
      low(newMotors[i]->pin);
    }

    waitcnt(startTmp + CLKFREQ/50);
  }
}

int clamp(int l, int n, int h)
{
  return n<l?l:(n>h?h:n);
}