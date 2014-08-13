#include "simpletools.h"

#define PIN_IMU_SDA    16
#define PIN_IMU_SCL    15
#define PIN_MOTOR_F   8
#define PIN_MOTOR_B   9
#define PIN_MOTOR_L   10
#define PIN_MOTOR_R   11
#define PIN_XBEE_IN    15 // DOUT
#define PIN_XBEE_OUT   14 // DIN
#define PIN_RANGE_TRIG 7
#define PIN_RANGE_ECHO 8

#define GYRO_ADDR  0xD0
#define GYRO_REG_X 0x1D
#define GYRO_REG_Y 0x1F
#define GYRO_REG_Z 0x21

#define ACCL_ADDR  0xA6
#define ACCL_REG_X 0x32
#define ACCL_REG_Y 0x34
#define ACCL_REG_Z 0x36

#define IMU_UPDATE_DELAY 100 // Time between updates (in ms)
#define ACCEL_FILTER_ALPHA 0.5 // Alpha value of low pass filter

#define XBEE_BAUD_RATE 9600

#define MOTOR_START 1000
#define MOTOR_LOW   1100
#define MOTOR_HOVER 1300
#define MOTOR_HIGH  2000

#define COMMAND_WAKEUP   0x10 // 1
#define COMMAND_FORWARD  0x20 // P
#define COMMAND_BACKWARD 0x30 // P
#define COMMAND_HOVER    0x40 // 1
#define COMMAND_DOWN     0x50 // P
#define COMMAND_UP       0x60 // P
#define COMMAND_CCW      0x70 // P
#define COMMAND_CW       0x80 // P
#define COMMAND_RIGHT    0x90 // P
#define COMMAND_LEFT     0xA0 // P
#define COMMAND_SHUTDOWN 0xB0 // 1
#define COMMAND_REQUEST  0xC0 // 1
#define COMMAND_TAKEOFF  0xD0 // A
#define COMMAND_LAND     0xE0 // A

#define PARAM_ONCE  0x00
#define PARAM_POWER 0x01
#define PARAM_AUTON 0x02

volatile unsigned int lastCommand;
volatile unsigned int range;

typedef struct
{
  int pin;
  int current_val;
} Motor;

typedef struct
{
  signed short raw;
  double filter;
} AccelRaw;

typedef struct
{
  AccelRaw x;
  AccelRaw y;
  AccelRaw z;
  double pitch;
  double roll;
} Accel;

typedef struct
{
  signed short raw;
  signed short offset;
  double filter;
} GyroRaw;

typedef struct
{
  GyroRaw x;
  GyroRaw y;
  GyroRaw z;
} Gyro;

typedef struct
{
  double input;
  double lastInput;
  double errSum;
  double lastErr;
  double output;
  double setpoint;
  double kp;
  double ki;
  double kd;
  volatile Motor* inc;
  volatile Motor* dec;
} Axis;

typedef struct
{
  Gyro g;
  Accel a;
  Axis pitch;
  Axis roll;
} IMU;

volatile IMU imu;

unsigned short read_from_register(i2c*, int, int);
void write_to_register(i2c*, int, int, int);
unsigned short read_value(i2c*, int, int, int);
unsigned short combine(char, char);

void imu_init();
void imu_update();
void compute_pid(volatile Axis*, int);
void imu_run();

void xbee_init();
void xbee_run();
unsigned int xbee_get_byte();
void xbee_send_byte(unsigned int);

void motor_init();
void motor_run();

void ultrasonic_run();

int clamp(int, int, int);