#include "simpletools.h"

#define PIN_IMU_SDA    16
#define PIN_IMU_SCL    15
#define PIN_MOTOR_F    8
#define PIN_MOTOR_B    9
#define PIN_MOTOR_L    10
#define PIN_MOTOR_R    11
#define PIN_XBEE_IN    13 // DOUT
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

#define COMMAND_WAKEUP   0x61
#define COMMAND_TAKEOFF  0x62
#define COMMAND_SHUTDOWN 0x63

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
  Axis height;
} IMU;

volatile IMU imu;
volatile Motor* motors[4];

unsigned short read_from_register(i2c*, int, int);
void write_to_register(i2c*, int, int, int);
unsigned short read_value(i2c*, int, int, int);
unsigned short combine(char, char);

void imu_init();
void imu_update();
void imu_run();
void compute_pid(volatile Axis*);
void configure_pid(volatile Axis*, int);

void xbee_init();
void xbee_run();
unsigned int xbee_get_byte();
void xbee_send_byte(unsigned int);

void motor_init();
void motor_run();

void ultrasonic_run();

int clamp(int, int, int);