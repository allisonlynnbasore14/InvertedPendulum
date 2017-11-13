
#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"

#define METERS_PER_CLICK 3.141592*80.0*(1/1000.0)/12.0/(162.5)
#define MOTOR_MAX 300
#define MAX_SPEED 0.75  // m/s
#define FORTY_FIVE_DEGREES_IN_RADIANS 0.78

extern int32_t angle_accum;
extern int32_t speedLeft;
extern int32_t driveLeft;
extern int32_t distanceRight;
extern int32_t speedRight;
extern int32_t distanceLeft;
extern int32_t distanceRight;
float int_left, int_right, omega_int;
float int_v_left, int_v_right;
float danceStart;
float Kp = 500;
float Ki = 5000;
float KIomega = -40;
float KPomega = -6;
float target_speed_R, target_speed_L;
float k = -.1;
float pos = 0;
float start_time;
float beginStable;
float w = 0;
float r = .05;

void balanceDoDriveTicks();

extern int32_t displacement;
int32_t prev_displacement = 0;

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;

uint32_t prev_time;

void setup()
{
  Serial.begin(9600);
  prev_time = 0;
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  angle_accum = 0;
  ledGreen(0);
  ledYellow(0);

}

extern int16_t angle_prev;
int16_t start_flag = 0;
int16_t armed_flag = 0;
int16_t start_counter = 0;
void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;

void newBalanceUpdate()
{
  static uint32_t lastMillis;
  uint32_t ms = millis();

  if ((uint32_t)(ms - lastMillis) < UPDATE_TIME_MS) {
    return;
  }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  // call functions to integrate encoders and gyros
  balanceUpdateSensors();

  if (imu.a.x < 0)
  {
    lyingDown();
    isBalancingStatus = false;
  }
  else
  {
    isBalancingStatus = true;
  }
}


float testSpeed = 0;          // this is the desired motor speed
bool started = false;
void loop()
{
  float cur_time = 0;
  static uint32_t prev_print_time = 0;   // this variable is to control how often we print on the serial monitor
  static float angle_rad;                // this is the angle in radians
  static float angle_rad_accum = 0;      // this is the accumulated angle in radians
  static float del_theta = 0;

  static float error_left_accum = 0;      // this is the accumulated velocity error in m/s
  static float error_right_accum = 0;      // this is the accumulated velocity error in m/s

  cur_time = millis();                   // get the current time in miliseconds



  newBalanceUpdate();                    // run the sensor updates. this function checks if it has been 10 ms since the previous

  if (angle > 3000 || angle < -3000)     // If angle is not within +- 3 degrees, reset counter that waits for start
  {
    start_counter = 0;
  }

  bool shouldPrint = cur_time - prev_print_time > 105;
  shouldPrint = false;
  if (shouldPrint)  // do the printing every 105 ms. Don't want to do it for an integer multiple of 10ms to not hog the processor
  {
    Serial.print(angle_rad);
    Serial.print("\t");
    Serial.print(angle);
    Serial.print("\t");
    Serial.print(speedLeft);
    Serial.print("\t");
    Serial.print(angle_rad_accum);
    Serial.println(testSpeed);
    prev_print_time = cur_time;
  }
  float delta_t = (cur_time - prev_time) / 1000.0;

  // handle the case where this is the first time through the loop
  if (prev_time == 0) {
    delta_t = 0.01;
  }

  if (start_counter == 1 && !started) {
    danceStart = millis();
    started = true;
  }
  int switchItUp = 0;
  float danceTime;

  //Set the timing for each move
  if (start_counter > 0) {
    danceTime = (millis() - danceStart) / 1000.;
    if (danceTime < 4) {
      switchItUp = 0;
    } else if (danceTime > 4 && danceTime < 10.) {
      switchItUp = 3;
    } else if (danceTime > 10. && danceTime < 18.) {
      switchItUp = 4;
    } else if (danceTime > 28. && danceTime < 33.) {
      switchItUp = 5;
    } else if (danceTime > 35. && danceTime < 42.) {
      switchItUp = 6;
    } else{
      switchItUp = 0;
    }
  }
  switch (switchItUp) {
    case 1:
      pos = pos + .02 * delta_t;
      w = 0;
      break;
    case 2:
      pos = pos - .02 * delta_t;
      w = 0;
      break;
      pos = -5.*sin(delta_t*3);
      w = 0;
      break;
    case 4:
      pos = -5.*sin(delta_t*3);
      w = 0;
      break;
    case 5:
      pos = 0;
      w = 2.5;
      break;
    case 6:
      pos = 0;
      w = -3;
      break;
    default:
      pos = pos;
      w = 0;
      break;
  }
  // every UPDATE_TIME_MS, check if angle is within +- 3 degrees and we haven't set the start flag yet
  if (cur_time - prev_time > UPDATE_TIME_MS && angle > -3000 && angle < 3000 && !armed_flag) {
    // increment the start counter
    start_counter++;
    // If the start counter is greater than 30, this means that the angle has been within +- 3 degrees for 0.3 seconds, then set the start_flag
    if (start_counter > 30)
    {
      armed_flag = 1;
      buzzer.playFrequency(DIV_BY_10 | 445, 1000, 15);
    }
  }



  // angle is in millidegrees, convert it to radians and subtract the desired theta
  angle_rad = ((float)angle) / 1000 / 180 * 3.14159 - (k * (((int_v_left + int_v_right) / 2.) - pos));

  // only start when the angle falls outside of the 3.0 degree band around 0.  This allows you to let go of the
  // robot before it starts balancing
  if (cur_time - prev_time > UPDATE_TIME_MS && (angle < -3000 || angle > 3000) && armed_flag)
  {
    start_flag = 1;
    armed_flag = 0;
  }
  // every UPDATE_TIME_MS, if the start_flag has been set, do the balancing
  if (cur_time - prev_time > UPDATE_TIME_MS && start_flag)
  {
    // set the previous time to the current time for the next run through the loop
    prev_time = cur_time;

    // speedLeft and speedRight are just the change in the encoder readings
    // we need to do some math to get them into m/s
    float vL = METERS_PER_CLICK * speedLeft / delta_t;
    float vR = METERS_PER_CLICK * speedRight / delta_t;


    // Set angle information
    omega_int = omega_int - angle_rad * delta_t;
    target_speed_L = (KPomega * (0 - angle_rad) + KIomega * omega_int) - w*r;
    target_speed_R = (KPomega * (0 - angle_rad) + KIomega * omega_int) + w*r;

    float PWM_left, PWM_right;
    bool flagRotate = true;

    int_v_left = int_v_left + delta_t*vL;
    int_v_right = int_v_right + delta_t*vR;
    int_left = int_left + delta_t * (target_speed_L - vL);
    int_right = int_right + delta_t* ((target_speed_R - vR));

    // set PWM_left and PWM_right here
    PWM_left = Kp * (target_speed_L - vL) + Ki * int_left;
    PWM_right = Kp * (target_speed_R - vR) + Ki * int_right;


    
    Serial.print(target_speed_L);
    Serial.print(" ");
    Serial.print(target_speed_R);
    Serial.print(" ");
    Serial.print(vR);
    Serial.print(" ");
    Serial.println(vL);
    // if the robot is more than 45 degrees, shut down the motor
    if (start_flag && fabs(angle_rad) > FORTY_FIVE_DEGREES_IN_RADIANS) // TODO: this was set to angle < -0.78... I changd it to angle_rad
    {
      // reset the accumulated errors here
      start_flag = 0;   // wait for restart
      prev_time = 0;
      motors.setSpeeds(0, 0);
    } else if (start_flag) {
      motors.setSpeeds((int)PWM_left, (int)PWM_right);
    }
  }

  // kill switch
  if (buttonA.getSingleDebouncedPress())
  {
    motors.setSpeeds(0, 0);
    while (!buttonA.getSingleDebouncedPress());
  }
}
