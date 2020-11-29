#include <PacketSerial.h>
#include <MsTimer2.h>
#include <AS5601.h>
#include <Servo.h>
#include <PID.h>
#include "RegisterMap.h"
#include "FlappingServo.h"

#define CONSOLE_CONTROL 1

/*TO DO***********************************************************
e_speed, e_degゲインの調整
******************************************************************/

int32_t Register[64] = {};
PacketSerial PC(Register);
uint8_t servoPin1 = 11;
uint8_t servoPin2 = 10;
AS5601_I2C encoder_i2c1;
AS5601_I2C encoder_i2c2;
FlappingServo my_servo1(servoPin1, &encoder_i2c1, 0);
FlappingServo my_servo2(servoPin2, &encoder_i2c2, 1);


uint8_t control_T = 1; // ms
//uint8_t write_T = 25; // ms //PCが追い付かないと思ったらグラフの描画が重いっぽい
uint8_t write_T = 40;

int data_set[5] = {};

void setup()
{
  pinMode(13, OUTPUT);
  my_servo1.init();
  my_servo2.init();
  Serial.begin(1000000);
  Serial.println("Power on");
  my_servo2.setServoDirectionReverse();
  my_servo1.stopMotor();
  my_servo2.stopMotor();
  my_servo1.A = 6.5;
  MsTimer2::set(control_T, tick);
  MsTimer2::start();

  delay(4000);
  Serial.println("Control Start");
}

float deg_abs = 0;
float deg_inc = 0;
float speed_abs = 0;
float speed_inc = 0;

float deg1 = 0;
float deg2 = 0;
float deg_per_sec1 = 0;
float deg_per_sec2 = 0;

bool servo_enable = true;

// loop_rate: 3-6 ms
void loop()
{
#ifdef CONSOLE_CONTROL
  static int target = 0;
  if( Serial.available() > 0 )
  {
    String s = Serial.readStringUntil('\n');
    if( sscanf(s.c_str(), "%d", &target) == 1 )
    {
      Serial.println(s);
      my_servo1.setTarget(target, CommandMode::SPEED);
      my_servo2.setTarget(target, CommandMode::SPEED);
    }
  }
  if(target > 0)
  {
    float cal_speed1 = my_servo1.getSpeed();
    float abs_degree1 = my_servo1.getAbsDegree();
    float cal_degree1 = my_servo1.getDegree();
    float cal_speed2 = my_servo2.getSpeed();
    float abs_degree2 = my_servo2.getAbsDegree();
    float cal_degree2 = my_servo2.getDegree();
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(cal_speed1);
    Serial.print(", ");
    Serial.print(cal_degree1);
    Serial.print(", ");
    Serial.print(cal_speed2);
    Serial.print(", ");
    Serial.println(cal_degree2);
  }
  
#else
  if( Register[COMMAND_MOTOR] == Register[COMMAND_MOTOR_CONFIRM] )
  {
    my_servo1.setTarget(Register[COMMAND_MOTOR], (CommandMode)Register[COMMAND_MODE]);
    my_servo2.setTarget(Register[COMMAND_MOTOR], (CommandMode)Register[COMMAND_MODE]);
  }
#endif
  
  //delay(10);
  //digitalWrite(13, Register[COMMAND_LED]);
  //setPIDparameter();
  //delay(1);

  my_servo1.updateAbsoluteEncoder();
  deg1 = my_servo1.getDegree();
  deg_per_sec1 = my_servo1.getSpeed(); 
  //delay(1);
  my_servo2.updateAbsoluteEncoder();
  deg2 = my_servo2.getDegree();
  deg_per_sec2 = my_servo2.getSpeed();

  static int last_time = 0;
  int current_time = micros();
  int dt = current_time - last_time;
  last_time = current_time;

  data_set[0] = millis();
  data_set[1] = deg1;
  data_set[2] = deg2;
  data_set[3] = deg_per_sec1;
  data_set[4] = deg_per_sec2;
 
  //Serial.print(deg1);
  //Serial.print(", ");
  //Serial.println(my_servo1.getAbsSpeed());
  

  //delay(1);
  
  if(Register[COMMAND_START] > 0)
  {
    digitalWrite(13, HIGH);
    PC.writeDataWithSize(123, START_REPLY);
    Register[COMMAND_START] = 0;
    servo_enable = true;
    my_servo1.pidReset();
    my_servo2.pidReset();
  }
  if(Register[COMMAND_STOP] > 0)
  {
    digitalWrite(13, LOW);
    Register[COMMAND_STOP] = 0;
    servo_enable = false;
    my_servo1.stopMotor();
    my_servo2.stopMotor();
  }
}

int t_ = 0;
int t_2 = 0;
void tick()
{
  t_++;
  t_2++;
  if(servo_enable)
  {
    servoControl();
  }
#ifndef CONSOLE_CONTROL
  if(t_ > write_T/control_T)
  {
    t_ = 0;
    writeData();
  }
#endif
}

void servoControl()
{
  float deg1 = my_servo1.getDegree();
  float deg2 = my_servo2.getDegree();
  my_servo1.setAnotherMotorDegree(deg2);
  my_servo2.setAnotherMotorDegree(deg1);

  my_servo1.controlMotor();
  my_servo2.controlMotor();
}

void setPIDparameter()
{
  float p = float(Register[PARAMETER_P]) / 1000.0;
  float i = float(Register[PARAMETER_I]) / 1000.0;
  float d = float(Register[PARAMETER_D]) / 1000.0;
  my_servo1.setPositionPIDParameter(p, i, d);
}

void writeData()
{
  static int func_switch = 0;
  if(func_switch == 0)
  {
    PC.writeDataWithSize(data_set[0], CURRENT_TIME);
    func_switch = 1;
  }
  else if(func_switch == 1)
  {
    PC.writeDataWithSize(data_set[1], MOTOR_DEGREE_1);
    func_switch = 2;
  }
  else if(func_switch == 2)
  {
    //PC.writeDataWithSize((int)my_servo1.currentTarget(), CURRENT_COMMAND);
    PC.writeDataWithSize(data_set[2], MOTOR_DEGREE_2);
    func_switch = 3;
  }
  else if(func_switch == 3)
  {
    PC.writeDataWithSize(data_set[3], MOTOR_SPEED_1);
    func_switch = 4;
  }
  else if(func_switch == 4)
  {
    PC.writeDataWithSize(data_set[4], MOTOR_SPEED_2);
    func_switch = 0;
  }
  else
  {
    func_switch = 0;
  }
}

#ifndef CONSOLE_CONTROL
void serialEvent()
{
  if (Serial.available() > 0)
  {
    PC.readData();
  }
}
#endif
