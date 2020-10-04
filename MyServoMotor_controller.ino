#include <PacketSerial.h>
#include <MsTimer2.h>
#include <AS5601.h>
#include <Servo.h>
#include <PID.h>
#include "RegisterMap.h"
#include "FlappingServo.h"

/*TO DO***********************************************************
sevo1の方が逆回転のときに動きが激しい

servo1を逆方向にしたが同期速度制御ができない
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
//uint8_t write_T = 40; // ms
//uint8_t write_T = 25; // ms
uint8_t write_T = 40;

int data_set[5] = {};

void setup()
{
  pinMode(13, OUTPUT);
  my_servo1.init();
  my_servo2.init();
  my_servo2.setServoDirectionReverse();
  Serial.begin(1000000);
  my_servo1.stopMotor();
  my_servo2.stopMotor();
  MsTimer2::set(control_T, tick);
  MsTimer2::start();
}

float deg_abs = 0;
float deg_inc = 0;
float speed_abs = 0;
float speed_inc = 0;

float deg1 = 0;
float deg2 = 0;
float deg_per_sec1 = 0;
float deg_per_sec2 = 0;

bool servo_enable = false;

void loop()
{
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

  my_servo1.setAnotherMotorDegree(deg2);
  my_servo2.setAnotherMotorDegree(deg1);

  data_set[0] = millis();
  data_set[1] = deg1;
  data_set[2] = deg2;
  data_set[3] = deg_per_sec1;
  data_set[4] = deg_per_sec2;

  
  
  /*static int prev_target_deg = 0;
  int target_deg = (int)my_servo1.currentTargetDegree();
  if(target_deg != prev_target_deg)
    PC.writeDataWithSize(target_deg, CURRENT_COMMAND);
  prev_target_deg = target_deg;*/
  //delay(1);
  //if(abs(Register[COMMAND_MOTOR]) < 1000)
  if( Register[COMMAND_MOTOR] == Register[COMMAND_MOTOR_CONFIRM] )
  {
    my_servo1.setTarget(Register[COMMAND_MOTOR], (CommandMode)Register[COMMAND_MODE]);
    my_servo2.setTarget(Register[COMMAND_MOTOR], (CommandMode)Register[COMMAND_MODE]);
  }
  
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
  if(t_ > write_T/control_T)
  {
    t_ = 0;
    writeData();
  }
}

void servoControl()
{
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

void serialEvent()
{
  if (Serial.available() > 0)
  {
    PC.readData();
  }
}
