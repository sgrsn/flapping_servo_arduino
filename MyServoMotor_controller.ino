#include <PacketSerial.h>
#include <MsTimer2.h>
#include <AS5601.h>
#include <Servo.h>
#include <PID.h>
#include "RegisterMap.h"
#include "FlappingServo.h"

/*TO DO***********************************************************

******************************************************************/

int32_t Register[128] = {};
PacketSerial PC(Register);
uint8_t servoPin = 11;
AS5601_I2C encoder_i2c;
FlappingServo my_servo(servoPin, &encoder_i2c);

uint8_t control_T = 1; // ms
uint8_t write_T = 25; // ms
uint8_t fix_T = 10;

void setup() {
  pinMode(13, OUTPUT);
  my_servo.init();
  Serial.begin(1000000);
  my_servo.stopMotor();
  MsTimer2::set(control_T, tick);
}

float deg_abs = 0;
float deg_inc = 0;
float speed_abs = 0;
float speed_inc = 0;

float deg = 0;
float deg_per_sec = 0;

void loop() {
  //delay(10);
  //digitalWrite(13, Register[COMMAND_LED]);
  //setPIDparameter();
  //delay(1);

  my_servo.updateAbsoluteEncoder();
  deg = my_servo.getDegree();
  deg_per_sec = my_servo.getSpeed(); 
  
  //if(abs(Register[COMMAND_MOTOR]) < 1000)
  if( Register[COMMAND_MOTOR] == Register[COMMAND_MOTOR_CONFIRM] )
    //my_servo.forceCommandESC(Register[COMMAND_MOTOR]);
    //my_servo.setTargetDegree( Register[COMMAND_MOTOR] );
    my_servo.setTargetSpeed(Register[COMMAND_MOTOR]);
  
  if(Register[COMMAND_START] > 0)
  {
    digitalWrite(13, HIGH);
    PC.writeDataWithSize(123, START_REPLY);
    Register[COMMAND_START] = 0;
    MsTimer2::start();
    my_servo.pidReset();
  }
  if(Register[COMMAND_STOP] > 0)
  {
    digitalWrite(13, LOW);
    Register[COMMAND_STOP] = 0;
    MsTimer2::stop();
    my_servo.stopMotor();
  }
}

int t_ = 0;
int t_2 = 0;
void tick()
{ 
  t_++;
  t_2++;
  servoControl();
  if(t_ > write_T/control_T)
  {
    t_ = 0;
    writeData();
  }
}

void servoControl()
{
  //my_servo.controlPosition();
  my_servo.controlSpeed();
}

void setPIDparameter()
{
  float p = float(Register[PARAMETER_P]) / 1000.0;
  float i = float(Register[PARAMETER_I]) / 1000.0;
  float d = float(Register[PARAMETER_D]) / 1000.0;
  my_servo.setPositionPIDParameter(p, i, d);
}

void writeData()
{
  static int func_switch = 0;
  if(func_switch == 0)
  {
    PC.writeDataWithSize(deg, MOTOR_DEGREE_INC);
    func_switch = 1;
  }
  else if(func_switch == 1)
  {
    PC.writeDataWithSize(deg_per_sec, MOTOR_SPEED_INC); 
    func_switch = 2;
  }
  else if(func_switch == 2)
  {
    //PC.writeDataWithSize((int)my_servo.currentTargetDegree(), CURRENT_COMMAND);
    PC.writeDataWithSize((int)my_servo.currentTargetSpeed(), CURRENT_COMMAND);
    func_switch = 0;
  }
  else
    func_switch = 0;
}

void serialEvent()
{
  if (Serial.available() > 0)
  {
    PC.readData();
  }
}
