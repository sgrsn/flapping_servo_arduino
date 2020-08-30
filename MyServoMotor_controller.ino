#include <PacketSerial.h>
#include <MsTimer2.h>
#include <AS5601.h>
#include <Servo.h>
#include <PID.h>
#include "RegisterMap.h"
#include "FlappingServo.h"

/*TO DO***********************************************************
動かしているとサーボからピーという音がして動作しなくなる
発熱エラー？
冷ましてから再実験
Motor TimingをHighからMidiumかMidium Highに変更すると収まった気がする
その代わり遅くなった？
I-PDでKp=2, Ki=8, Kd=0.1でいいかんじ

アブソリュートで位置制御がむずい
absluteだけだと通信が遅いので、AB+absoluteでいい感じにできれば
今のコードでいい感じだが、0, 360あたりで暴走する

アブソリュートだけでもrevを考慮すればいけるっぽいな
******************************************************************/

int32_t Register[128] = {};
PacketSerial PC(Register);
uint8_t servoPin = 11;
AS5601_AB encoder_ab;
AS5601_I2C encoder_i2c;
FlappingServo my_servo(servoPin, &encoder_ab, &encoder_i2c);

uint8_t control_T = 1; // ms
uint8_t write_T = 25; // ms
uint8_t fix_T = 10;

void setup() {
  pinMode(13, OUTPUT);
  encoder_ab.init(2, 3, encoderInterrupt);
  my_servo.init();
  Serial.begin(1000000);
  my_servo.stopMotor();
  MsTimer2::set(control_T, tick);
}

float deg_abs = 0;
long count_abs = 0;
float speed_abs = 0;

void loop() {

  //digitalWrite(13, Register[COMMAND_LED]);
  //setPIDparameter();
  //delay(1);
  encoder_ab.update();
  encoder_i2c.update();
  deg_abs = encoder_i2c.getDegree();
  count_abs = encoder_i2c.getEncoderCount();
  speed_abs = encoder_i2c.getDegreePerSeconds();
  my_servo.fixAbsoluteRev(speed_abs);
  my_servo.setAbsoluteDegree( deg_abs );
  
  //if(abs(Register[COMMAND_MOTOR]) < 1000)
  if( Register[COMMAND_MOTOR] == Register[COMMAND_MOTOR_CONFIRM] )
    my_servo.setTargetDegree( Register[COMMAND_MOTOR] );
  
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
    Register[COMMAND_STOP] = 0;
    MsTimer2::stop();
    my_servo.stopMotor();
  }
  
}

void encoderInterrupt()
{
  encoder_ab.interruptEncoderCount();
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
  if(t_2 > fix_T/control_T)
  {
    t_2 = 0;
    fixError();
  }
}

void servoControl()
{
  my_servo.controlPosition();
  //my_servo.controlAbsolutePosition();
  // my_servo2.controlPosition();
}

void fixError()
{
  my_servo.fixErrorFromAbsoluteCount(count_abs/2);
}

void setPIDparameter()
{
  float p = float(Register[PARAMETER_P]) / 1000.0;
  float i = float(Register[PARAMETER_I]) / 1000.0;
  float d = float(Register[PARAMETER_D]) / 1000.0;
  my_servo.setPIDParameter(p, i, d);
}

void writeData()
{
  static int func_switch = 0;
  if(func_switch == 0)
  {
    int deg = encoder_ab.getDegree();
    PC.writeDataWithSize(deg, MOTOR_DEGREE_INC);
    func_switch = 1;
  }
  else if(func_switch == 1)
  {
    //PC.writeDataWithSize((int)encoder_ab.getDegreePerSeconds(), MOTOR_SPEED);
    PC.writeDataWithSize(deg_abs, MOTOR_DEGREE_ABS); 
    func_switch = 2;
  }
  else if(func_switch == 2)
  {
    PC.writeDataWithSize((int)my_servo.currentTargetDegree(), CURRENT_COMMAND);
    func_switch = 3;
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
