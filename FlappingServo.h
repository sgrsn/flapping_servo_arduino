#ifndef FLAPPINGSERVO_H
#define FLAPPINGSERVO_H

#include <AS5601.h>
#include <Wire.h>
#include <Servo.h>
#include <PID.h>

#define IIC_SELECTOR_ADDRESS 0b1110000

uint16_t servo_min_us = 1004;
uint16_t servo_max_us = 2020;
uint16_t servo_mid_us = 1512;

uint16_t servo_min_cmd = 16;
int position_control_min_cmd = 1000;

enum class UsingEncoder
{
  BOTH = 0,
  AB = 1,
  I2C = 2,
};

enum class CommandMode
{
  FORCE = 0,
  POSITION = 1,
  SPEED = 2,
};

class FlappingServo
{
private:
  PID pid_;
  Servo esc_;
  AS5601_AB *encoder_;
  AS5601_I2C *encoder_abs_;
  uint8_t servo_pin_;
  uint8_t ch_;
  bool using_i2c_bus_;

  float A;
  float e;
  float prev_val;
  int values[100];
  int n;
  float prev_degree;

  float target_cmd_;
  float target_deg_;
  float target_speed_;
  float deg_abs_;
  float speed_abs_;
  int rev_;
  float deg_;
  float speed_;
  CommandMode mode_;

  float current_target_;

  unsigned long current_time;
  unsigned long prev_time;

  UsingEncoder using_encoder_;
  
public:
  FlappingServo(uint8_t servo_pin, AS5601_AB *encoder_ab, AS5601_I2C *encoder_i2c)
  {
    servo_pin_ = servo_pin;
    using_i2c_bus_ = false;
    A = 5.5;
    e = 0.0001;
    prev_val = 0;
    n = 100;
    for(int i = 0; i < n; i++) values[i] = 0;
    prev_degree = 0;
    target_cmd_ = 0;
    target_deg_ = 0;
    target_speed_ = 0;
    deg_abs_ = 0;
    rev_ = 0;
    deg_ = 0;
    speed_ = 0;
    mode_ = CommandMode::FORCE;
    current_target_ = 0;
    encoder_ = encoder_ab;
    encoder_abs_ = encoder_i2c;
    using_encoder_ = UsingEncoder::BOTH;
  }
  FlappingServo(uint8_t servo_pin, AS5601_I2C *encoder_i2c)
  {
    servo_pin_ = servo_pin;
    using_i2c_bus_ = false;
    A = 5.5;
    e = 0.0001;
    prev_val = 0;
    n = 100;
    for(int i = 0; i < n; i++) values[i] = 0;
    prev_degree = 0;
    target_cmd_ = 0;
    target_deg_ = 0;
    target_speed_ = 0;
    deg_abs_ = 0;
    rev_ = 0;
    deg_ = 0;
    speed_ = 0;
    mode_ = CommandMode::FORCE;
    current_target_ = 0;
    encoder_abs_ = encoder_i2c;
    using_encoder_ = UsingEncoder::I2C;
  }
  FlappingServo(uint8_t servo_pin, AS5601_I2C *encoder_i2c, uint8_t ch)
  {
    servo_pin_ = servo_pin;
    ch_ = ch;
    using_i2c_bus_ = true;
    A = 5.5;
    e = 0.0001;
    prev_val = 0;
    n = 100;
    for(int i = 0; i < n; i++) values[i] = 0;
    prev_degree = 0;
    target_cmd_ = 0;
    target_deg_ = 0;
    target_speed_ = 0;
    deg_abs_ = 0;
    rev_ = 0;
    deg_ = 0;
    speed_ = 0;
    mode_ = CommandMode::FORCE;
    current_target_ = 0;
    encoder_abs_ = encoder_i2c;
    using_encoder_ = UsingEncoder::I2C;
  }
  void init()
  {
    pid_.setParameter(3.0, 10.0, 0.1);
    //pid_.setParameter(0.6, 3.0, 0.01);
    if(using_encoder_ != UsingEncoder::I2C)
    {
      encoder_ -> setDirection(EncoderDirection::CCW);
    }
    if(using_encoder_ != UsingEncoder::AB)
    {
      if(using_i2c_bus_)
        setIICDevice(ch_);
      encoder_abs_ -> init();
      encoder_abs_ -> setDirection(EncoderDirection::CW);
    }
    delay(100);
    esc_.attach(servo_pin_, servo_min_us, servo_max_us);
    delay(100);
    esc_.writeMicroseconds(servo_mid_us);
    measureTimeInterval();
  }

  // Do this method in loop()
  // Donot run in MsTimer
  void updateAbsoluteEncoder()
  {
    if(using_i2c_bus_)
    {
      setIICDevice(ch_);
      delayMicroseconds(5);
    }
    encoder_abs_ -> update();
    deg_abs_ = encoder_abs_ -> getDegree();
    speed_abs_ = encoder_abs_ -> getDegreePerSeconds();
    fixAbsoluteRev();
    float dt = measureTimeInterval();
    speed_ = (deg_ - prev_degree) / dt;
    speed_ = smoothing(speed_);
    
    prev_degree = deg_;
  }

  // using PCA9547D
  // http://akizukidenshi.com/download/ds/nxp/PCA9547.pdf
  void setIICDevice(uint8_t device)
  {
    Wire.beginTransmission(IIC_SELECTOR_ADDRESS);
    Wire.write( 0b0001000 | (device & 0b0111) );
    Wire.endTransmission();
  }
  float smoothing(float val)
  {
    float sum = 0;
    for(int i = 0; i < n-1; i++)
    {
      values[i] = values[i+1];
      sum += values[i];
    }
    values[n-1] = val;
    sum += values[n-1];
    return sum / n;
  }
  
  float getDegree()
  {
    return deg_;
  }
  float getSpeed()
  {
    return speed_;
  }

  float measureTimeInterval()
  {
    prev_time = current_time;
    current_time = micros();
    float dt = float(current_time - prev_time) / 1000000.0;
    return dt;
  }
  void stopMotor()
  {
    esc_.writeMicroseconds(servo_mid_us);
  }
  void fixErrorFromAbsoluteCount(float abs_count)
  { 
    int resolution = encoder_ -> getEncoderResolution();
    encoder_ -> setCount( rev_*resolution + abs_count );
  }
  void fixAbsoluteRev()
  {
    if(speed_abs_ > 10000)
    {
      rev_--;
    }
    else if(speed_abs_ < -10000)
    {
      rev_++;
    }
    deg_ = rev_*360.0 + encoder_abs_ -> getDegree();
  }
  void controlMotor()
  {
    switch(mode_)
    {
      case CommandMode::FORCE:
        forceCommandESC(target_cmd_);
        break;
      case CommandMode::POSITION:
        controlPosition();
        break;
      case CommandMode::SPEED:
        controlSpeed();
        break;
    }
  }
  void commandMotor(float cmd)
  {
    float u = constrain(cmd, -position_control_min_cmd, position_control_min_cmd);
    if(servo_min_cmd < abs( u ))
    {
      int servo_us = servo_mid_us + ((float)(servo_max_us-servo_mid_us)/(1000.0)) * u;
      esc_.writeMicroseconds(servo_us);
    }
    else
    {
      esc_.writeMicroseconds(servo_mid_us);
    }
  }
  void controlPosition()
  {
    //float deg = encoder_ -> getDegree();
    //float deg = rev_*360.0 + encoder_abs_ -> getDegree();
    
    float u = pid_.control_I_PD(target_deg_, deg_);
    commandMotor( u );
  }

  void controlSpeed()
  {
    A += (speed_ - target_speed_) * e;
    float u = target_speed_ / A; 
    commandMotor( u );
  }
  void controlLowSpeed()
  {
    // staticをなくすべき
    static unsigned long prev_time = 0;
    static float target_deg = deg_;
    unsigned long current_time = micros();
    float lost_time = float(current_time - prev_time) / 1000000.0;
    prev_time = current_time;
    target_deg += target_speed_ * lost_time;
    float u = pid_.control_I_PD(target_deg, deg_);
    commandMotor( u );    
  }
  void controlAbsolutePosition()
  {
    float deg = encoder_ -> getDegree();
    float error0 = abs(target_deg_ - deg);
    float error1 = abs(target_deg_ - (deg - 360) );
    float error2 = abs(target_deg_ - (deg + 360) );
    if(error0 < error1)
    {
      if(error0 < error2) deg = deg;
      else  deg = deg + 360;
    }
    else if(error1 < error2)
    {
      deg = deg - 360;
    }
    else
      deg = deg + 360;
    // I-PDだと制御量も入力に使われるからだめか
    float u = pid_.control_I_PD(target_deg_, deg);
    commandMotor( u );
  }
  void forceCommandESC(int cmd) // cmd: -1000 to 1000
  {
    int servo_com = servo_mid_us + ((float)(servo_max_us-servo_mid_us)/(1000.0)) * cmd;
    esc_.writeMicroseconds(servo_com);
  }
  void pidReset()
  {
    pid_.reset();
  }
  void setPositionPIDParameter(float p, float i, float d)
  {
    pid_.setParameter(p, i, d);
  }

  void setTarget(float target, CommandMode mode)
  {
    mode_ = mode;
    current_target_ = target;
    switch(mode_)
    {
      case CommandMode::FORCE:
        target_cmd_ = target;
        break;
      case CommandMode::POSITION:
        target_deg_ = target;
        break;
      case CommandMode::SPEED:
        target_speed_ = target;
        break;
    }
  }
  void setTargetDegree(float target_deg)
  {
    target_deg_ = target_deg;
  }
  void setTargetSpeed(float target_speed)
  {
    target_speed_ = target_speed;
  }
  float currentTargetDegree()
  {
    return target_deg_;
  }
  float currentTargetSpeed()
  {
    return target_speed_;
  }
  float currentTarget()
  {
    return current_target_;
  }
};

#endif  // FLAPPINGSERVO_H
