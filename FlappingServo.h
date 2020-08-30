#ifndef FLAPPINGSERVO_H
#define FLAPPINGSERVO_H

#include <AS5601.h>
#include <Servo.h>
#include <PID.h>

uint16_t servo_min_us = 1004;
uint16_t servo_max_us = 2020;
uint16_t servo_mid_us = 1512;

uint16_t servo_min_cmd = 16;
int position_control_min_cmd = 1000;

class FlappingServo
{
private:
  PID pid_;
  Servo esc_;
  AS5601_AB *encoder_;
  AS5601_I2C *encoder_abs_;
  uint8_t servo_pin_;

  float target_deg_;
  float deg_abs_;
  int rev_;
public:
  FlappingServo(uint8_t servo_pin, AS5601_AB *encoder_ab, AS5601_I2C *encoder_i2c)
  {
    servo_pin_ = servo_pin;
    target_deg_ = 0;
    deg_abs_ = 0;
    rev_ = 0;
    encoder_ = encoder_ab;
    encoder_abs_ = encoder_i2c;
  }
  FlappingServo(uint8_t servo_pin, AS5601_AB *encoder_ab)
  {
    servo_pin_ = servo_pin;
    target_deg_ = 0;
    encoder_ = encoder_ab;
  }
  FlappingServo(uint8_t servo_pin, AS5601_I2C *encoder_i2c)
  {
    servo_pin_ = servo_pin;
    target_deg_ = 0;
    encoder_abs_ = encoder_i2c;
  }
  void init()
  {
    pid_.setParameter(3.0, 10.0, 0.1);
    encoder_ -> setDirection(EncoderDirection::CCW);
    encoder_abs_ -> init();
    encoder_abs_ -> setDirection(EncoderDirection::CW);
    delay(100);
    esc_.attach(servo_pin_, servo_min_us, servo_max_us);
    delay(100);
    esc_.writeMicroseconds(servo_mid_us);
  }
  void stopMotor()
  {
    esc_.writeMicroseconds(servo_mid_us);
  }
  void setAbsoluteDegree(float deg)
  {
    deg_abs_ = deg;
  }
  void fixErrorFromAbsoluteCount(float abs_count)
  { 
    int resolution = encoder_ -> getEncoderResolution();
    encoder_ -> setCount( rev_*resolution + abs_count );
  }
  void fixAbsoluteRev(float speed_abs)
  {
    if(speed_abs > 10000)
    {
      rev_--;
    }
    else if(speed_abs < -10000)
    {
      rev_++;
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
    float deg = encoder_ -> getDegree();
    //float deg = rev_*360.0 + encoder_abs_ -> getDegree();
    
    float u = pid_.control_I_PD(target_deg_, deg);
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
  void setPIDParameter(float p, float i, float d)
  {
    pid_.setParameter(p, i, d);
  }
  void setTargetDegree(float target_deg)
  {
    target_deg_ = target_deg;
  }
  float currentTargetDegree()
  {
    return target_deg_;
  }
};

#endif  // FLAPPINGSERVO_H
