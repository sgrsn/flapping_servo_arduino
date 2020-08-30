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

enum class UsingEncoder
{
  BOTH = 0,
  AB = 1,
  I2C = 2,
};

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
  float speed_abs_;
  int rev_;
  float deg_;
  float speed_;

  unsigned long current_time;
  unsigned long prev_time;

  UsingEncoder using_encoder_;
  
public:
  FlappingServo(uint8_t servo_pin, AS5601_AB *encoder_ab, AS5601_I2C *encoder_i2c)
  {
    servo_pin_ = servo_pin;
    target_deg_ = 0;
    deg_abs_ = 0;
    rev_ = 0;
    deg_ = 0;
    speed_ = 0;
    encoder_ = encoder_ab;
    encoder_abs_ = encoder_i2c;
    using_encoder_ = UsingEncoder::BOTH;
  }
  FlappingServo(uint8_t servo_pin, AS5601_I2C *encoder_i2c)
  {
    servo_pin_ = servo_pin;
    target_deg_ = 0;
    deg_abs_ = 0;
    rev_ = 0;
    deg_ = 0;
    speed_ = 0;
    encoder_abs_ = encoder_i2c;
    using_encoder_ = UsingEncoder::I2C;
  }
  void init()
  {
    pid_.setParameter(3.0, 10.0, 0.1);
    if(using_encoder_ != UsingEncoder::I2C)
    {
      encoder_ -> setDirection(EncoderDirection::CCW);
    }
    if(using_encoder_ != UsingEncoder::AB)
    {
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
    encoder_abs_ -> update();
    deg_abs_ = encoder_abs_ -> getDegree();
    speed_abs_ = encoder_abs_ -> getDegreePerSeconds();
    fixAbsoluteRev();

    static float prev_degree = 0;
    float dt = measureTimeInterval();
    speed_ = (deg_ - prev_degree) / dt;
    prev_degree = deg_;
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
