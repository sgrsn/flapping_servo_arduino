#ifndef FLAPPINGSERVO_H
#define FLAPPINGSERVO_H

#include <AS5601.h>
#include <Wire.h>
#include <Servo.h>
#include <PID.h>
#include <filters.h>

#define IIC_SELECTOR_ADDRESS 0b1110000

int servo_min_us = 1004;
int servo_max_us = 2020;
int servo_mid_us = 1512;

int servo_min_cmd = 16;
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
public:
  PID pid_;
  Servo esc_;
  AS5601_AB *encoder_;
  AS5601_I2C *encoder_abs_;
  uint8_t servo_pin_;
  uint8_t ch_;
  bool using_i2c_bus_;

  float A;
  float e_speed;
  float e_deg;
  float Kp;
  float prev_degree;
  float target_cmd_;
  float target_deg_;
  float target_speed_;
  float deg_abs_;
  float speed_abs_;
  int rev_;
  float deg_;
  float speed_;
  float last_speed_;
  float accel_abs;
  CommandMode mode_;
  float current_target_;
  float deg_another_;
  unsigned long current_time;
  unsigned long prev_time;
  unsigned long prev_time_forControl;
  float target_deg_forControl;
  UsingEncoder using_encoder_;
  float _dir;
  //Filter filter;
  
public:
  FlappingServo(uint8_t servo_pin, AS5601_AB *encoder_ab, AS5601_I2C *encoder_i2c)// : filter(0, 0, IIR::ORDER::OD3)
  {
    servo_pin_ = servo_pin;
    using_i2c_bus_ = false;
    mode_ = CommandMode::FORCE;
    encoder_ = encoder_ab;
    encoder_abs_ = encoder_i2c;
    using_encoder_ = UsingEncoder::BOTH;
  }
  FlappingServo(uint8_t servo_pin, AS5601_I2C *encoder_i2c)// : filter(0, 0, IIR::ORDER::OD3)
  {
    servo_pin_ = servo_pin;
    using_i2c_bus_ = false;
    mode_ = CommandMode::FORCE;
    encoder_abs_ = encoder_i2c;
    using_encoder_ = UsingEncoder::I2C;
  }
  FlappingServo(uint8_t servo_pin, AS5601_I2C *encoder_i2c, uint8_t ch)// : filter(0, 0, IIR::ORDER::OD3)
  {
    servo_pin_ = servo_pin;
    ch_ = ch;
    using_i2c_bus_ = true;
    mode_ = CommandMode::FORCE;
    encoder_abs_ = encoder_i2c;
    using_encoder_ = UsingEncoder::I2C;
  }
  void init()
  {
    A = 5.5;
    current_target_ = 0;
    prev_degree = 0;
    target_cmd_ = 0;
    target_deg_ = 0;
    target_speed_ = 0;
    deg_abs_ = 0;
    rev_ = 0;
    deg_ = 0;
    speed_ = 0;
    prev_time_forControl = micros();
    target_deg_forControl = 0;
    deg_another_ = 0;
    e_deg = 0.0001;
    e_speed = 0.001;//0.0005;
    Kp = 0.5;
    _dir = 1;
    //pid_.setParameter(3.0, 10.0, 0.1);  // for I-PD
    pid_.setParameter(2.14282, 0.98876, 0.00); // for PID +/-60deg
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
    if(_dir < 0)
    {
      deg_abs_ = 360.0 - deg_abs_;
    }
    speed_abs_ = _dir * encoder_abs_ -> getDegreePerSeconds();
    accel_abs = _dir * encoder_abs_ -> getAcceleration();
    fixAbsoluteRev();
    float dt = measureTimeInterval();
    last_speed_ = speed_;
    speed_ = (deg_ - prev_degree) / dt;
    speed_ = 0.5*last_speed_ + 0.5*speed_;
    //speed_ = filter.filterIn(speed_);
    
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
    if(speed_abs_ > 5000)
    {
      rev_--;
    }
    else if(speed_abs_ < -5000)
    {
      rev_++;
    }
    deg_ = rev_*360.0 + deg_abs_;//encoder_abs_ -> getDegree();
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
        //controlSpeed();
        controlSpeedSync();
        break;
    }
  }
  void commandMotor(float cmd)
  {
    //float u = constrain(cmd, -position_control_min_cmd, position_control_min_cmd);
    float u = _dir * cmd;
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
    
    //float u = pid_.control_I_PD(target_deg_, deg_);
    float u = pid_.control_PID(target_deg_, deg_);
    commandMotor( u );
  }

  void controlSpeed()
  {
    A += (speed_ - target_speed_) * e_speed;
    float u = target_speed_ / A;
    commandMotor( u );
  }

  float u_err = 0;
  void controlSpeedSync()
  {
    /*if(target_speed_ > 0)
      A += (speed_ - target_speed_) * e + (deg_ - deg_another_) * e_deg;
    else if(target_speed_ < 0)
      A += (target_speed_ - speed_) * e + (deg_another_ - deg_) * e_deg;
    float u = target_speed_ / A;
    commandMotor( u );*/
    u_err += (target_speed_ - speed_) * e_speed + (deg_another_ - deg_) * e_deg;
    float u = (target_speed_ - speed_)*Kp + u_err; //target_speed_ / A + u_err;
    commandMotor( u );
  }

  void controlSpeedBasedPosition()
  {
    unsigned long current_time = micros();
    float lost_time = float(current_time - prev_time_forControl) / 1000000.0;
    prev_time_forControl = current_time;
    target_deg_forControl += target_speed_ * lost_time;
    float u = pid_.control_I_PD(target_deg_forControl, deg_);
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
  void forceCommandESC(float cmd) // cmd: -1000 to 1000
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
        /*const float cutoff_freq   = target / 360.0;  //Cutoff frequency in Hz = deg/sec * 1 rev /360 deg
        const float sampling_time = 0.001; //Sampling time in seconds.
        IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)
        filter = Filter(cutoff_freq, sampling_time, order);*/
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
  void setAnotherMotorDegree(float deg)
  {
    deg_another_ = deg;
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
  void setServoDirectionReverse()
  {
    _dir = -1;
  }
  float getAbsSpeed()
  {
    return speed_abs_;
  }
  float getAbsDegree()
  {
    return deg_abs_;
  }
  float getAbsAcceleration()
  {
    return accel_abs;
  }
  int getRevolution()
  {
    return rev_;
  }
  
};

#endif  // FLAPPINGSERVO_H
