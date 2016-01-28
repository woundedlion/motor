#include <Adafruit_LEDBackpack.h>

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

#define START_STOP_PIN 22
#define SPEED_IN_PIN 23
#define SPEED_OUT_PIN A14
#define RUN_SPEED 480

class Motor
{
  public:

    Motor(int speed_in_pin, int speed_out_pin, int start_stop_pin) :
    start_stop_pin_(start_stop_pin),
    speed_in_pin_(speed_in_pin)
    {
      Motor::speed_out_pin_ = speed_out_pin;
      pinMode(start_stop_pin_, INPUT);
      pinMode(speed_in_pin_, INPUT);
      pinMode(speed_out_pin_, OUTPUT);
      attachInterrupt(digitalPinToInterrupt(speed_in_pin_), pulse, FALLING);
      attachInterrupt(digitalPinToInterrupt(start_stop_pin_), start_stop, RISING);
      timer_.begin(pid_sample, 100000);
    }

    ~Motor() {
      detachInterrupt(digitalPinToInterrupt(speed_in_pin_));
      detachInterrupt(digitalPinToInterrupt(start_stop_pin_));
      timer_.end();
    }

    static unsigned int get_rpm() {
      noInterrupts();
      unsigned int r = Motor::rpm_;
      interrupts();
      return r;
    }

    static unsigned int get_target_speed() {
      noInterrupts();
      unsigned int r = Motor::target_speed_;
      interrupts();
      return r;
    }

    static void speed_up() {
      if (v_control_ < 4095) {
        analogWrite(speed_out_pin_, ++v_control_);
        Serial.println(v_control_);
      }
    }

    static void slow_down()
    {
      if (v_control_ > 0) {
        analogWrite(speed_out_pin_, --v_control_);
        Serial.println(v_control_);
        }
    }
    
  private:

    static void start_stop() {
      noInterrupts();
      if (target_speed_ == 0) {
        Serial.println("START");
        target_rpm_ = RUN_SPEED;
      } else {
        Serial.println("STOP");
        target_rpm_ = 0;        
      }
      interrupts();
    }

    static void pulse() {
      noInterrupts();
      ++count;
      interrupts();
    }

    static void pid_sample() {
      noInterrupts();
      int rpm = 15 * count_;
      count_ = 0;
      int error = rpm - target_rpm_;
      error = rpm_
      i_state_ += error;
      i_state = max(min_i_, min(max_i_, i_state));
      int step = min(max_step_, 
        ((p_gain * error) + (i_gain * i_state_) - (d_gain * (rpm - rpm_))));
      v_control_ += step;
      rpm_ = rpm;
      interrupts();
    }

    IntervalTimer timer_;
    int start_stop_pin_;
    int speed_in_pin_;
    
    static int speed_out_pin_;
    static volatile unsigned int target_rpm_;
    static volatile int rpm_;
    static volatile unsigned int count_;
    static volatile unsigned int v_control_;
    static volatile int i_state_;
    const static volatile int p_gain_;
    const static volatile int i_gain_;
    const static volatile int d_gain_;    
};

int Motor::speed_out_pin_;
volatile unsigned int Motor::target_rpm_ = 0;
volatile int Motor::rpm_ = 0;
volatile unsigned int Motor::count_ = 0;
volatile unsigned int Motor::v_control_ = 0;
volatile int Motor::i_state_ = 0;
const int Motor::p_gain_ = 0;
const int Motor::i_gain_ = 0;
const int Motor::d_gain_ = 0;    

Motor motor(SPEED_IN_PIN, SPEED_OUT_PIN, START_STOP_PIN);
Adafruit_AlphaNum4 lcd = Adafruit_AlphaNum4();

void setup() {
  Serial.begin(9600);  
  analogWriteResolution(12);
  lcd.begin(0x70);  // pass in the address
}

void display(int n) {
  String s(n);
  lcd.clear();
  for (unsigned int i = 0; i < s.length() && i < 4; ++i) {
    lcd.writeDigitAscii(i, s.charAt(i));  
  }
  lcd.writeDisplay();
}

void loop() {
  Serial.println("INIT");
  unsigned long rpm = motor.get_rpm();
  unsigned int target_speed = motor.get_target_speed();
  display(rpm);
  if (rpm < target_speed) {
    motor.speed_up(); 
  } else if (rpm > target_speed) {
    motor.slow_down();
  }
  delay(100);
}
