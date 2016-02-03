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
      memset((void *)samples_, 0, sizeof(samples_));
      attachInterrupt(digitalPinToInterrupt(speed_in_pin_), pulse_sample, FALLING);
      attachInterrupt(digitalPinToInterrupt(start_stop_pin_), start_stop, RISING);
      timer_.begin(timer_sample, TIMER_SAMPLE_MS_ * 1000);
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

    static void speed_up() {
      if (v_control_ < 4095) {
        int error = target_rpm_ - rpm_;
        int step = min(32, max(1, error >> 3));
        v_control_ += min(step, (int)(4095 - v_control_));        
        analogWrite(speed_out_pin_, v_control_);
        Serial.println(v_control_);
      }
    }

    static void slow_down()
    {
      if (v_control_ > 0) {
        int error = rpm_ - target_rpm_;
        int step = min(32, max(1, error >> 3));
        v_control_ -= min(step, (int)v_control_);        
        analogWrite(speed_out_pin_, v_control_);
        Serial.println(v_control_);
        }
    }
    
  private:

    static void start_stop() {
      noInterrupts();
      if (millis() - last_start_stop_ > 500 || millis() < last_start_stop_) {
        last_start_stop_ = millis();
        if (target_rpm_ == 0) {
          Serial.println("START");
          target_rpm_ = RUN_SPEED;
        } else {
          Serial.println("STOP");
          target_rpm_ = 0;        
        }
      }
      interrupts();
    }

    static void pulse_sample() {
      noInterrupts();
      unsigned long now = micros();
      if (last_pulse_us_ != 0 && last_pulse_us_ < now) {
        samples_[cur_sample_] = (1000000UL / (now - last_pulse_us_)) * 15;
        cur_sample_ = (cur_sample_ + 1) % PULSE_SAMPLE_COUNT_;
      }
      last_pulse_us_ = now;
      interrupts();
    }

    static void timer_sample() {
      noInterrupts();
      unsigned long now = micros();
      if (last_pulse_us_ != 0 && (now - last_pulse_us_) > 500000) {
        memset((void*)samples_, 0, sizeof(samples_));
      }
      calc_rpm();
      
      if (rpm_ < target_rpm_) {
        speed_up();
      } else if (rpm_ > target_rpm_) {
        slow_down();
      }
      interrupts();
    }

    static void calc_rpm() {
      unsigned long avg = 0;
      for (unsigned int i = 0; i < PULSE_SAMPLE_COUNT_; ++i) {
        avg += samples_[i];
      }
      rpm_ = (unsigned int)(avg / PULSE_SAMPLE_COUNT_);
    }

    IntervalTimer timer_;
    int start_stop_pin_;
    int speed_in_pin_;
    
    static int speed_out_pin_;
    static const unsigned int PULSE_SAMPLE_COUNT_ = 16;
    static const unsigned int TIMER_SAMPLE_MS_ = 50;

    static volatile int samples_[PULSE_SAMPLE_COUNT_];
    static volatile unsigned long last_pulse_us_;
    static volatile unsigned int cur_sample_;
    
    static unsigned int last_start_stop_;
    static volatile unsigned int target_rpm_;
    static volatile unsigned int rpm_;
    static volatile unsigned int v_control_;
};

int Motor::speed_out_pin_;
volatile int Motor::samples_[PULSE_SAMPLE_COUNT_];
volatile unsigned long Motor::last_pulse_us_ = 0;
volatile unsigned int Motor::cur_sample_ = 0;

unsigned int Motor::last_start_stop_ = 0;
volatile unsigned int Motor::target_rpm_ = 0;
volatile unsigned int Motor::rpm_ = 0;
volatile unsigned int Motor::v_control_ = 0;

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
  display(motor.get_rpm());
  delay(100);
}
