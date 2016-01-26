#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

#define START_STOP_PIN 22
#define SPEED_IN_PIN 23
#define SPEED_OUT_PIN A14
#define RUN_SPEED 720

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
      attachInterrupt(digitalPinToInterrupt(speed_in_pin_), speed_pulse, FALLING);
      attachInterrupt(digitalPinToInterrupt(start_stop_pin_), start_stop, RISING);
    }

    ~Motor() {
      detachInterrupt(digitalPinToInterrupt(speed_in_pin_));
      detachInterrupt(digitalPinToInterrupt(start_stop_pin_));
    }

    static unsigned int get_rpm() {
      noInterrupts();
      unsigned int rpm = Motor::rpm_;
      interrupts();
      return rpm;
    }

  private:

    static void start_stop() {
      noInterrupts();
      if (speed_target_ == 0) {
        speed_target_ = RUN_SPEED;
      } else {
        speed_target_ = 0;        
      }
      interrupts();
    }

    static void speed_pulse() {
      noInterrupts();      
      unsigned long now = micros();
      if (last_pulse_us_ == 0 || now < last_pulse_us_) {
        last_pulse_us_ = now;
        // first pulse or rollover = bad sample
        return;
      }
      last_pulse_us_ = now;
      rpm_ = 15 * (1000000UL / (now - last_pulse_us_));

      if (rpm_ < speed_target_ && v_control_ < 4095) {
        analogWrite(speed_out_pin_, ++v_control_);
      } else if (rpm_ > speed_target_ && v_control_ > 0) {
        analogWrite(speed_out_pin_, --v_control_);
      }
      
      interrupts();
    }

    int start_stop_pin_;
    int speed_in_pin_;
    static int speed_out_pin_;
    
    static volatile unsigned int speed_target_;
    static volatile unsigned int v_control_;
    static volatile unsigned long last_pulse_us_;
    static volatile unsigned long pulse_count_;
    static volatile unsigned int rpm_;
};

int Motor::speed_out_pin_;
volatile unsigned int Motor::speed_target_ = 0;
volatile unsigned int Motor::v_control_ = 0;
volatile unsigned long Motor::last_pulse_us_ = 0;
volatile unsigned int Motor::rpm_ = 0;

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
}
