#include <FastLED.h>
#include <lib8tion.h>

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

#define SPEED_IN_PIN 23
#define SPEED_OUT_PIN A14
#define RUN_SPEED 720

class Motor
{
  public:

    enum State
    {
      STOP,
      RAMP_UP,
      STEADY,
      RAMP_DOWN
    };

    enum Event
    {
      EVENT_STOP,
      EVENT_START,
      EVENT_SPEED_UP,
      EVENT_SPEED_DOWN
    };

    Motor(int speed_in_pin, int speed_out_pin)
    {
      Motor::speed_in_pin_ = speed_in_pin;
      Motor::speed_out_pin_ = speed_out_pin;
      pinMode(speed_in_pin_, INPUT);
      pinMode(speed_out_pin_, OUTPUT);
      attachInterrupt(speed_in_pin_, speed_pulse, FALLING);
    }

    ~Motor() {
      timer_.end();
    }

    static unsigned int get_rpm() {
      noInterrupts();
      unsigned int rpm = Motor::rpm_;
      interrupts();
      return rpm;
    }

    void set_speed(int speed) {
      noInterrupts();
      speed_target_ = speed;
      state_ = RAMP_UP;
      timer_.end();
      timer_.begin(speed_ramp, 1000000);
      interrupts();
    }

  private:
    
    static void speed_pulse() {
      noInterrupts();
      ++count_;
      interrupts();
    }
    
    static void speed_ramp() {
      noInterrupts();
      rpm_ = 15 * count_;
      count_ = 0;      
      if (rpm_ < (speed_target_ - 15)) {
        handle(EVENT_SPEED_UP);
      } else if (rpm_ > speed_target_) {
        handle(EVENT_SPEED_DOWN);
      } else {
        state_ = STEADY;
      }
      interrupts();
    }

    static void handle(Event e) {
      switch(state_) {
        case STOP:
          break;
          
        case RAMP_UP:
          switch(e) {
            case EVENT_SPEED_UP:
              if (v_control_ < 4095 - 10) {
                v_control_ += 10;
                analogWrite(speed_out_pin_, v_control_);
                Serial.println(v_control_);                
              }
              break;
            case EVENT_SPEED_DOWN:
            {
              if (v_control_ > 0 + 10) {
                v_control_ -= 10;
                analogWrite(speed_out_pin_, v_control_);
                Serial.println(v_control_);
              }
            }
              break;
            default:
              break;
          }
          break;
          
        case STEADY:
          switch(e) {
            case EVENT_SPEED_UP:
              if (v_control_ < 4095) {
                analogWrite(speed_out_pin_, ++v_control_);
                Serial.println(v_control_);                
              }
              break;
            case EVENT_SPEED_DOWN:
              if (v_control_ > 0) {
                analogWrite(speed_out_pin_, --v_control_);
                Serial.println(v_control_);
              }
              break;
            default:
              break;
          }
          break;
          
        default:
          break;
      }
}
    static int speed_in_pin_;
    static int speed_out_pin_;
    static volatile State state_;
    static volatile unsigned int speed_target_;
    static volatile unsigned int v_control_;
    static volatile unsigned int count_;
    static volatile unsigned int rpm_;
    IntervalTimer timer_;
};

int Motor::speed_in_pin_ = 0;
int Motor::speed_out_pin_ = 0;
volatile Motor::State Motor::state_ = Motor::STOP;
volatile unsigned int Motor::count_ = 0;
volatile unsigned int Motor::rpm_ = 0;
volatile unsigned int Motor::speed_target_ = 0;
volatile unsigned int Motor::v_control_ = 0;

Motor motor(SPEED_IN_PIN, SPEED_OUT_PIN);
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
  unsigned int s = millis();
  motor.set_speed(200);
  while (millis() - s < 10000) {
    display(motor.get_rpm());
  }
  motor.set_speed(0);
  while (1) {
    display(motor.get_rpm());
  }
}
