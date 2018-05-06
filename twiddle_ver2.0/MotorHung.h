/*

*/
#include <Arduino.h>

class MotorHung
{
  public:
    MotorHung();
    MotorHung(int PWMpin, int DIRpin) {
      
      TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
      TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz

      _PWMpin = PWMpin;
      _DIRpin = DIRpin;
      pinMode(_PWMpin, OUTPUT); 
      pinMode(_DIRpin, OUTPUT);
    };
    ~MotorHung() {
    };

    void setSpeedMotor(int PWM) {
      _PWM = PWM;
      constrain(_PWM, -250, 250);
      if (_PWM >= 0)
      {
        digitalWrite(_DIRpin, LOW);
        analogWrite(_PWMpin, _PWM);
      }
      else {
        digitalWrite(_DIRpin, HIGH);
        analogWrite(_PWMpin, abs(_PWM));
      }
    }

  public:
    int _PWMpin;
    int _DIRpin;
    int _PWM = 0;
    float _I = 0; //Cuong do dong dien qua dong co
    
    volatile long lastEncoder = 0;
    volatile long encoder = 0;
    float omega = 0; // Van toc goc
    
};











