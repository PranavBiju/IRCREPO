#include "cytrons.h"

MDD10A::MDD10A(int pwm, int dir, int enca, int encb)
  : _pwm_pin(pwm), _dir_pin(dir), _enca(enca), _encb(encb) {

  pinMode(_pwm_pin, OUTPUT);
  pinMode(_dir_pin, OUTPUT);
  pinMode(_enca, INPUT);
  pinMode(_encb, INPUT);
}

void MDD10A::run(int pwr) {
  int dir = (pwr > 0) ? HIGH : LOW;
  int pwm_value = min(abs(pwr), 255);

  digitalWrite(_dir_pin, dir);
  analogWrite(_pwm_pin, pwm_value);
}

void MDD10A::stop() {
  analogWrite(_pwm_pin, 0); 
  digitalWrite(_dir_pin, LOW);
}
