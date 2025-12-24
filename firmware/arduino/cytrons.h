#ifndef CYTRONS_H
#define CYTRONS_H

#include "Arduino.h"

class MDD10A {
  public:
    MDD10A(int pwm, int dir, int enca, int encb);

    void run(int pwr);
    void stop();

  private:
    int _pwm_pin;
    int _dir_pin;
    int _enca;
    int _encb;
};

#endif
