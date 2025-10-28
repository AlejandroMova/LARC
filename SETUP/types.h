# pragma once

namespace Types {
  // inicializar los motores 
  struct Motor {
    int EN;
    int IN1;
    int IN2;
    
  };

  struct IR {
    int pin;
  };
  
  struct UltraSonic {
    int echo;
    int trigger;
  };

  struct Robot {
    Motor motor1;
    Motor motor2;
    IR leftIR;
    IR centerIR;
    IR rightIR;
    UltraSonic leftUS;
    UltraSonic centerUS;
    UltraSonic rightUS;
    ; 


  };

}