# pragma once

namespace Types {
  // inicializar los motores 
  struct Motor {
    int EN;
    int IN1;
    int IN2;

    // pines para encoders
    int pinEncoderA; // pin de interrupcion (CLK)
    int pinEncoderB; // pin de direccion (DT)
    
    // contador de pulsos (volatil porque se usa en interrupciones)
    volatile long ticks; 
  };

  struct IR {
    int pin;
  };
  
  struct UltraSonic {
    int echo;
    int trigger;
  };

  struct Robot {
    Motor motor1; // motor izquierdo
    Motor motor2; // motor derecho
    IR leftIR;
    IR centerIR;
    IR rightIR;
    UltraSonic leftUS;
    UltraSonic centerUS;
    UltraSonic rightUS;
  };

}