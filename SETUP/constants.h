# pragma once
#include <Arduino.h>
namespace CONSTANTS {
  // variables motores
  const int ENA = 46;
  const int IN1 = 35; 
  const int IN2 = 33; 
  const int ENB = 44;
  const int IN3 = 29;
  const int IN4 = 31;

  // asegurar que pines A estén en pines de "interrupción"
  
  // motor 1 (izquierda)
  const int M1_ENCODER_A = 18; // pin de interrupcion, como pin 2
  const int M1_ENCODER_B = 19; // pin normal 

  // motor 2 (derecha)
  const int M2_ENCODER_A = 2; // pin de interrupcion, como pin 3
  const int M2_ENCODER_B = 3; // pin normal 


  // variables IRs
  const int leftIR = 48; 
  const int centerIR = 52;
  const int rightIR = 42;

  // ultrasonics
  const int echoLeft = 10;
  const int trigLeft = 11;
  const int echoCenter = 4; // cafe
  const int trigCenter = 5;
  const int echoRight = 7;
  const int trigRight = 6;


// int of MPU is 20
}