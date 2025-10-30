# pragma once
#include <Arduino.h>
namespace CONSTANTS {
  // variables motores
  const int ENA = 1;
  const int IN1 = 2; //interrupción
  const int IN2 = 3; //interrupción
  const int ENB = 4;
  const int IN3 = 5;
  const int IN4 = 6;

  // asegurar que pines A estén en pines de "interrupción"
  
  // motor 1 (izquierda)
  const int M1_ENCODER_A = 2; // pin de interrupcion, como pin 2
  const int M1_ENCODER_B = 7; // pin normal 

  // motor 2 (derecha)
  const int M2_ENCODER_A = 3; // pin de interrupcion, como pin 3
  const int M2_ENCODER_B = A0; // pin normal 


  // variables IRs
  const int leftIR = 8;
  const int centerIR = 9;
  const int rightIR = 10;

  // ultrasonics
  const int echoLeft = 11;
  const int trigLeft = 12;
  const int echoCenter = 13;
  const int trigCenter = 14;
  const int echoRight = 15;
  const int trigRight = 16;

}