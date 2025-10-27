#include "constants.h"
#include "funciones.h"
#include "types.h"

void setup() {

  // setup ines robot
  pinMode(CONSTANTS::ENA, OUTPUT);
  pinMode(CONSTANTS::IN1, OUTPUT);  
  pinMode(CONSTANTS::IN2, OUTPUT);  
  pinMode(CONSTANTS::ENB, OUTPUT); 
  pinMode(CONSTANTS::IN3, OUTPUT);  
  pinMode(CONSTANTS::IN4, OUTPUT);  

  // setup pines IRs
  pinMode(CONSTANTS::centerIR, INPUT);
  pinMode(CONSTANTS::rightIR, INPUT);
  pinMode(CONSTANTS::leftIR, INPUT);




}

void loop() {
  // put your main code here, to run repeatedly:

}
