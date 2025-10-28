#include <funciones.h>
#include <types.h>
#include <constants.h>

// utilizamos right hand rule


// inicializar robot 
Types::Robot robot = INIT::init_robot();

// hace falta revisar tiempos y velocidad
// tiempo y velocidad para girar
int vGiro = 200;
int tGiro = 0.5;
// tiempo y velocidad para cambiar de bloque
int tBloque = 0.5;
int vBloque = 255;
// tiempo y velocidad para dar 180
int v180 = 200;
int t180 = 1;
// tiempo scan
int tScan = 10;



// LOW si no hay obstaculo, HIGH si lo hay
int distLeft = LOW;
int distRight = LOW;
int distCenter = LOW;

float useSensor(Types::UltraSonic us) {
  int trigger = us.trigger;
  int echo = us.echo;
  //aclaramos
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  // mandamos y leemos
  digitalWrite(trigger, HIGH);
  delayMicroseconds(tScan);
  digitalWrite(trigger, LOW);


  int duration = pulseIn(echo, HIGH);
  return duration * 0.0343 / 2;
}

void checkSurroundings() {
  // izquierda
  distLeft = useSensor(robot.leftUS);
  distRight = useSensor(robot.rightUS);
  distCenter = useSensor(robot.centerUS);

}


void setup() {
  // setup ines robot
  pinMode(CONSTANTS::ENA, OUTPUT);
  pinMode(CONSTANTS::IN1, OUTPUT);
  pinMode(CONSTANTS::IN2, OUTPUT);
  pinMode(CONSTANTS::ENB, OUTPUT);
  pinMode(CONSTANTS::IN3, OUTPUT);
  pinMode(CONSTANTS::IN4, OUTPUT);

  //setup ultrasonicos
  pinMode(robot.leftUS.echo, OUTPUT);
  pinMode(robot.leftUS.trigger, OUTPUT);

  pinMode(robot.centerUS.echo, OUTPUT);
  pinMode(robot.centerUS.trigger, OUTPUT);

  pinMode(robot.rightUS.echo, OUTPUT);
  pinMode(robot.rightUS.trigger, OUTPUT);

}

void loop() {
  // RIGHT HAND RULE
  
  
  // revisamos entorno
  checkSurroundings();

  // si tenemos pared a la derecha y libre frente, seguimos frente
  bool frente = distRight < 0.3 && distCenter > 0.5;
  // si no hay pared frente, pero no hay pared a la derecha, giramos hacia la derecha con la pared
  bool giraDerecha = distRight > 0.5 && distCenter > 0.5;
  // si hay pared enfrente y a la derecha, giramos en 180
  bool gira180 = distRight < 0.3 && distCenter < 0.3;
  
  if (frente) {
    MOVIMIENTO::moverFrente(robot,vBloque);
    delay(tBloque);
  } else if (giraDerecha) {
    //giramos a la derecha y damos recto
    //derecha
    MOVIMIENTO::giraDerecha(robot, vGiro);
    delay(tGiro);
    //recto
    MOVIMIENTO::moverFrente(robot, vBloque);
    delay(tBloque);
  } else if (gira180) {
    MOVIMIENTO::giraIzquierda(robot, v180);
    delay(t180);
  }

}