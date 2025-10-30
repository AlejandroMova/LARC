#include <funciones.h>
#include <types.h>
#include <constants.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h> 

// utilizamos right hand rule

// para este reto, necesitamos adafruit TCS34725 y adafruit MPU6050

// inicializar robot
Types::Robot robot = INIT::init_robot();

// crear objeto mpu 6050
Adafruit_MPU6050 mpu;

// inicializar deteccion de colores 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// enum para colores
enum COLORES {
  VERDE,
  AMARILLO,
  ROJO,
  AZUL,
  DESCONOCIDO

};

// funcion para detectar colores 
// anadimos & para pasar por referencia, mas rapido
COLORES detectColor(Adafruit_TCS34725 &tcs) {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Evitar división por cero
  float sum = r + g + b;
  if (sum == 0) {
    return DESCONOCIDO; // si no hay luz, no dividimos
  }

  float R = r / sum;
  float G = g / sum;
  float B = b / sum;

  // normalizamos valores basado en porcentaje
  // estos valores pueden necesitar ajuste
  if (R > 0.38 && G > 0.35 && B < 0.25) return AMARILLO;
  if (G > R + 0.05 && G > B + 0.05) return VERDE;
  if (B > R + 0.05 && B > G + 0.05) return AZUL;
  if (R > 0.35 && B > 0.35 && G < 0.30) return ROJO; // esto parece mas magenta que rojo

  return DESCONOCIDO;
}

// nueva funcion para convertir enum a texto
const char* colorToString(COLORES color) {
  switch (color) {
    case VERDE:    return "Verde";
    case AMARILLO: return "Amarillo";
    case ROJO:     return "Rojo";
    case AZUL:     return "Azul";
    default:       return "???";
  }
}

// nueva funcion para escanear y mostrar
void escanearYMostrarColor() {
  COLORES colorDetectado = detectColor(tcs);
  const char* textoColor = colorToString(colorDetectado);

  // escribimos el color en la segunda fila
  PANTALLA::write(0, 1, "Color:          "); // limpia la fila
  PANTALLA::write(7, 1, textoColor);
  
  delay(500); // pausa chica para leer
}


// hace falta revisar tiempos y velocidad
// tiempo y velocidad para girar
int vGiro = 200;
// tiempo y velocidad para cambiar de bloque
int tBloque = 500;
int vBloque = 255;
// tiempo y velocidad para dar 180
int v180 = 200;
int t180 = 1000;
// tiempo scan
int tScan = 10;

// distancia en metros
int distLeft = 0;
int distRight = 0;
int distCenter = 0;

float useSensor(Types::UltraSonic us) {
  // devuelve cms
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

void giroGrad(char direccion, int grados) {
  // inicializamos variables
  float angulo_girado = 0;

  if (direccion == 'D') {
    //girar hacia la derecha
    MOVIMIENTO::giraDerecha(robot, vGiro);
  } else if (direccion == 'I') {
    MOVIMIENTO::giraIzquierda(robot, vGiro);
  }
  unsigned long tiempoAnterior = micros();
  // girar hasta que el angulo de giro sea 90
  while (abs(angulo_girado) < grados) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // velocidad de giro en grados
    float velocidad_giro = g.gyro.z * (180 / PI);

    unsigned long tiempoActual = micros();
    // diferencia de tiempo desde que empezo el loop hasta el final
    long dif_tiempo = tiempoActual - tiempoAnterior;
    // pasamos de microsegundos a segundos
    float dif_tiempo_seg = dif_tiempo / 1000000.0;
    // velocidad en angulos por segundo * el tiempo  en segundos
    angulo_girado += velocidad_giro * dif_tiempo_seg;
    tiempoAnterior = tiempoActual; // corregido, estaba declarando una nueva variable
  }
  MOVIMIENTO::frenar(robot);
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
  // echo es input, trig es output
  pinMode(robot.leftUS.echo, INPUT); // corregido
  pinMode(robot.leftUS.trigger, OUTPUT);

  pinMode(robot.centerUS.echo, INPUT); // corregido
  pinMode(robot.centerUS.trigger, OUTPUT);

  pinMode(robot.rightUS.echo, INPUT); // corregido
  pinMode(robot.rightUS.trigger, OUTPUT);


  // init pantalla
  PANTALLA::init(16, 2);
  PANTALLA::write(0, 0, "Iniciando...");

  //init giroscopio
  if (!mpu.begin()) {
    Serial.println("No se encontro MPU 6050");
    PANTALLA::write(0, 1, "Error MPU");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 encontrado");

  // init sensor de color
  if (tcs.begin()) {
    Serial.println("Sensor TCS34725 encontrado");
  } else {
    Serial.println("No se encontro TCS34725");
    PANTALLA::write(0, 1, "Error TCS");
    while (1); // se congela si no lo encuentra
  }

  PANTALLA::write(0, 0, "Robot Listo");
  delay(500);
}

void loop() {
  // variables del giroscopio
  sensors_event_t a, g, temp;

  // esta variable sera utilizada para medir el giro
  float angulo_girar = 0;


  // RIGHT HAND RULE
  // revisamos entorno
  checkSurroundings();

  // mostramos distancia en pantalla
  String distStr = "C:" + String(distCenter) + " D:" + String(distRight);
  PANTALLA::write(0, 0, "                "); // limpia fila
  PANTALLA::write(0, 0, distStr.c_str());

  // si tenemos pared a la derecha y libre frente, seguimos frente
  bool frente = distRight < 30 && distCenter > 50;
  // si no hay pared frente, pero no hay pared a la derecha, giramos hacia la derecha con la pared
  bool giraDerecha = distRight > 50 && distCenter > 50;
  // si hay pared enfrente y a la derecha, giramos en 180
  bool gira180 = distRight < 30 && distCenter < 30;

  if (frente) {
    // CAMBIAR PARA ENCODERS
    //diametro 6.6 cm 
    // gear ratio 1:19.2
    // 11 ppr (pulsos por rev)
    // entonces 19.2 * 11 dan igual a 211 pulsos por vuelta de la rueda
    // circumferencia es igual a 6.6 * 3.14 = 20.7 cm por vuelta
    // por lo tanto, para calcular el movimiento de 15cm, 
    // 15 / 20.7 = 0.74 de la vuelta 
    // por lo tanto 211 * 0.74 = 156 pulsos para llegar a 15 cm 
    // MOVIMIENTO::moverFrente(robot, vBloque);
    MOVIMIENTO::avanzarDistancia(robot, 180, 156);
    delay(tBloque);
    // movimos un bloque, revisamos color
    escanearYMostrarColor();
  } else if (giraDerecha) {
    //giramos a la derecha y damos recto
    //derecha
    giroGrad('D', 90);
    //recto
    // CAMBIAR PARA ENCODERS
    //MOVIMIENTO::moverFrente(robot, vBloque);
    MOVIMIENTO::avanzarDistancia(robot, 180, 156);

    delay(tBloque);
    // movimos un bloque, revisamos color
    escanearYMostrarColor();
  } else if (gira180) {
    giroGrad('I', 180);
    // no nos movimos de bloque, solo giramos
    // asi que no escaneamos color
  } else {
    // caso por defecto, si nos atoramos
    MOVIMIENTO::frenar(robot);
  }
}
