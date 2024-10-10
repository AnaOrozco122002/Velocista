
//Incluir Librerias
#include <QTRSensors16.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

//Bluetooth BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "ceb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_3 "deb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_4 "eeb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_5 "feb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;
BLECharacteristic *pCharacteristic_2;
BLECharacteristic *pCharacteristic_3;
BLECharacteristic *pCharacteristic_4;
BLECharacteristic *pCharacteristic_5;

//Modulo de Inicio
const byte MInit=D5;
int Estado;

//TURBINA
//Creación del Objeto 
Servo myTurbina;

//PIN PARA EL CONTROL DE TURBINA
const byte Tur=D6;

//Variables para sensores
#define NUM_SENSORS             16  // Numero de sensores usados
#define NUM_SAMPLES_PER_SENSOR  3  // Numero de muestras
#define IN_PIN             A2  // PIN de entrada del multiplexor


// Inicialización del sensor, digitales D9,D10,D0,D1
QTRSensorsMux qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, (unsigned char) IN_PIN, (unsigned char[]){D9,D10,D0,D1} );
unsigned int sensorValues[NUM_SENSORS];


//Variables para el controlador
float Tm = 9.0;                                            //tiempo de muestreo en mili segundos
float Referencia=0.0, Control=0.0, Kp = 5.3, Ti = 0.0, Td = 0.03; 
float Salida=0.0, Error=0.0, Error_ant=0.0;                       //variables de control
float offset = 1, Vmax = 570,E_integral;
char caracter; String datos;   //  sintonizacion bluetooth
int d1, d2, d3,d4;                //  sintonizacion bluetooth
String S_Kp, S_Ti, S_Td, S_Vmax;       //  sintonizacion bluetooth
unsigned long int Tinicio = 0;
bool conect=false;
//CREACIÓN DE PWM
const uint16_t Frecuencia = 5000;
const byte Canales[] ={0,1};
const byte Resolucion = 10;

const int  PWMD = D4;                                             // Definición Pin 6 PWM Motor Derecho
const int  PWMI = D8;  
const int  DirD = D3;
const int  DirI = D7;


class MyCallbacks_1: public BLECharacteristicCallbacks {

    void onConnect(BLEServer *pServer){
      conect=true;
    }
    void onDisconnect(BLEServer *pServer){
      conect=false;
    }

    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        
        Serial.println("*********");
        datos = datos + value.c_str();
        Serial.println("Datos: " + String(datos));
        if (value[0] == '*') {
          int d1 = datos.indexOf(',');
          String S_Kp = datos.substring(1, d1);
          int d2 = datos.indexOf(',', d1 + 1);
          String S_Ti = datos.substring(d1 + 1, d2);
          int d3 = datos.indexOf(',', d2 + 1);
          String S_Td = datos.substring(d2 + 1, d3);

          int d4 = datos.indexOf(',', d3 + 1);
          String S_Vmax = datos.substring(d3 + 1, d4);

          datos="";
          
          Kp = S_Kp.toFloat();  Ti = S_Ti.toFloat();   Td = S_Td.toFloat();  Vmax = S_Vmax.toFloat();   
          Serial.println("Skp: " + String(S_Kp) + " S_Ti: " + String(S_Ti) + " Td: " + String(S_Td) + " Vmax: " + String(S_Vmax)); 
          Serial.println("kp: " + String(Kp) + " Ti: " + String(Ti) + " Td: " + String(Td) + " Vmax: " + String(Vmax));
        }

      }

      

    }
};

class MyCallbacks_2: public BLECharacteristicCallbacks {
};

class MyCallbacks_3: public BLECharacteristicCallbacks {
};
class MyCallbacks_4: public BLECharacteristicCallbacks {
};
class MyCallbacks_5: public BLECharacteristicCallbacks {
};


void setup() {
  Serial.begin(9600);

  //Inicialización de Pines
  Inicializacion_Pines();

  delay(3000);


  //Inicialización de Sensores
  Inicializacion_Sensores();

  //Creación del PWM
  CrearPWM();

  //Inicializacion Bluetooth
  Inicializacion_Bluetooth();

  //Inicialización de la turbina
  Inicializacion_turbina();

  
}

void loop() {
  Estado=digitalRead(MInit);
  //Estado=1;
  while(Estado){
    Estado=digitalRead(MInit);
    Tinicio    = millis();                                        // toma el valor en milisengundos
    Salida     = Lectura_Sensor();                                // funcion de lectura de la variable salida del  proceso
    Control    = Controlador(Referencia,Salida);                  // funcion de la ley de control 
    Esfuerzo_Control(Control);                                    // funcion encargada de enviar el esfuerzo de control
    Tm = Tiempo_Muestreo(Tinicio); 
    myTurbina.write(50);
    EnviarDatos();
  }
  ledcWrite(Canales[0], 0);
  ledcWrite(Canales[1], 0);

  EnviarDatos();
  

}

//Para leer el sensor
float Lectura_Sensor(void) {                                             
  Salida = (qtra.readLine(sensorValues)/7500.0) - 1.0;
  //Serial.println(Salida);
  return Salida;                                               // retorno la variable de salidad del proceso normalizada entre 0-1, al olgoritmo de control
}

//Controlador para Motoresu
float Controlador(float Referencia, float Salida) {                           // Funcion para la ley de control
  float E_derivativo;
  float Control;

  Error_ant      = Error; 
  Error          = Referencia - Salida;
  Error = (Error > -0.1 && Error < 0) ? 0 : (Error > 0 && Error < 0.1) ? 0 : Error;
  E_integral     = E_integral + (((Error*(Tm/1000.0)) + ((Tm/1000.0)*(Error - Error_ant)))/2.0);
  E_integral     = ( E_integral > 100.0) ? 100.0 :  (E_integral < -100.0 ) ? -100 : E_integral;
  E_derivativo   = (Error - Error_ant)/(Tm/1000.0);
  Control        = Kp*( Error + Ti*E_integral + Td*E_derivativo );

  Control     = ( Control > 1.5) ? 1.5 :  (Control < -1.5 ) ? -1.5 : Control;
  //Serial.println(Control); 
  return Control;
}
void Esfuerzo_Control(float Control) {                            //envia el esfuerzo de control en forma de PWM
  float s1 , s2;

  s1  = (offset - Control);  
  s2  = (offset + Control);
  
  ledcWrite(Canales[0], floor(constrain(abs(s1), 0.0, 1.0)* Vmax));
  ledcWrite(Canales[1], floor(constrain(abs(s2), 0.0, 1.0)* Vmax));
  /*Serial.print("Derecha: ");
  Serial.print(floor(constrain(abs(s1), 0.0, 1.0)* Vmax));
  Serial.print("Izquierda: ");
  Serial.print(floor(constrain(abs(s2), 0.0, 1.0)* Vmax));*/

   if( s1 <= 0.0 ){// Motor Derecho
    digitalWrite(DirD,HIGH);}
  else{digitalWrite(DirD,LOW);}                   
   
  
  if( s2 <= 0.0 ){ //Motor Izquierdo
    digitalWrite(DirI,HIGH);}
  else{digitalWrite(DirI,LOW);}
} 

unsigned long int Tiempo_Muestreo(unsigned long int Tinicio){//, unsigned int Tm){ // Funcion que asegura que el tiempo de muestreo sea el mismo siempre
  unsigned long int T =millis()-Tinicio;
  return  T;
  }

void CrearPWM(){
  ledcSetup(Canales[0],Frecuencia,Resolucion); 
  ledcSetup(Canales[1],Frecuencia,Resolucion); 
  ledcAttachPin(PWMD,Canales[0]);
  ledcAttachPin(PWMI,Canales[1]);
}
void Inicializacion_turbina(){
  //Mensajes de Inicio
  Serial.println("-------------- Proceso de Calibracion de ESC --------------" );
  Serial.println("Iniciando ......");
  Serial.println("ATENCIÓN El motor Iniciara a Girar");
  ESP32PWM::allocateTimer(2);
  myTurbina.setPeriodHertz(50);              //frecuencia de la señal cuadrada
  myTurbina.attach(Tur, 1000, 2000);  //(pin,min us de pulso, máx us de pulso)
  myTurbina.write(0);                        //Preparación de la turbina
  delay(2000);
}
void Inicializacion_Sensores(){
  //Mensajes de Inicio
  Serial.println("-------------- Proceso de Calibracion de Los Sensores --------------" );
  Serial.println("Iniciando ......");
  //Calibración Inicial de Pines Sensor
  for (int i = 0; i < 100; i++){  // make the calibration take about 10 seconds
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  delay(2000);
}

void Inicializacion_Pines(){
  pinMode(PWMD,OUTPUT);
  pinMode(PWMI,OUTPUT);
  pinMode(D3,OUTPUT);
  pinMode(D8,OUTPUT);
  pinMode(MInit,INPUT);
}


void Inicializacion_Bluetooth(){
  BLEDevice::init("SOLLOW");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pCharacteristic_2 = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_2,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pCharacteristic_3 = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_3,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );
  pCharacteristic_4 = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_4,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );
  pCharacteristic_5 = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_5,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );
  pCharacteristic->setCallbacks(new MyCallbacks_1());
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Inicializacion Sollow");
  pCharacteristic_2->setCallbacks(new MyCallbacks_2());
  pCharacteristic_2->addDescriptor(new BLE2902());
  pCharacteristic_2->setValue("Caracteristica 2");
  pCharacteristic_3->setCallbacks(new MyCallbacks_3());
  pCharacteristic_3->addDescriptor(new BLE2902());
  pCharacteristic_3->setValue("Caracteristica 3");
  pCharacteristic_4->setCallbacks(new MyCallbacks_4());
  pCharacteristic_4->addDescriptor(new BLE2902());
  pCharacteristic_4->setValue("Caracteristica 4");
  pCharacteristic_5->setCallbacks(new MyCallbacks_5());
  pCharacteristic_5->addDescriptor(new BLE2902());
  pCharacteristic_5->setValue("Caracteristica 5");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void EnviarDatos(){
  char kpcs[8];
  dtostrf(Kp,1,2,kpcs);
  char tics[8];
  dtostrf(Ti,1,2,tics);
  char tdcs[8];
  dtostrf(Td,1,2,tdcs);
  char vmaxcs[8];
  dtostrf(Vmax,1,2,vmaxcs);
  pCharacteristic->setValue("Lectura Correcta");
  pCharacteristic->notify();
  pCharacteristic_2->setValue(kpcs);
  pCharacteristic_2->notify();
  pCharacteristic_3->setValue(tics);
  pCharacteristic_3->notify();
  pCharacteristic_4->setValue(tdcs);
  pCharacteristic_4->notify();
  pCharacteristic_5->setValue(vmaxcs);
  pCharacteristic_5->notify();
}
