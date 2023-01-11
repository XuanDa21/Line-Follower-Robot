#include <QTRSensors.h>

#define btn1      0
#define btn2      1
#define led1      13

#define led_on    9   //~
#define mi1       3
#define mi2       4
#define pwmi      5   //~
#define md1       8
#define md2       7
#define pwmd      6   //~
  

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  1 // average 4 analog samples per sensor reading
#define EMITTER_PIN             9   // emitter is controlled by digital pin 2

QTRSensors qtra;
uint16_t sensorValues[NUM_SENSORS];
int proporcional=0;
int derivativo=0;
int integral=0;
int salida_pwm=0;
int proporcional_pasado=0;
int16_t  position;

////////////////////Parametros PID/////////////////////////////////
// Khai báo tốc độ ban đầu
int velocidad=170;
// chọn 1 trong 3 bộ pid với mỗi bộ sẽ cho 1 chất lượng điều khiển khác nhau
//float KP=0.18, KD=4, KI=0.01; 
float KP=0.01, KD=0.15 , KI=0.001; 
//float KP=0.2, KD=0.0435 , KI=0.00018; 
///////////////////////////////////////////////////////

/////////////////parametros de sensado/////////////////////////////
int linea= 0; //0 linea negra, 1 para linea blanca
int flanco_color=  0 ;
int en_linea=  500 ;
int ruido= 30;
/////////////////////////////////////////////////////////
int boton1=7;
int boton2=7;

void setup()
{ 
    // Tùy loại cảm biến sử dụng thì khai báo theo loại cảm biến đó
    qtra.setTypeAnalog();
    qtra.setSensorPins((const uint8_t[]){A7,A6,A5,A4,A3,A2,A1,A0}, NUM_SENSORS);
    qtra.setEmitterPin(9);
    pinMode(led1,OUTPUT);
    pinMode(led_on,OUTPUT);
    pinMode(mi1,OUTPUT);
    pinMode(mi2,OUTPUT);
    pinMode(pwmi,OUTPUT);
    pinMode(md1,OUTPUT);
    pinMode(md2,OUTPUT);
    pinMode(pwmd,OUTPUT);

digitalWrite(led1,HIGH);
delay(200);
digitalWrite(led1,LOW);
delay(200);

digitalWrite(led1, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(led1, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  Serial.begin(115200);
   //Serial.println();

   while(true)
   {
      botones();
      if(boton2==0) 
      {
        delay(20);
        digitalWrite(led1,HIGH);
        delay(100);
        digitalWrite(led1,LOW);
        delay(100);
        break;
      }
   }
  
//Serial.begin(115200);
//bt.println("Hello, world?");

}

void loop()
{
  pid(1,velocidad,KP,KI,KD);
  frenos_contorno(600);

 Serial.println(position);
 delay(2);
}

void pid(int linea, int velocidad, float Kp, float Ki, float Kd)
{
   position = qtra.readLineWhite(sensorValues);
  
  // Serial.println(position);
  proporcional =  3500 - position; // set point es 3500, asi obtenemos el error
  integral=integral + proporcional_pasado; //obteniendo integral
  derivativo = (proporcional - proporcional_pasado); //obteniedo el derivativo
  int ITerm=integral*KI;
  if(ITerm>=255) ITerm=255;
  if(ITerm<=-255) ITerm=-255;
  salida_pwm =( proporcional * KP ) + ( derivativo * KD )+(ITerm);
  
  if (  salida_pwm >velocidad )  salida_pwm = velocidad; //limitamos la salida de pwm
  if ( salida_pwm  <-velocidad )  salida_pwm = -velocidad;
  
  if (salida_pwm < 0)
 {
    int der=velocidad-salida_pwm; //(+)
    int izq=velocidad+salida_pwm;  //(-)
    if(der>=255)der=255;
    if(izq<=0)izq=0;
    motores(izq, der);
 }
 if (salida_pwm >0)
 {
  int der=velocidad-salida_pwm; //(-)
  int izq=velocidad+salida_pwm; //(+)
  
  if(izq >= 255) izq=255;
  if(der <= 0) der=0;
  motores(izq ,der );
 }

 proporcional_pasado = proporcional;  
}


void frenos_contorno(int flanco_comparacion)
{
    if (position >=6990 ) // NEU XE LECH PHAI
    {
      while(true)
      { 
        digitalWrite(led1,HIGH);
        motores(60,-125);
        qtra.read(sensorValues); //lectura en bruto de sensor
        if ( sensorValues[0]<flanco_comparacion || sensorValues[1]<flanco_comparacion || sensorValues[2]<flanco_comparacion 
        || sensorValues[3]<flanco_comparacion || sensorValues[4]<flanco_comparacion || sensorValues[5]<flanco_comparacion 
        || sensorValues[6]<flanco_comparacion || sensorValues[7]<flanco_comparacion)
        {
          break;
        }
        
      }
    }

    if (position<=10) // NEU XE LECH TRAI
    {
      while(true)
      {
        digitalWrite(led1,HIGH);
        motores(-125,60 );
        qtra.read(sensorValues);
        if (sensorValues[7]<flanco_comparacion || sensorValues[6]<flanco_comparacion|| 
        sensorValues[5]<flanco_comparacion || sensorValues[4]<flanco_comparacion || sensorValues[3]<flanco_comparacion 
        || sensorValues[2]<flanco_comparacion || sensorValues[1]<flanco_comparacion|| sensorValues[0]<flanco_comparacion)
        {
          break;
        }
      }
  }
  digitalWrite(led1,LOW);
}

void motores(int motor_izq, int motor_der)
{
  if ( motor_izq >= 0 )  
  {
    digitalWrite(mi1,LOW);
    digitalWrite(mi2,HIGH); 
    analogWrite(pwmi,motor_izq); 
  }
  else
  {
    digitalWrite(mi1,HIGH); 
    digitalWrite(mi2,LOW);
    motor_izq = motor_izq*(-1); 
    analogWrite(pwmi,motor_izq);
  }

  if ( motor_der >= 0 ) 
  {
    digitalWrite(md1,LOW);
    digitalWrite(md2,HIGH);
    analogWrite(pwmd,motor_der);
  }
  else
  {
    digitalWrite(md1,HIGH);
    digitalWrite(md2,LOW);
    motor_der= motor_der*(-1);
    analogWrite(pwmd,motor_der);
  }
}

void botones()
{
    boton1=digitalRead(btn2);
    boton2=digitalRead(btn1); 
}
