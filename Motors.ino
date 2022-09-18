/*  Arduino DC Motor Control - PWM | H-Bridge | L298N
    Robotics Project
    by
    Samantha Casilla
    Daniel Dorantes
    Carlos Silva
    David Valerio
    
    based on Dejan Nedelkovski, www.HowToMechatronics.com
*/


#define enA 13
#define in1 12 
#define in2 11
#define enB 10
#define in3 9
#define in4 8
#define enC 7
#define in5 6
#define in6 5
#define enD 4
#define in7 3
#define in8 2
#define swj 49


int motorSpeedA = 0;
int motorSpeedB = 0;
int motorSpeedC = 0;
int motorSpeedD = 0;

// Declaracion de variables globales
float tempC; // Variable para almacenar el valor obtenido del sensor (0 a 1023)
int pinLM35 = 7; // Variable del pin de entrada del sensor (A0)
int led=48;
int counter=0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
  pinMode(swj, INPUT);

  Serial.begin(9600);
  pinMode(48,OUTPUT);
}

void loop() {


  int xAxis = analogRead(A0);  // Read Joysticks X-axis
  int yAxis = analogRead(A1);  // Read Joysticks Y-axis
  //int turn  = digitalRead(22); // Read Joystick push button

  // Y-axis used for forward and backward control
  if (yAxis < 470) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
   // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Set Motor C backward
    digitalWrite(in5, HIGH);
    digitalWrite(in6, LOW);
    // Set Motor D backward
    digitalWrite(in7, HIGH);
    digitalWrite(in8, LOW);

    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 470, 0, 0, 255);
    motorSpeedB = map(yAxis, 470, 0, 0, 255);
    motorSpeedC = map(yAxis, 470, 0, 0, 255);
    motorSpeedD = map(yAxis, 470, 0, 0, 255);
  }
  else if (yAxis > 550) {
    // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Set Motor C forward
    digitalWrite(in5, LOW);
    digitalWrite(in6, HIGH);
    // Set Motor D forward
    digitalWrite(in7, LOW);
    digitalWrite(in8, HIGH);
    
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 550, 1023, 0, 255);
    motorSpeedB = map(yAxis, 550, 1023, 0, 255);
    motorSpeedC = map(yAxis, 550, 1023, 0, 255);
    motorSpeedD = map(yAxis, 550, 1023, 0, 255);
  }

  else if (digitalRead(swj) ==true){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Set Motor C backward
    digitalWrite(in5, HIGH);
    digitalWrite(in6, LOW);
    // Set Motor D backward
    digitalWrite(in7, HIGH);
    digitalWrite(in8, LOW);
  
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = 255;
    motorSpeedB = 255;
    motorSpeedC = 255;
    motorSpeedD = 255;
    }

  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
    motorSpeedC = 0;
    motorSpeedD = 0;
    
  }

  // X-axis used for left and right control
  if (xAxis < 470) {
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
    int xMapped = map(xAxis, 470, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    motorSpeedC = motorSpeedC + xMapped;
    motorSpeedD = motorSpeedD + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
    if (motorSpeedC > 255) {
      motorSpeedC = 255;
    }
    if (motorSpeedD > 255) {
      motorSpeedD = 255;
    }
  }
  if (xAxis > 550) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    motorSpeedC = motorSpeedC - xMapped;
    motorSpeedD = motorSpeedD - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedC < 0) {
      motorSpeedC = 0;
    }
    if (motorSpeedD < 0) {
      motorSpeedD = 0;
    }
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
  }
  
  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }
  if (motorSpeedC < 70) {
    motorSpeedC = 0;
  }
  if (motorSpeedD < 70) {
    motorSpeedD = 0;
  }

  counter=counter+1;
  if(counter==10000){
  
    // Con analogRead leemos el sensor, recuerda que es un valor de 0 a 1023
  tempC = analogRead(pinLM35); 
   
  // Calculamos la temperatura con la fórmula
  tempC = (tempC*500.0)/1024.0; 

  if (tempC<25){
    digitalWrite(led,HIGH);
    }
  else{
     digitalWrite(led,LOW);
    }
    
  // Envia el dato al puerto serial
  Serial.print(tempC);
  // Salto de línea
  Serial.print("\n");
  counter=0;
  }
  // Esperamos un tiempo para repetir el loop
  
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
  analogWrite(enC, motorSpeedC); // Send PWM signal to motor C
  analogWrite(enD, motorSpeedD); // Send PWM signal to motor D
}