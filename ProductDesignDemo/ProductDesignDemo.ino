#include <SPI.h>
#include <TMC26XStepper.h>
#include <Servo.h>

#define STEPPER_STEPS             200 // Full Turn
#define STEPPER_CURRENT           50 // [mA]

#define TMC2660_RESISTOR          10 // [ohm]
#define TMC2660_CS_PIN            6
#define TMC2660_EN_PIN            33 ///////////////////////////////// valeur de merde
#define TMC2660_STEP_PIN          5
#define TMC2660_DIR_PIN           4

#define SERVO_PIN                 7
#define SOLENOID_PIN              8
#define VIBRATOR_MOTOR_PIN        9
#define TOOTHPASTER_MOTOR_PIN     10

const byte ledPin = 13;
const byte interruptPin1 = 2;
const byte interruptPin2 = 3;
const byte interruptPin3 = 18;
const byte interruptPin4 = 19;

volatile byte state = LOW;

Servo MPServo;

TMC26XStepper tmc26XStepper = TMC26XStepper(STEPPER_STEPS, TMC2660_CS_PIN,TMC2660_DIR_PIN
                                           ,TMC2660_STEP_PIN,STEPPER_CURRENT, TMC2660_RESISTOR);

void setup() {
  Serial.begin(9600);
  Serial.println("=============================");
  Serial.println("        Brush'it Demo        ");
  Serial.println("=============================");

  Serial.println("Configuring TMC2660");
  pinMode(TMC2660_EN_PIN, OUTPUT);
  //char constant_off_time, char blank_time, char hysteresis_start, char hysteresis_end, char hysteresis_decrement
  tmc26XStepper.setSpreadCycleChopper(2,24,8,6,0);
  tmc26XStepper.setRandomOffTime(0);
  tmc26XStepper.SPI_setCoilCurrent(25);
  tmc26XStepper.setMicrosteps(128);
  tmc26XStepper.setStallGuardThreshold(4,0);
  Serial.println("config finished");

  MPServo.attach(SERVO_PIN);

  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(VIBRATOR_MOTOR_PIN, OUTPUT);
  pinMode(TOOTHPASTER_MOTOR_PIN, OUTPUT);

  pinMode(ledPin, OUTPUT);

  pinMode(interruptPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), Interrupt1, CHANGE);

  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), Interrupt2, CHANGE);

  pinMode(interruptPin3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin3), Interrupt3, CHANGE);

  pinMode(interruptPin4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin4), Interrupt4, CHANGE);
}

void loop() {
  if(state) {
    digitalWrite(TMC2660_EN_PIN, LOW);
    delay(100);
    Serial.println("Testing Stepper");
    tmc26XStepper.SPI_setSpeed(250);    //Set speed at 80 RPM
    tmc26XStepper.SPI_step(5000);       //set step at -200 steps,  that is to say stepper will turn a circle reverse
    tmc26XStepper.spi_start() ;         //start stepper
    delay(2000);
    tmc26XStepper.SPI_setSpeed(250);    //Set speed at 80 RPM
    tmc26XStepper.SPI_step(-5000);       //set step at -200 steps,  that is to say stepper will turn a circle reverse
    tmc26XStepper.spi_start() ;         //start stepper
    delay(2000);
  }
  else{
    digitalWrite(TMC2660_EN_PIN, HIGH);
  }

  Serial.println("Testing Servo");
  MPServo.writeMicroseconds(2230);
  delay(2000);
  MPServo.writeMicroseconds(1030);
  delay(2000);

  Serial.println("Testing Solenoid");
  for (int i = 0; i < 10; i++) 
  {
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(200);
    digitalWrite(SOLENOID_PIN, LOW);
    delay(200);
  }
  delay(2000);

  Serial.println("Testing Vibrator");
  digitalWrite(VIBRATOR_MOTOR_PIN, HIGH);
  delay(2000);
  digitalWrite(VIBRATOR_MOTOR_PIN, LOW);
  delay(2000);

  Serial.println("Testing Toothpaste Motor");
  digitalWrite(TOOTHPASTER_MOTOR_PIN, HIGH);
  delay(2000);
  digitalWrite(TOOTHPASTER_MOTOR_PIN, LOW);
  delay(2000);
}

void Interrupt1() {
  static unsigned long last_interrupt_time1 = 0;
  unsigned long interrupt_time1 = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time1 - last_interrupt_time1 > 200) 
  {
    state = !state;
    digitalWrite(ledPin, state);
  }
  last_interrupt_time1 = interrupt_time1;
}

void Interrupt2() {
  static unsigned long last_interrupt_time2 = 0;
  unsigned long interrupt_time2 = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time2 - last_interrupt_time2 > 200) 
  {
    state = !state;
    digitalWrite(ledPin, state);
  }
  last_interrupt_time2 = interrupt_time2;
}

void Interrupt3() {
  static unsigned long last_interrupt_time3 = 0;
  unsigned long interrupt_time3 = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time3 - last_interrupt_time3 > 200) 
  {
    state = !state;
    digitalWrite(ledPin, state);
  }
  last_interrupt_time3 = interrupt_time3;
}

void Interrupt4() {
  static unsigned long last_interrupt_time4 = 0;
  unsigned long interrupt_time4 = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time4 - last_interrupt_time4 > 200) 
  {
    state = !state;
    digitalWrite(ledPin, state);
  }
  last_interrupt_time4 = interrupt_time4;
}