#ifndef INIT_H
#define INIT_H

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

const bool CLICKED = 0;
const unsigned int FAST =250;
const unsigned int SLOW =100;
const byte ledPin = 13;
const byte interruptPin1 = 2;
const byte button1 = 3;
const byte end_stop = 18;
const byte mouth_sensor = 19;
// const byte interruptPin2 = 3;
// const byte interruptPin3 = 18;
// const byte interruptPin4 = 19;

volatile byte state = LOW;

Servo MPServo;

TMC26XStepper tmc26XStepper = TMC26XStepper(STEPPER_STEPS, TMC2660_CS_PIN,TMC2660_DIR_PIN
                                           ,TMC2660_STEP_PIN,STEPPER_CURRENT, TMC2660_RESISTOR);

#endif