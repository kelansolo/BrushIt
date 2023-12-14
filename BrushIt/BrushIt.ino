#include <Servo.h>
#include "src/TMC26XStepper.h"
#include "src/config.h"
#include "src/init.h"

enum State {WAIT, RUN, STOP};
State g_state = WAIT;
int run_state = 0;
bool e_stop = 0;
bool is_brushed = false;



void setup() {
  Serial.begin(9600);

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
  MPServo.writeMicroseconds(servo_180);

  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(VIBRATOR_MOTOR_PIN, OUTPUT);
  pinMode(TOOTHPASTER_MOTOR_PIN, OUTPUT);

  pinMode(ledPin, OUTPUT);

  pinMode(interruptPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), Interrupt1, CHANGE);

  pinMode(button1, INPUT_PULLUP);
  pinMode(end_stop, INPUT_PULLUP);
  pinMode(mouth_sensor, INPUT_PULLUP);

}

void loop() {

  if (e_stop == 0){
    switch(g_state) 
    {
      case WAIT:  
        if (digitalRead(button1) == CLICKED) 
        {
          g_state = RUN;
        }
        break;
      case RUN:
        switch(run_state)
        {
          case 0: // Home mouthpiece
            is_brushed = false;
            home(); // get 0
            run_state = (MPServo.readMicroseconds() != servo_0) ? 1 : 10;
            break;
          case 1: // Go to flipping height
            move_stepper(min_flip_height);
            updateSerial("@ flip height");
            delay(1000);
            run_state = 2;
          case 2:
            if (tmc26XStepper.isMoving() == 0) run_state = 3;
            break;
          case 3:  // Flip
            MPServo.writeMicroseconds(servo_0);
            run_state = 4;
          case 4:
            updateSerial("MP Flipped");
            if (MPServo.readMicroseconds() == servo_0) run_state = 0;
            break;
 
          case 10: // Apply toothpaste
            digitalWrite(TOOTHPASTER_MOTOR_PIN, HIGH);
            delay(tp_extrude_time);
            digitalWrite(TOOTHPASTER_MOTOR_PIN, LOW);
            run_state = 20;
            delay(200);
            updateSerial("Toothpaste applied");
            break;
          
          case 20: // Go to user height
            move_stepper(user_height);
            updateSerial("@ User Height");
            run_state = 21;
            updateSerial("Waiting for mouth sensor...");
            
          case 21:
            if (digitalRead(mouth_sensor) == CLICKED ) run_state = 30;
            break;

          case 30: // Vibrate
            digitalWrite(VIBRATOR_MOTOR_PIN, HIGH);
            delay(vibration_time);
            digitalWrite(VIBRATOR_MOTOR_PIN, LOW);
            run_state = 31;
            updateSerial("Waiting for mouth sensor...");
          case 31:
            if (digitalRead(mouth_sensor) == not CLICKED)
            {
              if(not is_brushed) 
              {
              is_brushed = true;
              run_state = 40;
              }
              else run_state = 50;
            }
            break;

          case 40: // Flip toothbrush
            MPServo.writeMicroseconds(servo_180);
            delay(2000);
            run_state = 21;
            break;

          case 50: // Home mouthpiece to be washed
            home();
            run_state = 60;
            break;
          
          case 60: // rinse brush
            digitalWrite(SOLENOID_PIN, HIGH);
            digitalWrite(VIBRATOR_MOTOR_PIN, HIGH);
            delay(wash_time);
            digitalWrite(SOLENOID_PIN, LOW);
            digitalWrite(VIBRATOR_MOTOR_PIN, LOW);
            run_state = 70;
            break;
          
          case 70: // rise brush to dry
            move_stepper(min_flip_height);
            run_state = 98;
            break;

          case 98: // Stop case
            run_state = 0;
            g_state = WAIT;
            break;

          default:
            run_state = 0;
            g_state = STOP;
        }
        break;

      case STOP:
        delay(200);
        g_state = WAIT;

        break;
    }
  }
}

void move_stepper(unsigned int tot_step)
{
  unsigned int part_step = 100;
  if(e_stop == 0)
  {
    while(tot_step - part_step > 0)
    {
      tmc26XStepper.SPI_setSpeed(FAST); 
      tmc26XStepper.SPI_step(part_step);
      tmc26XStepper.spi_start() ;
      tot_step = tot_step - part_step;
    }
    tmc26XStepper.SPI_setSpeed(FAST); 
    tmc26XStepper.SPI_step(tot_step); 
    tmc26XStepper.spi_start() ;
  }
}

void home()
{
    while(digitalRead(end_stop) == not CLICKED and e_stop == 0)
    {
      tmc26XStepper.SPI_setSpeed(FAST); 
      tmc26XStepper.SPI_step(-100);
      tmc26XStepper.spi_start() ;
    }
    if(e_stop == 0){
      delay(200);
      tmc26XStepper.SPI_setSpeed(FAST); 
      tmc26XStepper.SPI_step(100);
      tmc26XStepper.spi_start() ;
    }
    updateSerial("@ Home");
}

void updateSerial()
{
  Serial.print("g_state: ");
  Serial.print(g_state);
  Serial.print("    |    ");
  Serial.print("run_state: ");
  Serial.println(run_state);
}


void updateSerial(String message)
{
  Serial.print("g_state: ");
  Serial.print(g_state);
  Serial.print("    |    ");
  Serial.print("run_state: ");
  Serial.print(run_state);
  Serial.print("    |    ");
  Serial.println(message);
}

void Interrupt1() {
  static unsigned long last_interrupt_time1 = 0;
  unsigned long interrupt_time1 = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time1 - last_interrupt_time1 > 200) 
  {
    e_stop = !e_stop;
    if (e_stop == 1){
      g_state = STOP;
    }
  }
  last_interrupt_time1 = interrupt_time1;
}


