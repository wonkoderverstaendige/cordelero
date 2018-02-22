/*********************************************************************
This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

This example is for a 128x64 size display using I2C to communicate
3 pins are required to interface (2 I2C and one reset)

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AccelStepper.h>
#include <TimerOne.h>
#include <Encoder.h>

// OLED
#if (SSD1306_LCDHEIGHT != 64)
  #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
Adafruit_SSD1306 display(-1);

// Encoder
#define ENC_SW 2  // Interrupt
#define ENC_CLK 3 // Interrupt
#define ENC_DT 4  // Digital
Encoder encoder(ENC_CLK, ENC_DT);
volatile bool enc_switch = false;
long enc_position = 0;

// Stepper setup
#define In1 8
#define In2 7
#define In3 6
#define In4 5
#define TURNSTEPS 4076L // actual gearing of 28BYJ-48, not 4096
#define GEAR_RATIO 4L
#define STEP_ISR_INTERVAL 100
long stepperSpeed = 1750L;
long stepperAccel = 5000L;

long turns_forward = 80;
long turns_reverse = 40;
long turns_current = 0;

long steps_forward = (turns_forward * TURNSTEPS) / GEAR_RATIO; // small gear turns * steps/turn / gearing 
long steps_reverse = (turns_reverse * TURNSTEPS) / GEAR_RATIO;

enum MotorState {
  FAULT,
  HALT,
  DONE,
  FORWARD,
  REVERSE
};

MotorState motor_state = HALT;
AccelStepper stepper(AccelStepper::HALF4WIRE, In1, In3, In2, In4); // note order of pins! 1, 3, 2, 4


void setup()   {                
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)

  // Encoder
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_CLK, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_SW), toggle, FALLING);
  // Show image buffer on the display hardware.
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
  delay(500);
  
  display.setTextSize(2);
  display.setTextColor(WHITE);

  stepper.setMaxSpeed(stepperSpeed);
  stepper.setAcceleration(stepperAccel);
  Timer1.initialize(STEP_ISR_INTERVAL);
  Timer1.attachInterrupt(take_step);
}

void loop() {
    if (enc_switch) process_events();
    update_pos_display();

    // End of Motor travel
    if (stepper.distanceToGo() == 0) {
      if (motor_state == FORWARD) {
        reverse();
      } // or completely done
      else if (motor_state == REVERSE) {
        stop_turning();
      }
    }
}

void reverse() {
  motor_state = REVERSE;
  turns_current = turns_reverse;
  stepper.setCurrentPosition(0);
  update_pos_display();
  delay(500);
  stepper.move(-steps_reverse);
}

void stop_turning() {
  motor_state = DONE;
  stepper.stop();
  stepper.disableOutputs();
}

void start_turning() {
  motor_state = FORWARD;
  stepper.setCurrentPosition(0);
  stepper.enableOutputs();
  turns_current = turns_forward;
  stepper.move(steps_forward);
}

void halt() {
  motor_state = HALT;
}

void toggle() {
  enc_switch = true;
}

void take_step() {
  stepper.run();
}

void process_events() {
  if (enc_switch) {
    if (motor_state == DONE) {
      halt();
    } else if (motor_state == HALT) {
      start_turning();
    }
  }
}

void update_pos_display() {
    display.clearDisplay();
    display.setCursor(0,0);
    
    if (motor_state == FAULT) {
      display.print("ERROR!");
    }

    if (motor_state == HALT) {
      display.print("++ MENU ++");
      display.setCursor(0, 18);
      display.println(" PRESS TO   START ");

      display.setTextSize(1);
      display.setCursor(0, 52);
      display.print("Encoder: ");
      display.print(encoder.read() / 4);
      display.print(" T: ");
      if (enc_switch) {
        display.print("ON");
      } else {
        display.print("OFF");
      }

      display.setTextSize(2);
    }

    if (motor_state == DONE) {
      display.print("++ DONE ++");
      display.setCursor(0, 18);
      display.println(" Fire up     that    heatgun!");
    }

    if (motor_state >= FORWARD) {
      if (motor_state == FORWARD) {
        display.print("Forward >>");
      } else if (motor_state == REVERSE) {
        display.print("<< Reverse");
      }
  
      display.setCursor(0, 18);
      display.setTextSize(3);
      unsigned int pos = abs(ceil((float(stepper.distanceToGo())/TURNSTEPS)*GEAR_RATIO));
      display.print(turns_current - pos);
      display.print('/');
      display.println(turns_current);

      // DEBUG OUTPUT
      display.setTextSize(1);
      display.setCursor(0, 42);
      display.print("Steps left: ");
      display.println(stepper.distanceToGo());

      display.setCursor(0, 52);
      display.print("Encoder: ");
      display.print(encoder.read() / 4);
      display.print(" T: ");
      if (enc_switch) {
        display.print("ON");
      } else {
        display.print("OFF");
      }

      display.setTextSize(2);
    }
    display.display();
}

