/*********************************************************************
CORDELERO - Twisting Tetrodes Like Its 1853!
*********************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AccelStepper.h>
#include <TimerOne.h>
#include <Encoder.h>

#define DEBUG
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

// start conditions
uint16_t turns_forward = 20;
uint16_t turns_reverse = 11;
uint16_t turns_current = 0;

long steps_forward; // small gear turns * steps/turn / gearing 
long steps_reverse;

enum MotorState {
  FAULT,
  MENU,
  DONE,
  FORWARD,
  REVERSE
};

MotorState motor_state = MENU;
AccelStepper stepper(AccelStepper::HALF4WIRE, In1, In3, In2, In4); // note order of pins! 1, 3, 2, 4

// MENU ITEMS
byte menu_selection = 0;
byte menu_num_items = 3;
bool menu_editing = false;


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
    process_events();
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
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);
  update_pos_display();
  delay(300);
  
  steps_reverse = turns_to_steps(turns_reverse);
  motor_state = REVERSE;
  turns_current = turns_reverse;
  stepper.move(-steps_reverse);
}

void stop_turning() {
  motor_state = DONE;
  stepper.stop();
  stepper.disableOutputs();
}

void start_turning() {
  steps_forward = turns_to_steps(turns_forward);
  motor_state = FORWARD;
  stepper.setCurrentPosition(0);
  stepper.enableOutputs();
  turns_current = turns_forward;
  stepper.move(steps_forward);
}

void halt() {
  motor_state = MENU;
  menu_selection = 0;
}

void toggle() {
  enc_switch = true;
}

void take_step() {
  stepper.run();
}

void process_events() {
  if (enc_switch) {
    switch (motor_state) {
      case DONE:
        halt();
        break;
      case MENU:
        if (menu_selection == 0) {
          start_turning();
        } else {
          menu_editing = !menu_editing;
          if (menu_editing) {
            encoder.write(0);
          } else {
            encoder.write(menu_selection * 4);
          }
        }
        break;
    }
    enc_switch = false;
    delay(100); // "debounce"
  }

  if (motor_state == MENU) {
    if (!menu_editing) {
      // TODO: Negative numbers... ugh.
      menu_selection = encoder.read() / 4 % menu_num_items;
    } else {
      switch (menu_selection) {
        case 1:
          turns_forward += encoder.read() / 4;
          if (turns_forward < 1) turns_forward = 1;
          break;
        case 2:
          turns_reverse += encoder.read() / 4;
          if (turns_reverse < 1) turns_reverse = 1;
          break;
      }
      if (encoder.read() / 4) {
        encoder.write(0);
      }
    }
  }
}

void update_pos_display() {
    display.clearDisplay();
    display.setCursor(0,0);
    
    if (motor_state == FAULT) {
      display.print("ERROR!");
    }

    if (motor_state == MENU) {
      display.println("++ MENU ++");
      display.setCursor(0, 18);
      display.println("  Start "); //display.println(menu_selection);
      
      display.print("  Fwd: ");
      if (menu_editing && menu_selection == 1){
        display.setTextColor(BLACK, WHITE);
        display.println(turns_forward);
        display.setTextColor(WHITE);
      } else {
        display.println(turns_forward);
      }


      display.print("  Rev: ");
      if (menu_editing && menu_selection == 2) {
        display.setTextColor(BLACK, WHITE);
        display.println(turns_reverse);
        display.setTextColor(WHITE);
      } else {
        display.println(turns_reverse);
      }
      
      // cursor
      display.setCursor(0, 18 + menu_selection * 16);
      display.print(">");
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
#if defined DEBUG
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
#endif
    }
    display.display();
}

long turns_to_steps(uint16_t turns) {
  return long(turns) * TURNSTEPS / GEAR_RATIO;
}

