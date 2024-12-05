/* 
  Author(s): Tyerone Chen
    Innit Create: 6/30/2024
      Last update: 9/26/2024
*/

// ** REMINDER ** //
// Remove a bunch of the serial prints or whatever so we can burn the code onto the arduino.

// Arduino Float Code Remake
// Credits for Danny henningfield for the innit steps/state change which stopped my from
// having a god damn aneurism lmao
/// Side Note, we need to comment the crap out of this becuase i had an 
/// aneurism reading the old code （´∇｀''）

// Library included
#include <SPI.h>
#include <RH_RF95.h>
#include <ezButton.h>
#include <List.hpp>



// Innit deffinition and crap
//Radio crap
#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#endif

// button defin
ezButton topSwitch(12); // Top Siwtch Connected to pin 12
ezButton bottomSwitch(A3); // Bottom switch connected to pin A3

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// list defin area - Type: float, int
List<float> psiList;
List<float> depthList;
List<int> timeList;

// Data recived innit
/// essentialy this is where the cariable of the recieved data, from the pi, will be held
/// should always be a string, which is formated as a list of chars
char received_data[RH_RF95_MAX_MESSAGE_LEN];


// multithreading / timer code setup
/// reminder* are all in terms of millis, so 1000 millis = 1 sec
// Type - unsigned long, const long
// All "const long" are basically the interval
/// for any newbies, const means it will never change, and long is simmilar to the double time, except it doesnt do decimal
/// unsigned is essentially a value type with only stores in positive integers for memory saving, which works here cuz we only count up

unsigned long radio_task_millis = 0;
const long radio_task_interval = 1001;

unsigned long psi_task_half_millis = 0;
const long psi_task_half_interval = 1001;

unsigned long psi_task_full_millis = 0;
const long psi_task_full_interval = 1250;

unsigned long psi_change_check_millis = 0;
const long psi_change_check_interval = 500;

unsigned long list_updater_millis = 0;
const long list_updater_interval = 5000;

// psi calc vars setup
float psi_half_sec = 0;
float psi_full_sec = 0;
float psi_calc = 0;

// port setup - Type: int
int voltA = 5;
int diag_port_A = 6;

int voltB = 11;
int diag_port_B = 10;

int pwm_port = 9;
int duty_cycle = 0;

// Switch Check - Type: Bool
bool top_switch_pressed = false;
bool bottom_switch_pressed = false;

// Float surface to ground check - Type: Bool
/// will be set as true at the beggining of code, as the
/// float should always start surfaced
bool float_surfaced = true; 
bool float_floored = false;


void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  topSwitch.setDebounceTime(0);  // Reduce debounce time
  bottomSwitch.setDebounceTime(0);  // Reduce debounce time

  // Innitiates how each pin on the board should work
  pinMode(voltA, OUTPUT);
  pinMode(voltB, OUTPUT);
  pinMode(diag_port_A, INPUT_PULLUP);
  pinMode(diag_port_B, INPUT_PULLUP);
  pinMode(pwm_port, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  // Enables the Diag Ports
  digitalWrite(diag_port_A, HIGH);
  digitalWrite(diag_port_B, HIGH);

  // Resistor setting for the Limit Swtches
  pinmode(12, INPUT_PULLUP);
  pinmode(A#, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, LOW);

  // Serial.println("Feather LoRa TX Test!");

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    //Serial.println("LoRa radio init failed");
    //Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  //Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    //Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);

  // Attach interrupt handler for top switch
  attachInterrupt(digitalPinToInterrupt(12), topSwitchDetect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A3), bottomSwitchDetect, CHANGE);
}

// this is important, why i dont remeber honestly
// nevermin i remeber its mainly just to innit and define it 
int16_t packetnum = 0;

void loop() {
  // Millis Timer Start
  unsigned long current_millis = millis();

  /// pressure calc
  // makes sure that the pressure pin is set to A1
  int pressure_pin = analogRead(A1);
  float psi = (0.0374 * pressure_pin) - 3.3308;

  // switch loop function, refer to function for more info

  // Motor power and port setup
  analogWrite(pwm_port, duty_cycle);
  duty_cycle = 255;

  // Radio Communication Checker
  if (current_millis - radio_task_millis >= radio_task_interval){
    radio_task_millis = current_millis;

    if (rf95.waitAvailableTimeout(1000)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) {
        /*Serial.print("Got reply: ");
        Serial.println((char*)buf); // This is the reply
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
*/
        // Store the received data in the global variable
        strncpy(received_data, (char*)buf, len);
        //Serial.println(received_data);
      } 
      else {
        //Serial.println("Receive failed");
      }
    } 
    else {
      // Serial.println("No reply, is there a listener around?");
    }
  }
  // End of Radio Communication Checker

  ////// Code which actualy starts/functions after passing the radio communcation check
  if (strcmp(received_data, "initiate") == 0){
    ////// Motor Movement Determiner Section
    ///// Reminder Top Switch is 12, and Bottom Switch is A3
    ///// Motor Direction Reminder - 
    // voltA - High, voltB - Low ; Clockwise? the direction where it sucks water  
    // voltA - Low, voltB - High ; Counter Clockwise? the direction where it pushes water out 
    /// NOTE, When either digital reads as "1", that means that the switch has yet to be hit

    /*
    Personal Note (Tyerone): Currently this it the simplest and most streamline method for the movement
    I would probably do a switch case statement, but that can only take bools of one checker, not multiple
    like we would need here
    */
    
    /// Float Starts off as Surfaced - Reads that Top Switch has yet to be Hit -
    /// Reads that Bottom Switch has yet to be Hit - Begins to Start Moving Motor to Suck in Water
    if (float_surfaced == true && float_floored == false && digitalRead(12) == 1 && digitalRead(A3) == 1)  { 
      digitalWrite(voltA, HIGH);                     
      digitalWrite(voltB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Still detects as surfaced, for now - 
    /// Top Switch has NOT been Hit - Bottom Switch has been Hit -
    /// Continues to suck in water - Sets float_surfaced to false -
    /// Reasoning: as to directly move to the next if checker
    else if (float_surfaced == true && float_floored == false && digitalRead(12) == 0 && digitalRead(A3) == 1) { 
      float_surfaced = false;
      digitalWrite(voltA, HIGH);
      digitalWrite(voltB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Float is neither Surfaced nor Floored
    /// Top Switch has NOT been Hit - Bottom Switch has been Hit -
    /// Float Motor is turned off to sink down
    else if (float_surfaced == false && float_floored == false && digitalRead(12) == 0 && digitalRead(A3) == 1) { 
      digitalWrite(voltA, LOW);                            
      digitalWrite(voltB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Float is or has been Currently Floored
    /// Top Switch has NOT been Hit - Bottom Switch has been Hit
    /// Float Motor is turned on to push out water, i.e counter clockwise direction
    else if (float_surfaced == false && float_floored == true && digitalRead(12) == 0 && digitalRead(A3) == 1) { 
      digitalWrite(voltA, LOW);                           
      digitalWrite(voltB, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Float is or has been Currently Floored
    /// Top Switch has been Hit - Bottom Switch has been Hit
    /// Float Motor is turned on to push out water, i.e counter clockwise direction
    //// Note this is essentially just a failsaf, ensuring that it pushes out all of the water
    else if (float_surfaced == false && float_floored == true && digitalRead(12) == 1 && digitalRead(A3) == 1) {
      digitalWrite(voltA, LOW);                                                  
      digitalWrite(voltB, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Float is or has been Currently Floored
    /// Top switch has been Hit - Bottomw Switch has NOT been Hit
    /// Float Motor is turned off, to let the float resurface
    /// float_floored is set back to false so that the code can rerun again
    else if (float_surfaced == false && float_floored == true && digitalRead(12) == 1 && digitalRead(A3) == 0) {
      float_floored = false;
      digitalWrite(voltA, LOW);                                                    
      digitalWrite(voltB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Floats been surfaced and is not floored, obviously lol
    /// Top switch has been NOT Hit - Bottom Switch has been Hit
    /// Float Motor should now be sucking in water, essentailly restarting back to the top of the list of the float movement
    else if (float_surfaced == true && float_floored == false && digitalRead(12) == 0 && digitalRead(A3) == 1) {
      digitalWrite(voltA, HIGH);                     
      digitalWrite(voltB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    else if(//Put code which determines if the float is at a depth of 2.5m){
      digitalWrite(voltA, LOW);                                                    
      digitalWrite(voltB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      if(//put timer code which will triger motor functions after 45s){

      }
    }
    //// End of Motor Movement Determiner Section


    //// Multi-threading part ٩( ᐖ )۶


    /// Psi List Data Appender, for data collection when under water
    if (current_millis - list_updater_millis >= list_updater_interval){
      list_updater_millis = current_millis;
      psiList.add(psi);
    }


    /// Half Second psi getter, for getting psi to compare later
    if (current_millis - psi_task_half_millis >= psi_task_half_interval){
      psi_task_full_millis = current_millis;
      psi_half_sec = psi;
    }

    
    /// Full Second psi getter, will also use to compare later
    if (current_millis - psi_task_full_millis >= psi_task_full_interval){
      psi_task_full_millis = current_millis;
      psi_full_sec = psi;
    }
    

    /// Detrminer to decide whether or not the float is floored or surfaced
    /// Based on if there is a significant change in pressure
    if (current_millis - psi_change_check_millis >= psi_change_check_interval){
      psi_change_check_millis = current_millis;
      psi_calc = psi_full_sec - psi_half_sec;

      // Essentially, it determines the absolute value of the psi_calc, makes it so its a positive int,
      // So that if the calc differeance <= 1, then it must to motionless so either floored or surfaced
      if (abs(psi_calc) <= 1 && float_floored == false){
        if (top_switch_pressed == true){
          float_floored = true;
          float_surfaced = false;
        }
        else if(bottom_switch_pressed == false){
          float_floored = false;
          float_surfaced = true;
        }
      }

      if (abs(psi_calc) <= 1 && float_surfaced == false){
        if (top_switch_pressed == true){
          float_floored = true;
          float_surfaced = false;
        }
        else if(bottom_switch_pressed == true){
          float_floored = false;
          float_surfaced = true;
        }
      }
    }
  }
  ////// End if section where the code works when the succesfully recieves a correct signal
}


// Functionssssss (｡·  v  ·｡)

void bottomSwitchDetect(){
  if (digitalRead(A3) == 0){
    bottom_switch_pressed = true;
  }
  else{
    bottom_switch_pressed = false;
  }
}


void topSwitchDetect(){
  if (digitalRead(12) == HIGH){
    top_switch_pressed = true;
  }
  else{
    top_switch_pressed = false;
  }
}
