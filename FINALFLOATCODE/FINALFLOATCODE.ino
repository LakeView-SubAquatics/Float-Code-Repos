/* 
  Author(s): Tyerone Chen, Danny Henningfield, Adam Palma, 

    Innit Create: 6/30/2024
      Last update: 12/19/2024
*/

// ** NEEDED CHANGES REMINDER ** //
// Remove a bunch of the serial prints or whatever so we can burn the code onto the arduino. - DONE
// Set any use of the pinmodesin the code as referances as to help with readability of the code.
// Remake the code within the MAIN LOOP Program to be in functions which'll be called in
// Remake the motor process code to bealigned with the new competition requirements.
  // Try to setup enum data types and terenary operator for simpler and much more enhanced code?
  // Make sure to include "digitalWrite(LED_BUILTIN, LOW);" in any code which changes motor movement
    // THis is to ensure that the switches have a resistor for the voltage that is passed through
// Add some necessary checks and failsafes for the radio transmission code.
// Genrally clean up whatever miss haps and naming issues there are in the code.
// Remove or comment out any methods refering to psi change.
// Change the recieved data info into a status codes

// Arduino Float Code Remake

// Side Note, we need to comment the crap out of this becuase i had an 
// aneurism reading the old code （´∇｀''）
// Adam note: me too, i am now brain damaged .-.


// Included Library
#include <SPI.h>
#include <RH_RF95.h> // Used for the radio and specific adafruit board used
#include <ezButton.h> // Used for the much better button/switch detection
#include <List.hpp> // Used for the simpler functionality to make lists, instead of having to manually redefine arrays.



// Init definition and crap
//Radio crap
#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#endif

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

#pragma region Variable_Definement

// list defin area - Type: float, int
List<float> psiList;
List<float> depthList;
List<int> timeList;

// Data recived innit
// essentialy this is where the cariable of the recieved data, from the pi, will be held
// should always be a string, which is formated as a list of chars
char received_data[RH_RF95_MAX_MESSAGE_LEN];

// PSI Calculation Variables
float psi_half_sec = 0;
float psi_full_sec = 0;
float psi_calc = 0;

// Arduino & Motor Port Connection Variables
int outA = 5;
int diag_port_A = 6;

int outB = 11;
int diag_port_B = 10;

int pwm_port = 9;

const DUTY_CYCLE = 255;

ezButton switch_top(12);   // Top Siwtch Connected to pin 12
ezButton switch_bottom(A3);// Bottom switch connected to pin A3

// Enum for Float States
  // SURFACED - 
  // SUBMURSED - 
  // MOVING - 
  // FLOORED - 
enum Float_State {
  SURFACED,
  SUBMURSED,
  MOVING,
  FLOORED
}

// When the Code is Innitiated, the Float should be surfaced
volatile Float_State float_curr_state = SURFACED;

// Enum for Switch States
  //  ACTIVE - 
  // INACTIVE -
enum Switch_State{
  ACTIVE,
  INACTIVE
}

// Neither Switch Should be Activated when the code starts
volatile Switch_State switch_top_state = INACTIVE;
volatile Switch_State switch_bottom_state = INACTIVE;

// Enum for Motor Direction
  // CLOCKWISE - 
  // COUNTERCLOCKWISE - 
  // STALLED - 
enum Motor_Direction {
  CLOCKWISE,
  COUNTERCLOCKWISE,
  STALLED
}

#pragma endregion


#pragma region Setup
void setup() {
  analogWrite(pwm_port, DUTY_CYCLE);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Removes the Delay which the component would detect the Switch Having activity.
  switch_top.setDebounceTime(0);   
  switch_bottom.setDebounceTime(0); 

  // Initiates how each pin on the board should work

  // Try to set as addresses and pointers for the pins, to make it clearer what they do and which pins they are
  #pragma region Pin_Definition
  pinMode(outA, OUTPUT);
  pinMode(outB, OUTPUT);
  pinMode(diag_port_A, INPUT_PULLUP);
  pinMode(diag_port_B, INPUT_PULLUP);
  pinMode(pwm_port, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  // Enables the Diag Ports
  digitalWrite(diag_port_A, HIGH);
  digitalWrite(diag_port_B, HIGH);

  // Resistor setting for the Limit Swtches
  pinmode(12, INPUT_PULLUP);
  pinmode(A1, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, LOW);
  #pragma endregion

  // Feather LoRa TX Test!
  digitalWrite(RFM95_RST, LOW);
  digitalWrite(RFM95_RST, HIGH);

  while (!rf95.init()) {
    // LoRa radio init failed
    // Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    //setFrequency failed!
    while (1);  // lock up code, this would suck 
  }
  //Set Freq to: RF95_FREQ

  // set transfer power to 0
  rf95.setTxPower(23, false);
  // Attach interrupt handler for top switch
  // May Remove?
  attachInterrupt(digitalPinToInterrupt(12), switchTopDetect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A3), switchBottomDetect, CHANGE);
}
#pragma endregion

//to init and define it 
int16_t packetnum = 0;

// multithreading / timer code setup
// reminder* are all in terms of millis, so 1000 millis = 1 sec
// Type - unsigned long, const long
// All "const long" are basically the interval
// for any newbies, const means it will never change, and long is simmilar to the double time, except it doesnt do decimal
// unsigned is essentially a value type with only stores in positive integers for memory saving, which works here cuz we only count up

unsigned long radio_task_millis = 0;
unsigned long psi_task_half_millis = 0;
unsigned long psi_task_full_millis = 0;
unsigned long psi_change_check_millis = 0;
unsigned long list_updater_millis = 0;

const long RADIO_TASK_INTERVAL = 1001;
const long PSI_TASK_HALF_INTERVAL = 1001;
const long PSI_TASK_FULL_INTERVAL = 1250;
const long PSI_CHANGE_CHECK_INTERVAL = 500;
const long LIST_UPDATER_INTERVAL = 5000;


#pragma region Main_Program/Loop
void loop() {
  
  // Millis Timer Start
  unsigned long current_millis = millis();

  /// pressure calc
  // makes sure that the pressure pin is set to A1
  int pressure_pin = analogRead(A1);
  float psi = (0.0374 * pressure_pin) - 3.3308;

  // switch loop function, refer to function for more info 

#pragma region Radio_Communications 
  
  // Radio Communication Checker
  if (current_millis - radio_task_millis >= RADIO_TASK_INTERVAL){
    radio_task_millis = current_millis;

    if (rf95.waitAvailableTimeout(1000)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) {
        
        
        // Store the received data in the global variable
        strncpy(received_data, (char*)buf, len);
        
      } 
      else {
        //Receive failed
      }
    } 
    else {
      // No reply, is there a listener around?
    }
  }
  // End of Radio Communication Checker

#pragma endregion


  // Code which actualy starts/functions after passing the radio communcation check
  if (strcmp(received_data, "initiate") == 0) {
    // Motor Movement Determiner Section
    // Reminder Top Switch is 12, and Bottom Switch is A3
    
    // Motor Direction Reminder - 
    
    // outA - High, outB - Low ; Clockwise? the direction where it sucks water  
    // outA - Low, outB - High ; Counter Clockwise? the direction where it pushes water out


    /// NOTE, When either digital reads as "1", that means that the switch has yet to be hit

    /*
      Personal Note (Tyerone): Currently this it the simplest and most streamline method for the movement
      I would probably do a switch case statement, but that can only take bools of one checker, not multiple
      like we would need here
    */

    /*
      New Motor Process should be the same as before, where we lower down to a depth, and then rise back up and then repeat.
      However there will be a change on the flooring process
      Instead of being at the bottom of the pool. We'll need to be at a depth of 2.5m, for at LEAST 45sec.
      THis shouldn't be too muich of a change.
    */
    
    /// Float Starts off as Surfaced - Reads that Top Switch has yet to be Hit -
    /// Reads that Bottom Switch has yet to be Hit - Begins to Start Moving Motor to Suck in Water
    if (float_surfaced == true && float_floored == false && digitalRead(12) == 1 && digitalRead(A3) == 1)  { 
      digitalWrite(outA, HIGH);                     
      digitalWrite(outB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Still detects as surfaced, for now - 
    /// Top Switch has NOT been Hit - Bottom Switch has been Hit -
    /// Continues to suck in water - Sets float_surfaced to false -
    /// Reasoning: as to directly move to the next if checker
    else if (float_surfaced == true && float_floored == false && digitalRead(12) == 0 && digitalRead(A3) == 1) { 
      float_surfaced = false;
      digitalWrite(outA, HIGH);
      digitalWrite(outB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Float is neither Surfaced nor Floored
    /// Top Switch has NOT been Hit - Bottom Switch has been Hit -
    /// Float Motor is turned off to sink down
    else if (float_surfaced == false && float_floored == false && digitalRead(12) == 0 && digitalRead(A3) == 1) { 
      digitalWrite(outA, LOW);                            
      digitalWrite(outB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Float is or has been Currently Floored
    /// Top Switch has NOT been Hit - Bottom Switch has been Hit
    /// Float Motor is turned on to push out water, i.e counter clockwise direction
    else if (float_surfaced == false && float_floored == true && digitalRead(12) == 0 && digitalRead(A3) == 1) { 
      digitalWrite(outA, LOW);                           
      digitalWrite(outB, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Float is or has been Currently Floored
    /// Top Switch has been Hit - Bottom Switch has been Hit
    /// Float Motor is turned on to push out water, i.e counter clockwise direction
    //// Note this is essentially just a failsaf, ensuring that it pushes out all of the water
    else if (float_surfaced == false && float_floored == true && digitalRead(12) == 1 && digitalRead(A3) == 1) {
      digitalWrite(outA, LOW);                                                  
      digitalWrite(outB, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Float is in the water - Float is or has been Currently Floored
    /// Top switch has been Hit - Bottomw Switch has NOT been Hit
    /// Float Motor is turned off, to let the float resurface
    /// float_floored is set back to false so that the code can rerun again
    else if (float_surfaced == false && float_floored == true && digitalRead(12) == 1 && digitalRead(A3) == 0) {
      float_floored = false;
      digitalWrite(outA, LOW);                                                    
      digitalWrite(outB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /// Floats been surfaced and is not floored, obviously lol
    /// Top switch has been NOT Hit - Bottom Switch has been Hit
    /// Float Motor should now be sucking in water, essentailly restarting back to the top of the list of the float movement
    else if (float_surfaced == true && float_floored == false && digitalRead(12) == 0 && digitalRead(A3) == 1) {
      digitalWrite(outA, HIGH);                     
      digitalWrite(outB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }

    /*
    else if (//Put code which determines if the float is at a depth of 2.5m){
      digitalWrite(outA, LOW);                                                    
      digitalWrite(outB, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      if(//put timer code which will triger motor functions after 45s){

      }
    }
    */

    //// End of Motor Movement Determiner Section


    //// Multi-threading part ٩( ᐖ )۶
#pragma region PSI Data Code
    if (current_millis - list_updater_millis >= LIST_UPDATER_INTERVAL){
      list_updater_millis = current_millis;
      psiList.add(psi);
    }

    /// Half Sec PSI
    if (current_millis - psi_task_half_millis >= PSI_TASK_HALF_INTERVAL){
      psi_task_full_millis = current_millis;
      psi_half_sec = psi;
    }
    /// Full Sec PSI
    if (current_millis - psi_task_full_millis >= PSI_TASK_FULL_INTERVAL){
      psi_task_full_millis = current_millis;
      psi_full_sec = psi;
    }
    

    /// Detrminer to decide whether or not the float is floored or surfaced
    /// Based on if there is a significant change in pressure
    if (current_millis - psi_change_check_millis >= PSI_CHANGE_CHECK_INTERVAL)
      psi_change_check_millis = current_millis;

      float_curr_state = psiCompare(psi_half_sec, psi_full_sec, switch_bottom_state, switch_top_state);
    }
  }
#pragma endregion

}
#pragma endregion


// Functions (｡· v ·｡)
#pragma region Functions

// Calculates the change in PSI. If it is < 1, then it detects there is MINIMAL change in PSI, meaning that the Float is in one of 3 states
  // SURFACED - If the Bottom Switch is PRESSED & the TOP Switch is NOT PRESSED, then it must be SURFACED
  // FLOORED - If the Bottom Switch is NOT PRESSED & the TOP Switch is PRESSED, then it must be FLOORED
  // SUBMURSED - If the Bottom Switch is NOT PRESSED & the NOT TOP Switch is PRESSED, then it must be SUBMURSED
enum Float_State psiCompare(int half_time_psi, int full_time_psi, enum Switch_State switch_bottom_state, enum Switch_State switch_top_state){
  int calc_psi_diff = abs(full_time_psi - half_time_psi);

  if (calc_psi_diff <= 1){
    if(switch_bottom_state == ACTIVE && switch_top_state == INACTIVE){
      return SURFACED;
    } else if(switch_bottom_state == INACTIVE && switch_top_state == ACTIVE){
      return FLOORED;
    } else if(switch_bottom_state == INACTIVE && switch_top == INACTIVE){
      return SUBMURSED;
    }
  }
}

// Main Bulk of the Code
void motorDirection(enum State float_state, enum Switch_State switch_bottom_state, enum Switch_State switch_top_state){

}

void switchBottomDetect(){
  if (digitalRead(A3) == HIGH){
    switch_bottom_state = ACTIVE
  } else{
    bottom_switch_pressed = INACTIVE;
  }
}

void switchTopDetect(){
  if (digitalRead(12) == HIGH){
    top_switch_pressed = ACTIVE;
  } else{
    top_switch_pressed = INACTIVE;
  }
}

#pragma endregion