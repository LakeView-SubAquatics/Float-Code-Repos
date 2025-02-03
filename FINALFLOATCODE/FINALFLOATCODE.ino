/* 
  Author(s): Tyerone Chen, Danny Henningfield, Adam Palma, 

    Innit Create: 6/30/2024
      Last update: 1/18/2025
*/

//// Arduino Float Code Remake

// Included Library
#include <SPI.h> // Used for synchronization of communication between different boards
#include <RH_RF95.h> // Used for the radio and specific adafruit board used
#include <ezButton.h> // Used for the much better button/switch detection
#include <List.hpp> // Used for the simpler functionality to make lists, instead of having to manually redefine arrays.

// Innitial Definitions
  // Radio Communications Setup
#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#endif

#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#pragma region Variable_Definement

// List Definition
  // psiList - float - Contains the data colelceted of the psi
  // depthList - float - Contains the data coillected of the depth
  // timeList - int - Contains a rough time that is parralel to the previous lists
List<float> psiList;
List<float> depthList;
List<unsigned long> timeList;

// Recievement Data Innitializer
char received_data[RH_RF95_MAX_MESSAGE_LEN];

// PSI Calculation Variables
float psi_half_sec = 0;
float psi_full_sec = 0;
float psi_calc = 0;

// Arduino & Motor Port Connection Variables Definition
int outA = 5;
int diag_port_A = 6;
int outB = 11;
int diag_port_B = 10;

int pwm_port = 9;
const DUTY_CYCLE = 255;

ezButton switch_top(12);   // Top Swtch Connected to pin 12
ezButton switch_bottom(A3);// Bottom Switch connected to pin A3

// Enum for Float States
  // SURFACED - The Float is currently surfaced at the top of the water
  // SUBMURSED - The FLoat is currently under the water, but presumed to be not moving. Mostly used as a placeholder in case of a unforseen error.
  // MOVING - The Float is currently moving, presumed as under water.
  // MAINTAIN - The FLoat is maintaing a desired depth, currently not moving
  // FLOORED -  The FLoat is at the bottom of the pool, currently not moving
enum Float_State {
  SURFACED,
  SUBMURSED,
  MOVING,
  MAINTAIN,
  FLOORED
};

// When the Code is Innitiated, the Float should be surfaced
volatile Float_State float_curr_state = SURFACED;

// Enum for Switch States
  //  ACTIVE - Switch is being activated/pressed
  // INACTIVE - Switch is NOT being activated/pressed
enum Switch_State{
  ACTIVE,
  INACTIVE
};

// Neither Switch Should be Activated when the code starts
volatile Switch_State switch_top_state = INACTIVE;
volatile Switch_State switch_bottom_state = INACTIVE;

// Enum for Motor Direction
  // CLOCKWISE - Motor is moving downwards, towards the bottom switch
  // COUNTERCLOCKWISE - Motor is moving upwards, towards the top switch
  // STALLED - Motor is not being moved
enum Motor_Direction {
  CLOCKWISE,
  COUNTERCLOCKWISE,
  STALLED
};
#pragma endregion

#pragma region Setup
void setup() {
  // Defines what each pin should be setup to respond as
  analogWrite(pwm_port, DUTY_CYCLE);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Removes the Delay which the component would detect the Switch Having activity.
  switch_top.setDebounceTime(0);   
  switch_bottom.setDebounceTime(0); 
    // Attach interrupt handler for top switch
  attachInterrupt(digitalPinToInterrupt(12), switchTopDetect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A3), switchBottomDetect, CHANGE);

  // Initiates how each pin on the board should respond
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
  pinMode(12, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, LOW);

  // Feather LoRa TX Tester
  digitalWrite(RFM95_RST, LOW);
  digitalWrite(RFM95_RST, HIGH);

  // Radio response handleres
  while (!rf95.init()) {
    handleNoResponse() // No Responce Handler
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    handleNoResponse() // No Responce Handler
  }
  //Sets Freq to: RF95_FREQ
  rf95.setTxPower(23, false);
}
#pragma endregion

//Innitializes and defines the packet size
int16_t packetnum = 0;

// "Multithreading Section
// Millis Timer Variables
unsigned long radio_task_millis = 0;
unsigned long psi_task_half_millis = 0;
unsigned long psi_task_full_millis = 0;
unsigned long psi_change_check_millis = 0;
unsigned long list_updater_millis = 0;
unsigned long maintain_depth_millis = 0;

// Task Interval Definements
const long RADIO_TASK_INTERVAL = 1001;
const long PSI_TASK_HALF_INTERVAL = 1001;
const long PSI_TASK_FULL_INTERVAL = 1250;
const long PSI_CHANGE_CHECK_INTERVAL = 500;
const long LIST_UPDATER_INTERVAL = 5000; // 5 Seconds
const long MAINTAIN_DEEPTH_DURATION = 60000; // 1 minute 


#pragma region Main_Program/Loop
void loop() {
  // Millis Timer Start
  unsigned long current_millis = millis();

  /// Pressure Based CaLCULATION
    // makes sure that the pressure pin is set to A1
  int pressure_pin = analogRead(A1);
  float psi = (0.0374 * pressure_pin) - 3.3308;
  float pascal_pi = psi * 6894.76;
  float depth = (pascal_pi / (1000 * 9.81) ); // Caqlculated in Meters

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
        handleNoResponse() // No Responce Handler
      }
    } 
    else {
      handleNoResponse() // No Responce Handler
    }
  }
  // End of Radio Communication Checker
#pragma endregion

  // Code which actualy starts/functions after passing the radio communcation check
  if (strcmp(received_data, "initiate") == 0) {
    // Reminder Top Switch is 12, and Bottom Switch is A3
    // Float State determined based on the psi change and what switches are and aren't bneing pressed
    float_curr_state = psiCompare(psi_half_sec, psi_full_sec, depth, switch_bottom_state, switch_top_state);
    // Motor Direction Determined after the Float State is Determined
    motorDirection(float_curr_state);
    if (float_curr_state == MAINTAIN){ 
      maintainDepth(current_millis);
    }

    #pragma region PSI Data Code
    // List Data Adder
    if (current_millis - list_updater_millis >= LIST_UPDATER_INTERVAL){
      list_updater_millis = current_millis;
      psiList.add(psi);
      depthList.add(depth);
      timeList.add(current_millis);
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
    if (current_millis - psi_change_check_millis >= PSI_CHANGE_CHECK_INTERVAL){
      psi_change_check_millis = current_millis;

      float_curr_state = psiCompare(psi_half_sec, psi_full_sec, switch_bottom_state, switch_top_state);
    }
    #pragma endregion

    if (float_curr_state == SURFACED){
      sendData();
    }
  }
}
#pragma endregion


// Functions
#pragma region Functions

void sendData(){
  // Inntialy defines the data
  String data = "PSI: ";
  // Adds each data from the psi list
  for (float psi : psiList){
    data +=  String(psi) + ", ";
  }
  // Adds each data from the depth list
  data += "DEPTH: ";
  for (float depth : depthList){
    data +=  String(depth) + ", ";
  }
  // adds each data from the timeList
  data += "TIME: ";
  for (unsigned long time : timeList){
    data += string(time) + ", ";
  }

  // Converts the string data into a char list
  char send_buffer[RH_RF95_MAX_MESSAGE_LEN];
  message.toCharArray(send_buffer, RH_RF95_MAX_MESSAGE_LEN);

  // Sends sed list
  rf95.send((uint8_t *)send_buffer, strlen(send_buffer));
  rf95.waitPacketSent();
}


// Handles any issues when there is no response from the radio
void handleNoResponse(){
  float_curr_state = STALLED; // Defaults float to stall
  strncpy(received_data, "", sizeof(received_data)); // Clears out recieved_data
}


// Calculates the change in PSI. If it is < 1, then it detects there is MINIMAL change in PSI, meaning that the Float is in one of 3 states
  // MAINTAIN - IF the current Depth is between 2.5m - 2.6m, the State must be MAINTAIN
  // SURFACED - If the Bottom Switch is PRESSED & the TOP Switch is NOT PRESSED, then it must be SURFACED
  // FLOORED - If the Bottom Switch is NOT PRESSED & the TOP Switch is PRESSED, then it must be FLOORED
  // SUBMURSED - If the Bottom Switch is NOT PRESSED & the NOT TOP Switch is PRESSED, then it must be SUBMURSED
enum Float_State psiCompare(float half_time_psi, float full_time_psi, float curr_depth, 
    enum Switch_State switch_bottom_state, enum Switch_State switch_top_state){
  float calc_psi_diff = abs(full_time_psi - half_time_psi);

  if (calc_psi_diff <= 1.0){
    if(curr_depth < 2.6 && curr_depth > 2.5){
      return MAINTAIN;
    } else if (switch_bottom_state == ACTIVE && switch_top_state == INACTIVE){
      return SURFACED;
    } else if(switch_bottom_state == INACTIVE && switch_top_state == ACTIVE){
      return FLOORED;
    } else if(switch_bottom_state == INACTIVE && switch_top == INACTIVE){
      return SUBMURSED;
    } else{
    return MOVING;
    }
  }
}


// Motor Diurefction determiner, based on float state
void motorDirection(enum State float_state){
  switch (float_state){
  case SURFACED: // Counter-Clockwise Motor Movemenet, Sucks in water
    digitalWrite(outA, LOW);
    digitalWrite(outB, HIGH);
    break;
  
  case SUBMURSED: // Clockwise Motor Movemenet, Pushes out water
  // Add code where it'll stay at a certain depth for a certain amount of time before resuming movement
    digitalWrite(outA, HIGH);
    digitalWrite(outB, LOW);
    break;
  
   case FLOORED: // Clockwise Motor Movemenet, Pushes out water
    digitalWrite(outA, HIGH);
    digitalWrite(outB, LOW);
    break;

   case MOVING:
    digitalWrite(outA, LOW);
    digitalWrite(outB, LOW);
    break;
  
   case MAINTAIN:
   // Default to stall
    digitalWrite(outA, LOW);
    digitalWrite(outB, LOW);
    maintain_depth_millis = millis(); // Starts a Timer
    break;

  default: // defaults to have the motor to be stalled
    digitalWrite(outA, LOW);
    digitalWrite(outB, LOW);
    break;
  }
  digitalWrite(LED_BUILTIN, LOW); // Ensures switch will always have a resistor setup regardless of state
}


// Depth Maintainer
void maintainDepth(unsigned long current_millis){
  // Once the time reaches 1 minute, resumes to moving satate
  if (current_millis - maintain_depth_millis >= MAINTAIN_DEEPTH_DURATION){
    float_curr_state = MOVING;
  } else{
    // If not it'll stay at a no movement state
    digitalWrite(outA, LOW);
    digitalWrite(outB, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  }
}


// Switch Detections
void switchBottomDetect(){
  if (digitalRead(A3) == HIGH){
    switch_bottom_state = ACTIVE;
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
