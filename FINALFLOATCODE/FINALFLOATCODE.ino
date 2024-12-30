/* 
  Author(s): Tyerone Chen, Danny Henningfield, Adam Palma, 

    Innit Create: 6/30/2024
      Last update: 12/30/2024
*/

// Arduino Float Code Remake

// Side Note, we need to comment the crap out of this becuase i had an 
// aneurism reading the old code （´∇｀''）
// Adam note: me too, i am now brain damaged .-.

// Included Library
#include <SPI.h>
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
  // psiList - float
  // depthList - float
  // timeList - int
List<float> psiList;
List<float> depthList;
List<int> timeList;

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
  // SURFACED - 
  // SUBMURSED - 
  // MOVING - 
  // FLOORED - 
enum Float_State {
  SURFACED,
  SUBMURSED,
  MOVING,
  FLOORED
};

// When the Code is Innitiated, the Float should be surfaced
volatile Float_State float_curr_state = SURFACED;

// Enum for Switch States
  //  ACTIVE - 
  // INACTIVE -
enum Switch_State{
  ACTIVE,
  INACTIVE
};

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
};
#pragma endregion

#pragma region Setup
void setup() {
  #pragma region Pin_Definition

  analogWrite(pwm_port, DUTY_CYCLE);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Removes the Delay which the component would detect the Switch Having activity.
  switch_top.setDebounceTime(0);   
  switch_bottom.setDebounceTime(0); 

  // Initiates how each pin on the board should work
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
  attachInterrupt(digitalPinToInterrupt(12), switchTopDetect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A3), switchBottomDetect, CHANGE);
}
#pragma endregion

//to init and define it 
int16_t packetnum = 0;

// "Multithreading Section"
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

  /// Pressure CaLCULATION
    // makes sure that the pressure pin is set to A1
  int pressure_pin = analogRead(A1);
  float psi = (0.0374 * pressure_pin) - 3.3308;

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
    // Reminder Top Switch is 12, and Bottom Switch is A3

    // Motor Movement Determiner
    float_curr_state = psiCompare(psi_half_sec, psi_full_sec, switch_bottom_state, switch_top_state);
    motorDirection(float_curr_state);

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
    if (current_millis - psi_change_check_millis >= PSI_CHANGE_CHECK_INTERVAL){
      psi_change_check_millis = current_millis;

      float_curr_state = psiCompare(psi_half_sec, psi_full_sec, switch_bottom_state, switch_top_state);
    }
  }
}
#pragma endregion

#pragma endregion


// Functions
#pragma region Functions

// Calculates the change in PSI. If it is < 1, then it detects there is MINIMAL change in PSI, meaning that the Float is in one of 3 states
  // SURFACED - If the Bottom Switch is PRESSED & the TOP Switch is NOT PRESSED, then it must be SURFACED
  // FLOORED - If the Bottom Switch is NOT PRESSED & the TOP Switch is PRESSED, then it must be FLOORED
  // SUBMURSED - If the Bottom Switch is NOT PRESSED & the NOT TOP Switch is PRESSED, then it must be SUBMURSED
enum Float_State psiCompare(int half_time_psi, int full_time_psi, enum Switch_State switch_bottom_state, enum Switch_State switch_top_state){
  float calc_psi_diff = abs(full_time_psi - half_time_psi);

  if (calc_psi_diff <= 1.0){
    if(switch_bottom_state == ACTIVE && switch_top_state == INACTIVE){
      return SURFACED;
    } else if(switch_bottom_state == INACTIVE && switch_top_state == ACTIVE){
      return FLOORED;
    } else if(switch_bottom_state == INACTIVE && switch_top == INACTIVE){
      return SUBMURSED;
    }  
    } else{
    return MOVING;
  }
}

// Main Bulk of the Code
void motorDirection(enum State float_state){
  switch (float_state)
  {
  case SURFACED // Counter-Clockwise Motor Movemenet, Sucks in water
    digitalWrite(outA, LOW);
    digitalWrite(outB, HIGH);
    break;
  
  case SUBMURSED // Clockwise Motor Movemenet, Pushes out water
  // Add code where it'll stay at a certain depth for a certain amount of time before resuming movement
    digitalWrite(outA, HIGH);
    digitalWrite(outB, LOW);
    break;
  
   case FLOORED // Clockwise Motor Movemenet, Pushes out water
    digitalWrite(outA, HIGH);
    digitalWrite(outB, LOW);
    break;
  
  default: // defaults to have the motor to be stalled
    digitalWrite(outA, LOW);
    digitalWrite(outB, LOW);
    break;
  }
  digitalWrite(LED_BUILTIN, LOW); // Ensures switch will always have a resistor setup regardless of state
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