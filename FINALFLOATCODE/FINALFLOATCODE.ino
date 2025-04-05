/* 
  Author(s): Tyerone Chen, Danny Henningfield, Adam Palma
  Init Create: 6/30/2024
  Last update: 4/4/2025
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <ezButton.h>
#include <List.hpp>

#pragma region Variable_Definement

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#endif

// Radio Definement
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
char received_data[RH_RF95_MAX_MESSAGE_LEN];
int16_t packetnum = 0;

// List Definement
List<float> psiList;
List<float> depthList;
List<unsigned long> timeList;

// List Variables Definement
const int MAX_LIST_SIZE = 2000;
const String COMPANY_NAME = "LakeView SubAquatics-#";
float psi_half_sec = 0.0;
float psi_full_sec = 0.0;
float psi_change = 0.0;
bool has_maintained = false;
int maintain_updates = 0;
const int MAX_MAINTAINS = 12; // WIll be roughly 1 Minute of total time of maintaining depth
const float MIN_MAINTAIN_DEPTH = 2.4;
const float MAX_MAINTAIN_DEPTH = 2.6;

// Port Definement
const int OUT_A = 5;
const int DIAG_PORT_A = 6;
const int OUT_B = 11;
const int DIAG_PORT_B = 10;
const int PWM_PORT = 9;
const int DUTY_CYCLE = 255;
const int PRESSURE_PIN = A1;
const int SWITCH_TOP_PIN = 12;
const int SWITCH_BOTTOM_PIN = A3;

// TImer Variables Definemnt - In MiliSeconds
// Timers
unsigned long radio_task_millis = 0;
unsigned long psi_task_half_millis = 0;
unsigned long psi_task_full_millis = 0;
unsigned long psi_change_check_millis = 0;
unsigned long list_updater_millis = 0;
//unsigned long maintain_depth_millis = 0;

// Intervals
const long RADIO_TASK_INTERVAL = 1000; // About 1 Second
const long PSI_TASK_HALF_INTERVAL = 1001; // About 1/2 Second
const long PSI_TASK_FULL_INTERVAL = 1250; // 1.25 Seconds
const long PSI_CHANGE_CHECK_INTERVAL = 500; // 1 a second
const long LIST_UPDATER_INTERVAL = 5000; // 5 Seconds
//const long MAINTAIN_DEEPTH_DURATION = 60000; // 1 Minute

// Button Port Definement
ezButton switch_top(SWITCH_TOP_PIN);
ezButton switch_bottom(SWITCH_BOTTOM_PIN);

bool send_float = false;

// Float Movement
enum Float_State { 
  SURFACED, 
  MOVING_UP,
  MOVING_DOWN, 
  FLOORED,
  MAINTAIN, // Hold Position
  IDLE // Stall until further notice
};
volatile Float_State float_curr_state = SURFACED;

enum Motor_Direction { 
  CLOCKWISE, 
  COUNTERCLOCKWISE, 
  STALLED 
};
volatile Motor_Direction motor_direction = STALLED;

#pragma endregion

#pragma region Setup
void setup() {
  // Establish Port Settings
  analogWrite(PWM_PORT, DUTY_CYCLE);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  pinMode(OUT_A, OUTPUT);
  pinMode(OUT_B, OUTPUT);
  pinMode(DIAG_PORT_A, INPUT_PULLUP);
  pinMode(DIAG_PORT_B, INPUT_PULLUP);
  pinMode(PWM_PORT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(DIAG_PORT_A, HIGH);
  digitalWrite(DIAG_PORT_B, HIGH);

  // Establish Button Port Settings
  switch_top.setDebounceTime(50);
  switch_bottom.setDebounceTime(50);

  pinMode(12, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  
  // Get bottom switch state
  switch_bottom.loop();
  // Sets the motor to hit the bottom switch if it isn't already
  if (!switch_bottom.isPressed()){
    float_curr_state = MOVING_DOWN;
    motor_direction = CLOCKWISE;
  } 
  else {
    float_curr_state = IDLE;
    motor_direction = STALLED;
  }

  // Setup radio pins
  digitalWrite(RFM95_RST, LOW);
  digitalWrite(RFM95_RST, HIGH);

  while (!rf95.init()) {
    handleNoResponse();
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    handleNoResponse();
  }
  rf95.setTxPower(23, false);
}
#pragma endregion


#pragma region Main_Program/Loop
void loop() {
  // Save Current Time
  unsigned long current_millis = millis();

  // Calculate Current Pressure
  float pressure_volt_reading = analogRead(PRESSURE_PIN);
  float psi = (0.0374 * pressure_volt_reading) - 3.3308;
  float pascal_pi = (psi - 14.7) * 6894.76;
  float depth = (pascal_pi / (1002 * 9.81));
  depth = depth < 0 ? 0 : depth;

  // Check Limit Switch/Button State
  switch_top.loop();
  switch_bottom.loop();
  bool switch_top_state = switch_top.isPressed();
  bool switch_bottom_state = switch_bottom.isPressed();

  // Check to See that Radio is Connected
  if (current_millis - radio_task_millis >= RADIO_TASK_INTERVAL) {
    radio_task_millis = current_millis;
    if (rf95.waitAvailableTimeout(1000)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        strncpy(received_data, (char*)buf, len);
      } 
      else {
        handleNoResponse();
      }
    } 
    else {
      handleNoResponse();
    }
  }

  if (strcmp(received_data, "initiate") == 0) {
    send_float = true; 
  }

  if (send_float){
    // Gets data for current psi at 1/2 a second    
    if (current_millis - psi_task_half_millis >= PSI_TASK_HALF_INTERVAL) {
      psi_task_half_millis = current_millis;
      psi_half_sec = psi;
    }
    // Gets data for current psi at 1 a second  
    if (current_millis - psi_task_full_millis >= PSI_TASK_FULL_INTERVAL) {
      psi_task_full_millis = current_millis;
      psi_full_sec = psi;
    }
    // Determines if the float is moving based on a change of psi
    if (current_millis - psi_change_check_millis >= PSI_CHANGE_CHECK_INTERVAL) {
      psi_change_check_millis = current_millis;
      psi_change = psi_half_sec - psi_full_sec;
    }

    // Checks what the float state will be currently at
    checkMotorState(float_curr_state, depth, abs(psi_change), switch_top_state, switch_bottom_state);
    moveMotor(motor_direction);

    // Every 10 Seconds update list
    if (current_millis - list_updater_millis >= LIST_UPDATER_INTERVAL) {
      list_updater_millis = current_millis;
      // If data gets too large reset list 
      if (psiList.getSize() >= MAX_LIST_SIZE) {
        psiList.remove(0);
        depthList.remove(0);
        timeList.remove(0);
      }
      // Adds data list
      psiList.add(psi);
      depthList.add(depth);
      timeList.add(current_millis);
      // If the float is MAINTAIN update the ammount of times list has been updated while under this state
      if (float_curr_state == MAINTAIN){
        maintain_updates++;
      }
    }

    if (float_curr_state == SURFACED) {
      if (rf95.waitAvailableTimeout(1000)) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (rf95.recv(buf, &len)) {
          strncpy(received_data, (char*)buf, len);
          sendData();
          has_maintained = false;
          maintain_updates = 0;
        } 
        else {
          handleNoResponse();
        }
      } 
      else {
        handleNoResponse();
      }
    }
  }
}
#pragma endregion

#pragma region Functions
void sendData() {
  String data = COMPANY_NAME + ": ";
  data += "PSI: ";
  for (int i = 0; i < psiList.getSize(); i++) {
    data += String(psiList.get(i)) + ", ";
  }
  data += "DEPTH: ";
  for (int i = 0; i < depthList.getSize(); i++) {
    data += String(depthList.get(i)) + ", ";
  }
  data += "TIME: ";
  for (int i = 0; i < timeList.getSize(); i++) {
    data += String(timeList.get(i)) + ", ";
  }

  if (data.length() >= RH_RF95_MAX_MESSAGE_LEN) {
    data = data.substring(0, RH_RF95_MAX_MESSAGE_LEN - 1);
  }

  char send_buffer[RH_RF95_MAX_MESSAGE_LEN];
  data.toCharArray(send_buffer, RH_RF95_MAX_MESSAGE_LEN);
  rf95.send((uint8_t *)send_buffer, strlen(send_buffer));
  rf95.waitPacketSent();
}

void handleNoResponse() {
  // Add things
}

void checkMotorState (Float_State float_state, float curr_depth, float psi_change, bool switch_top_state, bool switch_bottom_state) {
  switch (float_state) {
    case SURFACED:
      if (psi_change != 0){
        float_curr_state = IDLE; 
        motor_direction = STALLED;
      }
      else {
        float_curr_state = MOVING_UP; 
        motor_direction = COUNTERCLOCKWISE; 
      }
      break;
    case FLOORED:
      if (psi_change != 0){
        float_curr_state = IDLE; 
        motor_direction = STALLED;
      }
      else {
        float_curr_state = MOVING_DOWN; 
        motor_direction = CLOCKWISE; 
      }
      break;
    case MOVING_UP:
      if (switch_bottom_state){
        float_curr_state = SURFACED;
        motor_direction = STALLED;
      }
      else {
        float_curr_state = MOVING_UP;
        motor_direction = COUNTERCLOCKWISE;
      }
      break;
    case MOVING_DOWN:
      if (switch_top_state){
        float_curr_state = FLOORED;
        motor_direction = STALLED;
      }
      else {
        float_curr_state = MOVING_DOWN;
        motor_direction = CLOCKWISE;
      }
      break;
    case MAINTAIN:
      maintainDepth(maintain_updates, curr_depth);
      break;
    case IDLE:
      motor_direction = STALLED;
      break;
    default:
      float_curr_state = IDLE;
      motor_direction = STALLED;
      break;
  }
}

void moveMotor(Motor_Direction motor_direction){
  switch(motor_direction){
    case CLOCKWISE:
      digitalWrite(OUT_A, HIGH);
      digitalWrite(OUT_B, LOW);
      break;
    case COUNTERCLOCKWISE:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, HIGH);
      break;
    case STALLED:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
      break;
    default:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
      break;
  }
}

void maintainDepth(int total_maintain_updates, float curr_depth) {
  if (total_maintain_updates <= MAX_MAINTAINS) { // Check to see how many times the list has been updated while MAINTAINED
    if (curr_depth < MIN_MAINTAIN_DEPTH) { // Attempts to readjust float
      motor_direction = CLOCKWISE;
    } 
    else if (curr_depth > MAX_MAINTAIN_DEPTH) { // Attempts to readjust float
      motor_direction = COUNTERCLOCKWISE;
    } 
    else {
      motor_direction = STALLED;
    }
  } 
  else {
    has_maintained = true;
    float_curr_state = MOVING_DOWN;
  }
}

#pragma endregion

// This code makes me want tpo cry - Tyerone
