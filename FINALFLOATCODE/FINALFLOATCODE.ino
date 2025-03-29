/* 
  Author(s): Tyerone Chen, Danny Henningfield, Adam Palma
  Innit Create: 6/30/2024
  Last update: 3/28/2025
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
float depth = 0;
float psi_half_sec = 0;
float psi_full_sec = 0;
bool has_maintained = false;
int maintain_updates = 0;
const int MAX_MAINTAINS = 12; // WIll be roughly 1 Minute of total time of maintaining depth

// Port Definement
const int OUT_A = 5;
const int DIAG_PORT_A = 6;
const int OUT_B = 11;
const int DIAG_PORT_B = 10;
const int PWM_PORT = 9;
const int DUTY_CYCLE = 255;

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
const long PSI_TASK_HALF_INTERVAL = 500; // About 1/2 Second
const long PSI_TASK_FULL_INTERVAL = 1250; // 1.25 Seconds
const long PSI_CHANGE_CHECK_INTERVAL = 1000; // 1 a second
const long LIST_UPDATER_INTERVAL = 5000; // 5 Seconds
//const long MAINTAIN_DEEPTH_DURATION = 60000; // 1 Minute

// Button Port Definement
ezButton switch_top(12);
ezButton switch_bottom(A3);

// Float Movement
enum Float_State { 
  SURFACED, 
  SUBMURSED, 
  MOVING, 
  MAINTAIN, 
  FLOORED 
};

volatile Float_State float_curr_state = SURFACED;

float const MAX_MAINTAIN_DEPTH = 2.6;
float const MIN_MAINTAIN_DEPTH = 2.4;
bool send_float = false;

enum Switch_State { 
  ACTIVE, 
  INACTIVE 
};

volatile Switch_State switch_top_state = INACTIVE;
volatile Switch_State switch_bottom_state = INACTIVE;

enum Motor_Direction { 
  CLOCKWISE, 
  COUNTERCLOCKWISE, 
  STALLED 
};
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
  attachInterrupt(digitalPinToInterrupt(12), switchTopDetect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A3), switchBottomDetect, CHANGE);

  pinMode(12, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  // Pressure Based Calculations Setup
  float pressure_pin = analogRead(A1);
  float psi = (0.0374 * pressure_pin) - 3.3308;
  float pascal_pi = (psi - 14.7) * 6894.76;
  depth = (pascal_pi / (1002 * 9.81));
  // Start up Initial Float State
  float_curr_state = psiCompare(psi_half_sec, psi_full_sec, depth, switch_bottom_state, switch_top_state, switch_bottom.isPressed(), switch_top.isPressed());

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
  float pressure_pin = analogRead(A1);
  float psi = (0.0374 * pressure_pin) - 3.3308;
  float pascal_pi = (psi - 14.7) * 6894.76;
  depth = (pascal_pi / (1002 * 9.81));
  depth = depth < 0 ? 0 : depth;

  // Check Limit Switch/Button State
  switch_top.loop();
  switch_bottom.loop();
  bool ez_switch_top = switch_top.isPressed();
  bool ez_switch_bottom = switch_bottom.isPressed();

  // Check to See that Radio is Connected
  if (current_millis - radio_task_millis >= RADIO_TASK_INTERVAL) {
    radio_task_millis = current_millis;
    if (rf95.waitAvailableTimeout(1000)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        strncpy(received_data, (char*)buf, len);
      } else {
        handleNoResponse();
      }
    } else {
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
      float_curr_state = psiCompare(psi_half_sec, psi_full_sec, depth, switch_bottom_state, switch_top_state, ez_switch_bottom, ez_switch_top);
    }

    // Checks what the float state will be currently at
    float_curr_state = psiCompare(psi_half_sec, psi_full_sec, depth, switch_bottom_state, switch_top_state, ez_switch_bottom, ez_switch_top);
    // Acts accordingly based on the state
    motorDirection(float_curr_state, depth, psi_half_sec - psi_full_sec);
    // If the float is Maintaining run maintain func
    if (float_curr_state == MAINTAIN) {
      maintainDepth(maintain_updates, depth);
    }

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
        } else {
          handleNoResponse();
        }
      } else {
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

Float_State psiCompare(float half_time_psi, float full_time_psi, float curr_depth, 
    Switch_State switch_bottom_state, Switch_State switch_top_state, bool ez_switch_bottom, bool ez_switch_top) {
  float calc_psi_diff = abs(full_time_psi - half_time_psi);

  if (calc_psi_diff <= 1.0) {
    if (curr_depth <= MAX_MAINTAIN_DEPTH && curr_depth >= MIN_MAINTAIN_DEPTH && !has_maintained) {
      return MAINTAIN;
    } else if ((switch_bottom_state == ACTIVE) || (ez_switch_bottom == true)) {
      return SURFACED;
    } else if ((switch_top_state == ACTIVE) || (ez_switch_top == true)) {
      return FLOORED;
    } else if ((switch_bottom_state == INACTIVE && switch_top_state == INACTIVE) || (ez_switch_bottom == false && ez_switch_top == false)) {
      return SUBMURSED;
    }
  }
  return MOVING;
}

void motorDirection(Float_State float_state, float curr_depth, float psi_diff) {
  switch (float_state) {
    case SURFACED:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case SUBMURSED:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case FLOORED:
      digitalWrite(OUT_A, HIGH);
      digitalWrite(OUT_B, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case MOVING:
      if (psi_diff < 0) { // Psi diff will be negative meaning it must be moving down and will continue to do so
        digitalWrite(OUT_A, HIGH);
        digitalWrite(OUT_B, LOW);
      } else if (psi_diff > 0) { // Psi diff will be positive meaning it must be moving up and will continue to do so
        digitalWrite(OUT_A, LOW);
        digitalWrite(OUT_B, HIGH);
      }
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case MAINTAIN:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      break;
    default:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      break;
  }
  pinMode(12, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
}

void maintainDepth(int total_maintain_updates, float curr_depth) {
  if (total_maintain_updates <= MAX_MAINTAINS) { // Check to see how many times the list has been updated while MAINTAINED
    if (curr_depth < MIN_MAINTAIN_DEPTH) { // Attempts to readjust float
      digitalWrite(OUT_A, HIGH);
      digitalWrite(OUT_B, LOW);
    } else if (curr_depth > MAX_MAINTAIN_DEPTH) { // Attempts to readjust float
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, HIGH);
    } else {
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
    }
  } else {
    has_maintained = true;
    float_curr_state = MOVING;
  }
  pinMode(12, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
}

void switchBottomDetect() {
  if (digitalRead(A3) == HIGH) {
    switch_bottom_state = ACTIVE;
    switch_top_state = INACTIVE;
  }
}

void switchTopDetect() {
  if (digitalRead(12) == HIGH) {
    switch_top_state = ACTIVE;
    switch_bottom_state = INACTIVE;
  }
}
#pragma endregion

// This code makes me want tpo cry - Tyerone