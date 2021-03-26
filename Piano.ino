#include <U8g2lib.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

// Pin definitions
// Row select and enable
const int RA0_PIN = D3, RA1_PIN = D6, RA2_PIN = D12, REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2, C1_PIN = D9, C2_PIN = A6, C3_PIN = D1, OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4, OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0, JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3, DRST_BIT = 4, HKOW_BIT = 5, HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Global varible declaration
const int num[] = {1, 2, 4, 8};
const String keys[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", " "};
const uint32_t stepSizes[] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007187, 96418756, 0}; //Shared  
const uint32_t sinStep[] = {131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 0}; //Shared

// Look up table for sine and cosine
uint32_t sinLUT[11000];
float cosLUT[330];

// Data regarding current pressed key
uint8_t currentKey;                 //Shared
volatile uint32_t currentStepSize, keyState;  //Shared
volatile uint8_t keyArray[7]; //Shared
SemaphoreHandle_t keyArrayMutex;

// Data for ADSR
volatile uint32_t A[] = {110, 3300, 6600};
volatile uint32_t D[] = {22000, 660, 330};
volatile float S[] = {0.1, 0.8, 1};
volatile uint32_t R[] = {440, 3300, 4400};
volatile float step1[3], step2[3], step4[3];

// Varible for knobs 1, 2, 3 and joystick axis x and y
volatile int rotation3 = 16, rotation2 = 4, rotation1 = 0;  //Shared
volatile int joystick1 = 512, joystick2 = 512;               

// Serial output data
volatile char noteMessage[]= "xxx";                 
const char intToHex[] = "0123456789ABCDEF";
QueueHandle_t msgOutQ;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

//Function to set outputs via matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Initialise UART
  Serial.begin(115200);
  Serial.println("Hello World");

  // Calculate adsr step value
  for (int i = 0; i < 3; i++) {
    step1[i] = 1.0 / A[i];
    step2[i] = (1 - S[i]) / D[i];
    step4[i] = S[i] / R[i];
  }

  // Generate sine and cos LUT
  for (int i = 0; i < 11000; i++) {
    sinLUT[i] = (sin(i * 0.00057119866) + 1) * 2147483648;
  }
  for (int i = 0; i < 110; i++) {
    cosLUT[i] = (sin(i * 0.05712)+1) * 0.005;
  }
  
  // Set pin directions
  pinMode(RA0_PIN, OUTPUT); pinMode(RA1_PIN, OUTPUT); pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT); pinMode(OUTL_PIN, OUTPUT); pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(C0_PIN, INPUT); pinMode(C1_PIN, INPUT); pinMode(C2_PIN, INPUT); pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT); pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  // Mutex
  keyArrayMutex= xSemaphoreCreateMutex();

  // Queue
  msgOutQ = xQueueCreate(8, 4);

  // Setup Timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer= new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Thread - scanKeys
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask, "scanKeys", 128, NULL, 3, &scanKeysHandle);

  //Thread - displayUpdate
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(displayUpdateTask, "displayUpdate",512, NULL, 1, &displayUpdateHandle);

  //Thread - msgOutTask
  TaskHandle_t msgOutTaskHandle = NULL;
  xTaskCreate(msgOutTask, "msgOut", 32, NULL, 2, &msgOutTaskHandle);

  //Thread - msgInTask
  TaskHandle_t msgInTaskHandle = NULL;
  xTaskCreate(msgInTask, "msgIn", 32, NULL, 5, &msgInTaskHandle);

  //Thread - joystickTask
  TaskHandle_t joystickTaskHandle = NULL;
  xTaskCreate(joystickTask, "joystick", 256, NULL, 4, &joystickTaskHandle);

  vTaskStartScheduler();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t readCols() {
  uint8_t output = 0;
  if (digitalRead(C0_PIN)) output += 0x01;
  if (digitalRead(C1_PIN)) output += 0x02;
  if (digitalRead(C2_PIN)) output += 0x04;
  if (digitalRead(C3_PIN)) output += 0x08;
  return output;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setRow(const uint8_t rowIdx) {
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN,HIGH);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Variable used in sampleISR
uint32_t phaseAcc = 0;
uint32_t vib_step = 0;
uint32_t ctr = 0;
uint32_t ctr2 = 0;
float envolope = 0;
float scale = 0;
uint8_t control = 0;         //Shared
int vibrato_freq = 10;       //Shared
int vibrato_amp = 0;         //Shared
int tremolo_freq = 0;        //Shared
int tremolo_amp = 1;         //Shared

void sampleISR() {
  // reset value when new key is pressed
  if (control == 0) {
    envolope = 0;
    scale = 0;
    control = 1;
  } else if (control == 2) {
    scale = 1;
    control = 1;
  }
  
  // key is pressed - start ADS
  if (keyState == 1) {
    if (ctr < A[rotation1]) {
      envolope += step1[rotation1];
    } else if (ctr >= A[rotation1] && ctr < (A[rotation1] + D[rotation1])) {
      envolope -= step2[rotation1];
    }
  // key is released - start R
  } else {                      
    if (scale > 0) {
      envolope -= step4[rotation1];
    } else {
      envolope = 0;
      scale = 0;
    }
  }
  ctr++;

  // Vibrato and Tremolo calculation
  vib_step = currentStepSize * (1 + vibrato_amp * cosLUT[int(ctr2 / vibrato_freq)]);
  phaseAcc += vib_step;
  scale = envolope * (1 - tremolo_amp * 100 * cosLUT[int(ctr2 / tremolo_freq)]);
  ctr2++;
  if (ctr2 >= 3300) ctr2 = 0;
  
  // Output wave generation
  // sine wave
  if (rotation1 == 0) {
    if (phaseAcc >= 11000) phaseAcc = 0;
    analogWrite(OUTR_PIN, (int(scale * (sinLUT[phaseAcc] >> 24))) >> (8 - rotation3 / 2));
  // triangle wave
  } else if (rotation1 == 1) {
    analogWrite(OUTR_PIN, (int(scale * ( phaseAcc >> 24))) >> (8 - rotation3 / 2));
  // square wave
  } else {
    if (phaseAcc < 536870912) {
      analogWrite(OUTR_PIN, (int(scale * 255)) >> (8 - rotation3 / 2));
    } else {
      analogWrite(OUTR_PIN, 0);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {}                        // There is nothing in the loop function

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();

  int ror1, ror2, ror3;
  uint8_t pre1 = 22, pre2 = 22, pre3 = 22;
  uint8_t cur1, cur2, cur3;
  bool dir1 = 1, dir2 = 1, dir3 = 1;
  uint8_t posn1 = 0, posn2 = 8;
  int preKey = 12;
  int octave;
  int stepSize = 0;
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Scan the entire key array
    for (int i=0; i<=4; i++) {
      setRow(i);
      delayMicroseconds(3);
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      keyArray[i] = readCols();
      xSemaphoreGive(keyArrayMutex);
    }

    // Detect which piano key is pressed (12 indicates no key pressed)
    __atomic_store_n(&currentKey, 12, __ATOMIC_RELAXED);
    for (int i=0; i<=2; i++) {
      for (int j=0; j<=3; j++) {
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        if ((keyArray[i] & num[j]) == 0x00) __atomic_store_n(&currentKey, i*4+j, __ATOMIC_RELAXED);
        xSemaphoreGive(keyArrayMutex);
      }
    }

    // Detect a change in key
    if (currentKey != preKey) {     // if a key is pressed or released
      if (currentKey < 12) {        // press DOWN
        __atomic_store_n(&keyState, 1, __ATOMIC_RELAXED);
        __atomic_store_n(&ctr, 0, __ATOMIC_RELAXED);
        __atomic_store_n(&control, 0, __ATOMIC_RELAXED);
        // Generate serial output message
        noteMessage[0] = 'P'; noteMessage[1] = (rotation2 + 48); noteMessage[2] = intToHex[currentKey];
        if (rotation1 == 0) {
          stepSize = sinStep[currentKey];
        } else {
          stepSize = stepSizes[currentKey];
        }
      } else {                      // press RELEASE
        __atomic_store_n(&keyState, 0, __ATOMIC_RELAXED);
        __atomic_store_n(&ctr, 0, __ATOMIC_RELAXED);
        // Generate serial output message
        noteMessage[0] = 'R'; noteMessage[1] = (rotation2 + 48); noteMessage[2] = intToHex[preKey];
      }
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);

      // Update tone values
      octave = rotation2 - 4;
      if (octave > 0) {
        __atomic_store_n(&currentStepSize, stepSize << octave, __ATOMIC_RELAXED);
      } else {
        __atomic_store_n(&currentStepSize, stepSize >> (0 - octave), __ATOMIC_RELAXED);
      }
    }
    preKey = currentKey;

    // Volume Knob
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    cur3 = keyArray[3] & 0x03;
    xSemaphoreGive(keyArrayMutex);
    ror3 = (pre3<<2) + cur3;
    if (ror3 == 0x01 || ror3 == 0x07 || ror3 == 0x08 || ror3 == 0x0e) {
      __atomic_add_fetch(&rotation3, 1, __ATOMIC_RELAXED);
      dir3 = 1;
    } else if (ror3 == 0x02 || ror3 == 0x04 || ror3 == 0x0b || ror3 == 0x0d) {
      __atomic_add_fetch(&rotation3, -1, __ATOMIC_RELAXED);
      dir3 = -1;
    } else if (ror3 == 0x03 || ror3 == 0x06 || ror3 == 0x09 || ror3 == 0x0c) {
      __atomic_add_fetch(&rotation3, dir3, __ATOMIC_RELAXED);
    }
    if (rotation3 < 0) __atomic_store_n(&rotation3, 0, __ATOMIC_RELAXED);
    if (rotation3 > 16) __atomic_store_n(&rotation3, 16, __ATOMIC_RELAXED);
    pre3 = cur3;

    // Octave Knob
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    cur2 = keyArray[3] & 0x0c;
    xSemaphoreGive(keyArrayMutex);
    ror2 = (cur2>>2) + pre2;
    if (ror2 == 0x01 || ror2 == 0x07 || ror2 == 0x08 || ror2 == 0x0e) {
      posn2++;
      dir2 = 1;
    } else if (ror2 == 0x02 || ror2 == 0x04 || ror2 == 0x0b || ror2 == 0x0d) {
      posn2--;
      dir2 = -1;
    } else if (ror2 == 0x03 || ror2 == 0x06 || ror2 == 0x09 || ror2 == 0x0c) {
      posn2 += dir2;
    }
    if (posn2 < 2) posn2 = 2;
    if (posn2 > 16) posn2 = 16;
    pre2 = cur2;
    __atomic_store_n(&rotation2, posn2 / 2, __ATOMIC_RELAXED);

    // Instrement Knob
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    cur1 = keyArray[4] & 0x03;
    xSemaphoreGive(keyArrayMutex);
    ror1 = (pre1<<2) + cur1;
    if (ror1 == 0x01 || ror1 == 0x07 || ror1 == 0x08 || ror1 == 0x0e) {
      posn1++;
      dir1 = 1;
    } else if (ror1 == 0x02 || ror1 == 0x04 || ror1 == 0x0b || ror1 == 0x0d) {
      posn1--;
      dir1 = -1;
    } else if (ror1 == 0x03 || ror1 == 0x06 || ror1 == 0x09 || ror1 == 0x0c) {
      posn1 += dir1;
    }
    if (posn1 < 0) posn1 = 0;
    if (posn1 > 4) posn1 = 4;
    pre1 = cur1;
    __atomic_store_n(&rotation1, posn1 / 2, __ATOMIC_RELAXED);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void joystickTask(void * pvParameters) {
  const TickType_t xFrequency = 10/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Read joytick y axis and update vibrato value
    joystick1 = 512 - analogRead(JOYX_PIN);
    joystick1 = abs(joystick1) / 100;
    __atomic_store_n(&vibrato_amp, joystick1, __ATOMIC_RELAXED);
    __atomic_store_n(&vibrato_freq, 4 * joystick1, __ATOMIC_RELAXED);

    // Read joytick x axis and update tremolo value
    joystick2 = 512 - analogRead(JOYY_PIN);
    joystick2 = abs(joystick2) / 100;
    __atomic_store_n(&tremolo_freq, 10, __ATOMIC_RELAXED);
    __atomic_store_n(&tremolo_amp, 0.2 * joystick2, __ATOMIC_RELAXED);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 36/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    //Update display
    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
    
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

    // Key array
    u8g2.setCursor(2,10);
    u8g2.print(keyArray[0],HEX);
    u8g2.print(keyArray[1],HEX);
    u8g2.print(keyArray[2],HEX);

    // Note message
    u8g2.setCursor(2,20);
    u8g2.print((char*) noteMessage);

    // Current note being played
    u8g2.setCursor(2,30);
    u8g2.print("note:");
    u8g2.setCursor(35,30);
    u8g2.print(keys[currentKey]);

    // Volume
    u8g2.setCursor(60,10);
    u8g2.print("Volume:");
    u8g2.setCursor(115,10);
    u8g2.print(rotation3);

    // Octave
    u8g2.setCursor(69,20);
    u8g2.print("Octave:");
    u8g2.setCursor(115,20);
    u8g2.print(rotation2);

    // Musical Instrument
    u8g2.setCursor(90,30);
    if (rotation1 == 0) {
      u8g2.print("Piano");
    } else if (rotation1 == 1) {
      u8g2.print("Violin");
    } else {
      u8g2.print("Sax");
    }
    
    xSemaphoreGive(keyArrayMutex);
    
    u8g2.sendBuffer();                    // transfer internal memory to the display

    //Toggle LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void msgOutTask(void *pvParameters) {
  char outMsg[4];
  while(1) {
    xQueueReceive(msgOutQ, outMsg, portMAX_DELAY);
    Serial.println(outMsg);
  } 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void msgInTask(void *pvParameters) {
  const TickType_t xFrequency = 5/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();
  byte msg;
  int counter = 0;
  char inMsg[] = "xxx";
  String hexTable = "0123456789AB";
  int stepSize = 0;
  int localOctave = 4;
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    while(Serial.available() > 0) {
      msg = Serial.read();
      if (counter <= 3) inMsg[counter] = msg;
      counter++;
      if (msg == '\n') {          // Start to decode Msg and play tone, same in scanKeyTask
        counter = 0;
        if (inMsg[0] == 'R') {
          __atomic_store_n(&keyState, 0, __ATOMIC_RELAXED);
          __atomic_store_n(&ctr, 0, __ATOMIC_RELAXED);
        } else if (inMsg[0] == 'P') {
          __atomic_store_n(&keyState, 1, __ATOMIC_RELAXED);
          __atomic_store_n(&ctr, 0, __ATOMIC_RELAXED);
          __atomic_store_n(&control, 0, __ATOMIC_RELAXED);
          if (rotation1 == 0) {
            stepSize = sinStep[hexTable.indexOf(inMsg[2])];
          } else {
            stepSize = stepSizes[hexTable.indexOf(inMsg[2])];
          }
        }
        localOctave = hexTable.indexOf(inMsg[1]) - 4;
        if (localOctave > 0) {
          __atomic_store_n(&currentStepSize, stepSize << localOctave, __ATOMIC_RELAXED);
        } else {
          localOctave = 0 - localOctave;
          __atomic_store_n(&currentStepSize, stepSize >> localOctave, __ATOMIC_RELAXED);
        }
      }
    }
  }
}
