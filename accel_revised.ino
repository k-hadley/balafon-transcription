// Full-balafon sensor data collection system
// By Katelyn Hadley and Andrew McPherson

/*** Instructions for use: ***
  Before uploading this code to the Arduino Due, input which keys are being used 
  into the "Accelerometer Wiring" section:
  - enter the sensor number corresponding to the lowest-numbered key being recorded
  - enter how many total keys will be used, including the lowest-numbered key and continuing upward

  Next, upload the program to the Arduino Due's Programming Port 
  (the micro USB plug closest to the power jack).

  Once finished uploading, switch the plug to the Native USB Port 
  (closest to the reset button on the Arduino) to receive data to the computer.

  In Calibrate mode, if the LEDs are turning on too often or not often enough, 
  adjust the sensitivity of the strike detection in the "Mode Switch" section 
  by raising or lowering the "threshold" value.
*/

#include <SPI.h>
#include <new_Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// -------- Accelerometer Wiring --------
// ***** Note to user: keep this section up to date **********

// Define a continuous range of keys to enable by noting the lowest key number (0 through 23) 
// and number of subsequent keys to turn on
const uint8_t lowest_key = 1;
#define NUM_KEYS 19  // Total number of sequential keys to record, starting from and including the lowest-numbered key in the set

// ----------- Mode Switch ----------------------
const int threshold = 5000;
#define SWITCH_PIN 7
#define SWITCH_LED 8
int strike_size = 0;
volatile uint8_t flip = 0;
uint8_t data_switch = 0;

// ----------- SPI setup -------------------
#define CS_BASE_PIN 30 // Pin 30 defined on PCB as accelerometer number 0 - DO NOT CHANGE
uint8_t cs_pins[NUM_KEYS] = {0};
const uint32_t spi_clock = 5000000;
Adafruit_LIS3DH accels[NUM_KEYS];

// -------- debugging on oscilloscope -----------
#define DEBUG 0
/* Debug levels: 
   0 = data only (running mode);
   1 = flag pins and data;
   2 = print setup messages, data, flag pins;
   3 = no data, only LED messages and flag pins;
   4 = only mic data and flag pins
*/
#define FLAG_PIN1 10
#define FLAG_PIN2 11
#define FLAG_PIN3 12

// --------- For data printing -----------------
#define USB_BUFFER_LENGTH 2048
char usbBuffer[USB_BUFFER_LENGTH] = {0};
unsigned int usbBufferPointer = 0;

// ---------- Data timer ---------------------
unsigned timestamp = 0;
unsigned first_timestamp = 0;

// ------ Variables for getting data ------------
volatile uint8_t sensor_num = 0;
sensors_event_t event;

// ------ Microphone (for audio sync) -----------
const int microphonePin = A0;
int mic_signal = 0;

// --- LED Matrix (pin definitions are permanent on PCB - don't change) ---
uint8_t active_led = 0;

const int row_0 = 2;
const int row_1 = 3;
const int row_2 = 4;
const int column_0 = A3;
const int column_1 = A4;
const int column_2 = A5;
const int column_3 = A6;
const int column_4 = A7;
const int column_5 = A8;
const int column_6 = A9;
const int column_7 = A10;

// ----------- Functions ------------------------
void lis3dh_setup(Adafruit_LIS3DH *any_sensor, uint8_t i);
void print_data_csv(unsigned time, uint8_t sensor, int x, int y, int z, int audio);
void led_on(int led);
void led_off(int led);
void reset_leds(void);
void led_init(void);


void setup(void) {
  // Initialise debugging pins
  pinMode(FLAG_PIN1, OUTPUT);
  pinMode(FLAG_PIN2, OUTPUT);
  pinMode(FLAG_PIN3, OUTPUT);

  pinMode(microphonePin, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(SWITCH_LED, OUTPUT);

  // Initialize Native USB port. Remember to switch to this plug after uploading sketch. Baud rate doesn't matter according to documentation
  SerialUSB.begin(2000000);

  while (!SerialUSB) delay(10);  // pause until serial console opens

  if (DEBUG>1) SerialUSB.println("LIS3DH test!");

  // Initialise accelerometers
  for(unsigned int i = 0; i < NUM_KEYS; i++) {
    cs_pins[i] = lowest_key + i;
    accels[i] = Adafruit_LIS3DH(CS_BASE_PIN + cs_pins[i], &SPI, spi_clock);
    lis3dh_setup(&accels[i], i);
  }

  // Set up data mode switch for enabling/disabling calibration with LEDs
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switch_flip, CHANGE);
  data_switch = digitalRead(SWITCH_PIN);
  digitalWrite(SWITCH_LED, data_switch);

  if (DEBUG>1) {
    SerialUSB.print("Current mode: ");
    SerialUSB.print(data_switch);
    SerialUSB.println(", where 0 = fast data collection and 1 = calibration with LEDs");
  }

  if (DEBUG<3) SerialUSB.println("Timestamp, Sensor number, X acceleration, Y acceleration, Z acceleration, Mic signal");

  // Turn on all LEDs to check that the program is running
  led_init();
  delay(1000);
  reset_leds();

  digitalWrite(FLAG_PIN1, LOW); // setup is complete

  first_timestamp = micros();
}


void loop() {  
  // Cycle through checking each accelerometer

  timestamp = micros();

  // get a new sensor event, normalized (units m/s)
  if (DEBUG) digitalWrite(FLAG_PIN1, HIGH);
  accels[sensor_num].read();
  if (DEBUG) digitalWrite(FLAG_PIN1, LOW);

  mic_signal = analogRead(microphonePin);

  // Check whether the calibration/data mode switch has been flipped
  if (flip) {
    flip = 0;
    data_switch = digitalRead(SWITCH_PIN);
    if (DEBUG > 1) {
      SerialUSB.print("New mode: ");
      SerialUSB.println(data_switch);
    }

    if (!data_switch) {
      // when exiting calibration mode, turn off the currently lit LED
      led_off(active_led);
    }
    
    digitalWrite(SWITCH_LED, data_switch);
  }
  

  // If switched, to Calibrate Mode, turn on the LED corresponding to the most recently struck key
  if (data_switch) {
    strike_size = sqrt((accels[sensor_num].x)*(accels[sensor_num].x) + (accels[sensor_num].y)*(accels[sensor_num].y) + (accels[sensor_num].z)*(accels[sensor_num].z)); // subtract g, mind units

    if (strike_size > threshold) {
      // Switch LED from previous to current key strike
      led_off(active_led);
      active_led = cs_pins[sensor_num];
      led_on(active_led);
    }
  }

  // Send data over serial
  if (DEBUG) digitalWrite(FLAG_PIN2, HIGH);
  if (DEBUG < 3) print_data_csv(timestamp-first_timestamp, cs_pins[sensor_num], accels[sensor_num].x, accels[sensor_num].y, accels[sensor_num].z, mic_signal);
  else if (DEBUG == 4) {
    SerialUSB.println(mic_signal);
    delay(2);
  }
  if (DEBUG) digitalWrite(FLAG_PIN2, LOW);

  if(++sensor_num >= NUM_KEYS)
    sensor_num = 0;

}

// --------- Functions -----------

// Setup routine for each accelerometer
void lis3dh_setup(Adafruit_LIS3DH *any_sensor, uint8_t i) {
  if (DEBUG>1) {
    SerialUSB.print("initializing sensor "); SerialUSB.print(i+1); SerialUSB.print(" of "); SerialUSB.println(NUM_KEYS);
  }

  if (!any_sensor->begin(0x18)) { 
    SerialUSB.print("Error: Couldn't start sensor "); SerialUSB.print(i+1); SerialUSB.print(" of "); SerialUSB.println(NUM_KEYS);
  } 
  else {
    if (DEBUG>1) SerialUSB.println("LIS3DH found!");

    any_sensor->setRange(LIS3DH_RANGE_16_G);  // 2, 4, 8 or 16 G
    any_sensor->setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); // options: 1,10,25,50,100,200,400,LOWPOWER_1K6HZ,LOWPOWER_5KHZ
  }
}

// Store CSV-formatted data in a buffer, and send buffer over Native USB when full
void print_data_csv(unsigned time, uint8_t sensor, int x, int y, int z, int audio) {
  const unsigned int maxStringLength = 64;
  
  // Print the data into the buffer at the current location
  int printLength = snprintf(&usbBuffer[usbBufferPointer], maxStringLength, "%x,%x,%x,%x,%x,%x\r\n", time, sensor, x, y, z, audio);

  // Work out how many characters were actually printed
  if(printLength < 0)
    printLength = 0;
  if(printLength > maxStringLength)
    printLength = maxStringLength;
  
  // If there isn't enough space for the next print, then flush the buffer to the USB port
  usbBufferPointer += printLength;
  if(USB_BUFFER_LENGTH - usbBufferPointer < maxStringLength + 1) {
    // (the +1 is to allow for the trailing '\0')
    if (DEBUG) digitalWrite(FLAG_PIN3, HIGH);
    SerialUSB.print(usbBuffer);
    if (DEBUG) digitalWrite(FLAG_PIN3, LOW);
    usbBufferPointer = 0;
  }
}

// Turn on a single LED corresponding to a key
void led_on(int led) {
  if (DEBUG==3) {
    SerialUSB.print("turning on led "); SerialUSB.println(led);
  }
  
  if (led <= 7) {
    digitalWrite(row_0, HIGH);
  } else if (led <= 15) {
    digitalWrite(row_1, HIGH);
  } else {
    digitalWrite(row_2, HIGH);
  }

  switch (led % 8) {
    case 0:
      digitalWrite(column_0, LOW);
      break;
    case 1:
      digitalWrite(column_1, LOW);
      break;
    case 2:
      digitalWrite(column_2, LOW);
      break;
    case 3:
      digitalWrite(column_3, LOW);
      break;
    case 4:
      digitalWrite(column_4, LOW);
      break;
    case 5:
      digitalWrite(column_5, LOW);
      break;
    case 6:
      digitalWrite(column_6, LOW);
      break;
    case 7:
      digitalWrite(column_7, LOW);
      break;
    default:
      SerialUSB.println("Invalid LED number, could not turn on");
      break;
  }
}

// Turn off a single LED corresponding to a key
void led_off(int led) {
  if (DEBUG==3) {
    SerialUSB.print("turning off led "); SerialUSB.println(led);
  }

  if (led <= 7) {
    digitalWrite(row_0, LOW);
  } else if (led <= 15) {
    digitalWrite(row_1, LOW);
  } else {
    digitalWrite(row_2, LOW);
  }

  switch (led % 8) {
    case 0:
      digitalWrite(column_0, HIGH);
      break;
    case 1:
      digitalWrite(column_1, HIGH);
      break;
    case 2:
      digitalWrite(column_2, HIGH);
      break;
    case 3:
      digitalWrite(column_3, HIGH);
      break;
    case 4:
      digitalWrite(column_4, HIGH);
      break;
    case 5:
      digitalWrite(column_5, HIGH);
      break;
    case 6:
      digitalWrite(column_6, HIGH);
      break;
    case 7:
      digitalWrite(column_7, HIGH);
      break;
    default:
      SerialUSB.println("Invalid LED number, could not turn off");
      break;
  }
}

// Turn off all LEDs at once
void reset_leds(void) {
  digitalWrite(row_0, LOW);
  digitalWrite(row_1, LOW);
  digitalWrite(row_2, LOW);
  digitalWrite(column_0, HIGH);
  digitalWrite(column_1, HIGH);
  digitalWrite(column_2, HIGH);
  digitalWrite(column_3, HIGH);
  digitalWrite(column_4, HIGH);
  digitalWrite(column_5, HIGH);
  digitalWrite(column_6, HIGH);
  digitalWrite(column_7, HIGH);
}

// Initialize and turn on all LEDs as a test
void led_init(void) {
  pinMode(row_0, OUTPUT);
  pinMode(row_1, OUTPUT);
  pinMode(row_2, OUTPUT);
  pinMode(column_0, OUTPUT);
  pinMode(column_1, OUTPUT);
  pinMode(column_2, OUTPUT);
  pinMode(column_3, OUTPUT);
  pinMode(column_4, OUTPUT);
  pinMode(column_5, OUTPUT);
  pinMode(column_6, OUTPUT);
  pinMode(column_7, OUTPUT);
  
  digitalWrite(row_0, HIGH);
  digitalWrite(row_1, HIGH);
  digitalWrite(row_2, HIGH);
  digitalWrite(column_0, LOW);
  digitalWrite(column_1, LOW);
  digitalWrite(column_2, LOW);
  digitalWrite(column_3, LOW);
  digitalWrite(column_4, LOW);
  digitalWrite(column_5, LOW);
  digitalWrite(column_6, LOW);
  digitalWrite(column_7, LOW);
}

// Raise an interrupt flag when the mode switch is flipped
void switch_flip() {
  flip = 1;
}
