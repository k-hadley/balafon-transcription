// Full-balafon sensor data collection system
// By Katelyn Hadley and Andrew McPherson

/*** Instructions for use: ***
  Optional setup before uploading: 
  - Enter the number of balafon keys and available breakout board ports 
  in the "Accelerometer Wiring" section below to increase the program's efficiency.
  - After testing Calibrate mode, if the LEDs are turning on too often or not often enough, 
  adjust the sensitivity of the strike detection in the "Mode Switch" section 
  by raising or lowering the "THRESHOLD" value.

  Quick Start:
  1. Upload the program to the Arduino Due's Programming Port 
     (the micro USB plug closest to the power jack).
  2. Once finished uploading, switch the plug to the Native USB Port 
     (closest to the reset button on the Arduino) to receive data to the computer.
  
*/

#include <SPI.h>
#include <new_Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// -------- Accelerometer Wiring --------
// Optional: Increase efficiency of the program (decrease memory usage of variables) 
// by entering the number of balafon keys that have a sensor attached

#define NUM_KEYS 24  // Total number of keys to record (default = 24)
#define MAX_KEYS 24  // Maximum number of ports (8 * number of breakout boards; default = 24)

// ----------- Mode Switch ----------------
#define THRESHOLD 5000 // Optional: adjust sensitivity of the strike detection in Calibrate mode
int strike_size = 0;   // For calculating whether the acceleration of a keystrike was above the threshold

#define SWITCH_PIN 7  // Defined on PCB - Do not change
#define SWITCH_LED 8  // Defined on PCB - Do not change
volatile uint8_t flip = 0; // Interrupt flag for checking if Mode Switch was flipped
uint8_t data_switch = 0;   // Current status of  Mode Switch

// ----------- SPI setup -------------------
const uint32_t spi_clock = 5000000;
#define CS_BASE_PIN 30 // Arduino Pin 30 is defined on PCB as Chip Select (CS) port number 0 - Do not change

// Variables to store the port numbers & connection info of all successfully connected accelerometers
uint8_t cs_portnums[NUM_KEYS] = {0}; 
Adafruit_LIS3DH accels[NUM_KEYS];
volatile uint8_t connected_keys = 0; // total number of keys that have successfully connected to Arduino so far

// --------- Data printing -----------------
#define USB_BUFFER_LENGTH 2048
char usbBuffer[USB_BUFFER_LENGTH] = {0}; // Data buffer: for sending a large packet of sensor data in each Serial USB transmission
unsigned int usbBufferPointer = 0;
uint8_t serialusb_started = 0;  // Current status of Serial USB connection (initialized or not)

// --------- Collecting data -----------------
unsigned timestamp = 0;

// Accelerometer polling
volatile uint8_t sensor_num = 0; // Only used for looping through array of sensors - value does not necessarily correspond to key number
sensors_event_t event;

// Microphone (for audio sync)
const int microphonePin = A0;
int mic_signal = 0;

// --------- LED Matrix  --------------
uint8_t active_led = 0;

// These pin definitions are permanent on the Arduino Shield PCB - Do not change
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
uint8_t lis3dh_setup(Adafruit_LIS3DH *any_sensor);
void print_data_csv(unsigned time, uint8_t sensor, int x, int y, int z, int audio);
void led_on(int led);
void led_off(int led);
void reset_leds(void);
void led_init(void);


void setup(void) {
  // Initialize Arduino shield pins
  pinMode(microphonePin, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(SWITCH_LED, OUTPUT);

  // Initialize accelerometers
  Adafruit_LIS3DH accel_loop;
  for(unsigned int i = 0; ((i < MAX_KEYS) && (connected_keys < NUM_KEYS)); i++) {
    SerialUSB.print(i);
    accel_loop = Adafruit_LIS3DH(CS_BASE_PIN + i, &SPI, spi_clock);
    
    // Attempt to connect to each sensor. If it successfully connects, save its connection info to get data later
    if (lis3dh_setup(&accel_loop)) { 
      cs_portnums[connected_keys] = i;
      accels[connected_keys] = accel_loop;
      connected_keys++;
    }
  }

  // Set up data mode switch for enabling/disabling calibration with LEDs
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switch_flip, CHANGE);
  data_switch = digitalRead(SWITCH_PIN);
  digitalWrite(SWITCH_LED, data_switch);

  // Turn on all LEDs to show that the program is running
  led_init();
  delay(1000);
  reset_leds();
}


void loop() { 
  // ---------- SET MODE ------------

  // Check whether the calibration/data mode switch has been flipped
  if (flip) {
    flip = 0;
    reset_leds();
    data_switch = digitalRead(SWITCH_PIN);
    digitalWrite(SWITCH_LED, data_switch);
  }

  // ---------- CONNECT USB ----------

  // When entering Data Mode for the first time, start Native USB transmission.
  if (!serialusb_started && !data_switch) {
    SerialUSB.begin(2000000); // Baud rate doesn't matter according to Native USB documentation

    while (!flip && !SerialUSB) delay(10);  // pause until serial console opens, unless switched back to Calibrate mode
    
    if (!flip && !serialusb_started && SerialUSB) { // Print connection info the first time the USB connection starts
      SerialUSB.println("\n\n Initializing Balafon Transcription Data Collection System. If not printing sensor info within 3 seconds, press the Arduino's Reset button or unplug from computer and try again.\n");
      
      SerialUSB.print("Connected to sensor ports:");
      for(uint8_t i = 0; i < connected_keys; i++) {
        SerialUSB.print("  ");
        SerialUSB.print(cs_portnums[i]);
      }

      SerialUSB.println("\nReady to transmit data \n");
      SerialUSB.println("\nTimestamp, Sensor port number, X acceleration, Y acceleration, Z acceleration, Mic signal");

      serialusb_started = 1;
      delay(2000); // Make the message visible onscreen
    } else { // If the switch was flipped while waiting, continue on in the correct mode
      data_switch = digitalRead(SWITCH_PIN);
    }
  }

  // --------- PROCESS DATA ------------ 

  // Cycle through checking each accelerometer
  timestamp = micros();

  accels[sensor_num].read();  // get a new sensor event, normalized
  mic_signal = analogRead(microphonePin); // sample the microphone on the Arduino shield

  if (data_switch) { 
    // In Calibrate Mode, calculate the magnitude of acceleration to later determine whether a key is vibrating
    strike_size = sqrt((accels[sensor_num].x)*(accels[sensor_num].x) + (accels[sensor_num].y)*(accels[sensor_num].y) + (accels[sensor_num].z)*(accels[sensor_num].z));

  } else if (serialusb_started) { 
    // In Data Mode, send data over serial
    print_data_csv(timestamp, cs_portnums[sensor_num], accels[sensor_num].x, accels[sensor_num].y, accels[sensor_num].z, mic_signal);
  }

  // -------- SET LIGHTS ------------

  // In Data Mode: Turn on LED of each sensor being checked to visually see which ones are collecting data
  // In Calibrate Mode: Turn on the LED corresponding to the most recently struck key
  if (!data_switch || (data_switch && (strike_size > THRESHOLD))) {
    // Switch LED from previous to current key strike
    led_off(active_led);
    active_led = cs_portnums[sensor_num];
    led_on(active_led);
  }

  // Increment to poll the next sensor
  if(++sensor_num >= connected_keys)
    sensor_num = 0;

}

// -------------- FUNCTIONS -----------------

// Setup routine for each accelerometer
uint8_t lis3dh_setup(Adafruit_LIS3DH *any_sensor) {
  // Try setting up the sensor connected to a particular CS pin
  if (!any_sensor->begin(0x18)) { 
    // Sensor does not connect
    return 0;
  } else {
    // Sensor does connect: set to maximum data range and sample rate
    any_sensor->setRange(LIS3DH_RANGE_16_G);  // 2, 4, 8 or 16 G
    any_sensor->setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); // options: 1,10,25,50,100,200,400,LOWPOWER_1K6HZ,LOWPOWER_5KHZ
    return 1;
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
    SerialUSB.print(usbBuffer);
    usbBufferPointer = 0;
  }
}

// Turn on a single LED corresponding to a key
void led_on(int led) {
  // Rows of LED matrix (each corresponding to one CS cable on Arduino Shield PCB)
  if (led <= 7) {
    digitalWrite(row_0, HIGH);
  } else if (led <= 15) {
    digitalWrite(row_1, HIGH);
  } else {
    digitalWrite(row_2, HIGH);
  }

  // Columns of LED matrix
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
  // Rows of LED matrix (each corresponding to one CS cable on Arduino Shield PCB)
  if (led <= 7) {
    digitalWrite(row_0, LOW);
  } else if (led <= 15) {
    digitalWrite(row_1, LOW);
  } else {
    digitalWrite(row_2, LOW);
  }

  // Columns of LED matrix
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

// Raise an interrupt flag when the Mode switch is flipped
void switch_flip() {
  flip = 1;
}
