// Connecting multiple accelerometers to arduino and testing data transfer rates
// Revised from accel_PrintUSBBuffer_test4, removed accelerometer interrupts

// *** Instructions for use: ***
/* Before uploading this code to the Arduino Due, input which keys are being used 
  into the "Accelerometer Wiring" section.

  For a continuous set of keys with no gaps: 
  - set CS_MODE to 0
  - enter how many total keys will be used, starting from the lowest key and continuing upward
  - enter the sensor number corresponding to the lowest key being recorded

  For a selected mix of keys which may contain gaps:
  - set CS_MODE to 1
  - enter the total number of keys in the set
  - enter the sensor numbers corresponding to all keys to be recorded

Next, upload the program to the Arduino Due's Programming Port 
(the micro USB plug closest to the power jack).

Once finished uploading, switch to the Native USB Port 
(closest to the reset button on the Arduino) to receive data.
*/

#include <SPI.h>
#include <new_Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// -------- Accelerometer Wiring --------
// **** Keep this section up to date! **********

// ACCELEROMETER INPUT OPTIONS: define CS_MODE corresponding to which option is in use
#define CS_MODE 0  // Mode 0 or Mode 1
#define NUM_KEYS 8  // Total number of keys to record 

// Mode 0: Define a continuous range of keys to enable by noting the lowest key number (0 through 23) and number of subsequent keys to turn on (NUM_KEYS above)
const uint8_t lowest_key = 0;

// Mode 1: Define a non-continuous range of keys by listing all key numbers, and total number of keys (above)
uint8_t cs_pins[NUM_KEYS] = {2,3,4,5, 17,18,19,20};

// ----------- SPI setup -------------------
#define CS_BASE_PIN 30 // Pin 30 defined on PCB as accelerometer number 0
const uint32_t spi_clock = 5000000;
Adafruit_LIS3DH accels[NUM_KEYS];

// -------- debugging on oscilloscope -----------
#define DEBUG 0  // Debug levels: 0 = nothing, 1 = flag pins, 2 = print messages and flag pins

#define FLAG_PIN1 10
#define FLAG_PIN2 11
#define FLAG_PIN3 12

// --------- For data printing -----------------
#define USB_BUFFER_LENGTH 2048
char usbBuffer[USB_BUFFER_LENGTH] = {0};
unsigned int usbBufferPointer = 0;

// ---------- Loop timing ---------------------
unsigned timestamp = 0;
unsigned first_timestamp = 0;
#define RUNTIME 0xFFFFFFFF // microseconds of total data collection

// ----------- Functions ------------------------

void lis3dh_setup(Adafruit_LIS3DH *any_sensor, uint8_t i);
void print_data_csv(unsigned time, uint8_t sensor, float x, float y, float z);

// ------ Variables for getting data ------------
volatile uint8_t sensor_num = 0;
sensors_event_t event;


void setup(void) {
  // Initialise debugging pins
  pinMode(FLAG_PIN1, OUTPUT);
  pinMode(FLAG_PIN2, OUTPUT);
  pinMode(FLAG_PIN3, OUTPUT);

  SerialUSB.begin(2000000);    // Initialize Native USB port. Remember to switch to this plug after uploading sketch. Baud rate doesn't matter according to documentation
  while (!SerialUSB) delay(10);  // pause until serial console opens

  if (DEBUG>1) SerialUSB.println("LIS3DH test!");

  // Initialise accelerometers
  for(unsigned int i = 0; i < NUM_KEYS; i++) {
    if (!CS_MODE) {
      cs_pins[i] = lowest_key + i;
    }
    accels[i] = Adafruit_LIS3DH(CS_BASE_PIN + cs_pins[i], &SPI, spi_clock);
    lis3dh_setup(&accels[i], i);
  }

  if (DEBUG>1) SerialUSB.println("Setup complete");
  digitalWrite(FLAG_PIN1, LOW);

  SerialUSB.println("Timestamp, Sensor number, X acceleration, Y acceleration, Z acceleration");

  first_timestamp = micros();
}

void loop() {  
  timestamp = micros();

  if (timestamp - first_timestamp < RUNTIME) { // auto-shutoff if running for too long

    // Cycle through checking each accelerometer

    // get a new sensor event, normalized (units m/s)
    if (DEBUG) digitalWrite(FLAG_PIN1, HIGH);
    accels[sensor_num].read();
    if (DEBUG) digitalWrite(FLAG_PIN1, LOW);

    // Send data over serial
    if (DEBUG) digitalWrite(FLAG_PIN2, HIGH);
    print_data_csv(timestamp-first_timestamp, cs_pins[sensor_num], accels[sensor_num].x, accels[sensor_num].y, accels[sensor_num].z);
    if (DEBUG) digitalWrite(FLAG_PIN2, LOW);

    if(++sensor_num >= NUM_KEYS)
      sensor_num = 0;

  }
}

void lis3dh_setup(Adafruit_LIS3DH *any_sensor, uint8_t i) {// Setup routine for each accelerometer
  if (DEBUG>1) {
    SerialUSB.print("initializing sensor "); SerialUSB.print(i+1); SerialUSB.print(" of "); SerialUSB.println(NUM_KEYS);
  }

  if (!any_sensor->begin(0x18)) { 
    SerialUSB.print("Couldn't start sensor "); SerialUSB.print(i+1); SerialUSB.print(" of "); SerialUSB.println(NUM_KEYS);
  } 
  else {
    if (DEBUG>1) SerialUSB.println("LIS3DH found!");

    any_sensor->setRange(LIS3DH_RANGE_16_G);  // 2, 4, 8 or 16 G
    any_sensor->setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); // options: 1,10,25,50,100,200,400,LOWPOWER_1K6HZ,LOWPOWER_5KHZ
  }
}

void print_data_csv(unsigned time, uint8_t sensor, int x, int y, int z) {
  const unsigned int maxStringLength = 64;
  
  // Print the data into the buffer at the current location
  int printLength = snprintf(&usbBuffer[usbBufferPointer], maxStringLength, "%x,%x,%x,%x,%x\r\n", time, sensor, x, y, z);

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
