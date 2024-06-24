#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>


Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);


int latchPin = 17;      // Latch pin of 74HC595 is connected to Digital pin 17
int clockPin = 18;      // Clock pin of 74HC595 is connected to Digital pin 18 SRCLK
int dataPin = 16;       // Data pin of 74HC595 is connected to Digital pin 16 SER

byte leds = 0;         // Variable to hold the pattern of which LEDs are currently turned on or off


const int NUM_MIDI_NOTES = 8; // Define the number of MIDI notes
byte midiNotes[NUM_MIDI_NOTES] = {68, 69, 70, 71,72,73,74,75}; // Define the MIDI notes
byte buttonNumbers[NUM_MIDI_NOTES] = {1, 2, 3, 4,5,6,7,8}; // Define the corresponding button numbers

/*
 * setup() - this function runs once when you turn your Arduino on
 * We initialize the serial connection with the computer
 */
void setup() 
{
  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif

  usb_midi.begin();
  MIDI_USB.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB.setHandleNoteOn(handleNoteOn);
  // Do the same for MIDI Note Off messages.
  MIDI_USB.setHandleNoteOff(handleNoteOff);

  Serial.begin(115200);

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);

  leds = 0;        // Initially turns all the LEDs off, by giving the variable 'leds' the value 0
  updateShiftRegister();
  Serial.println("Ready");
}

/*
 * loop() - this function runs over and over again
 */
void loop() 
{
  // read any new MIDI messages
  MIDI_USB.read(); 
}

int getButtonNumber(byte midiNote) {
  for (int i = 0; i < NUM_MIDI_NOTES; i++) {
    if (midiNote == midiNotes[i]) {
      return buttonNumbers[i];
    }
  }
  return -1; // Return -1 if the MIDI note is not found in the mappings
}

void handleNoteOn(byte channel, byte note, byte velocity){
  // Check if note is in range
  Serial.print("Note On : ");
  Serial.print(note);
  int buttonNumber = getButtonNumber(note);
  if(buttonNumber > 0){
    bitSet(leds, buttonNumber);
    updateShiftRegister();
  }
}


void handleNoteOff(byte channel, byte note, byte velocity){
  // Check if note is in range
  Serial.print("Note Off : ");
  Serial.print(note);
  int buttonNumber = getButtonNumber(note);
  if(buttonNumber > 0){
    bitClear(leds, buttonNumber);
    updateShiftRegister();
  }
}
/*
 * updateShiftRegister() - This function sets the latchPin to low, then calls the Arduino function 'shiftOut' to shift out contents of variable 'leds' in the shift register before putting the 'latchPin' high again.
 */
void updateShiftRegister()
{
   digitalWrite(latchPin, LOW);
   shiftOut(dataPin, clockPin, MSBFIRST, leds);
   digitalWrite(latchPin, HIGH);
}
