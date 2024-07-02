

#include <usb_midi_host.h>
#include "pio_usb_configuration.h"
#include <Arduino.h>
#include <MIDI.h>
#include "usb_host_wrapper.h"
#include "pio_usb.h"
#define HOST_PIN_DP   16   // Pin used as D+ for host, D- = D+ + 1
#include "EZ_USB_MIDI_HOST.h"
// USB Host object
Adafruit_USBH_Host USBHost;
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);

int latchPin = 17;      // Latch pin of 74HC595 is connected to Digital pin 17
int clockPin = 18;      // Clock pin of 74HC595 is connected to Digital pin 18 SRCLK
int dataPin = 16;       // Data pin of 74HC595 is connected to Digital pin 16 SER

byte leds = 0;         // Variable to hold the pattern of which LEDs are currently turned on or off

void process_midi_data(uint8_t *data, uint32_t length);

static bool core0_booting = true;
static bool core1_booting = true;

USING_NAMESPACE_MIDI
USING_NAMESPACE_EZ_USB_MIDI_HOST

EZ_USB_MIDI_HOST<MidiHostSettingsDefault> myMidiHost;
EZ_USB_MIDI_HOST<MidiHostSettingsDefault>& midiHost = myMidiHost; // Initialize the reference


void onNoteOffHandle(byte channel, byte note, byte velocity){
    Serial.printf("C%u: Note off#%u v=%u\r\n", channel, note, velocity);
}

void onNoteOnHandle(byte channel, byte note, byte velocity){
    Serial.printf("C%u: Note on#%u v=%u\r\n", channel, note, velocity);
}

void onPolyphonicAftertouchHandle(byte channel, byte note, byte amount){
    Serial.printf("C%u: PAT#%u=%u\r\n", channel, note, amount);
}

void onControlChangeHandle(byte channel, byte controller, byte value){
    Serial.printf("C%u: CC#%u=%u\r\n", channel, controller, value);
}

void onProgramChangeHandle(byte channel, byte program){
    Serial.printf("C%u: Prog=%u\r\n", channel, program);
}

void onAftertouchHandle(byte channel, byte value){
    Serial.printf("C%u: AT=%u\r\n", channel, value);
}

void onPitchBendHandle(byte channel, int value){
    Serial.printf("C%u: PB=%d\r\n", channel, value);
}

void onSysExHandle(byte * array, unsigned size){
    Serial.printf("SysEx:\r\n");
    unsigned multipleOf8 = size/8;
    unsigned remOf8 = size % 8;
    for (unsigned idx=0; idx < multipleOf8; idx++) {
        for (unsigned jdx = 0; jdx < 8; jdx++) {
            Serial.printf("%02x ", *array++);
        }
        Serial.printf("\r\n");
    }
    for (unsigned idx = 0; idx < remOf8; idx++) {
        Serial.printf("%02x ", *array++);
    }
    Serial.printf("\r\n");
}

void onMidiClockHandle(){
    Serial.printf("Clock\r\n");
}

void onMidiStartHandle(){
    Serial.printf("Start\r\n");
}

void onMidiContinueHandle(){
    Serial.printf("Cont\r\n");
}

void onMidiStopHandle(){
    Serial.printf("Stop\r\n");
}

NoteOffFunctionPtr onNoteOff = onNoteOffHandle;
NoteOnFunctionPtr onNoteOn = onNoteOnHandle;
PolyphonicAftertouchFunctionPtr onPolyphonicAftertouch = onPolyphonicAftertouchHandle;
ControlChangeFunctionPtr onControlChange = onControlChangeHandle;
ProgramChangeFunctionPtr onProgramChange = onProgramChangeHandle;
AftertouchFunctionPtr onAftertouch = onAftertouchHandle;
PitchBendFunctionPtr onPitchBend = onPitchBendHandle;
SysExFunctionPtr onSysEx = onSysExHandle;
MidiClockFunctionPtr onMidiClock = onMidiClockHandle;
MidiStartFunctionPtr onMidiStart = onMidiStartHandle;
MidiContinueFunctionPtr onMidiContinue = onMidiContinueHandle;
MidiStopFunctionPtr onMidiStop = onMidiStopHandle;

void setup() {

  usb_midi.begin();
  MIDI_USB.begin(MIDI_CHANNEL_OMNI);
  Serial.begin(115200);
  while (!Serial) {
    delay(100);   // wait for native usb
  }
  Serial.println("TinyUSB MIDI Host Example");
  core0_booting = false;
  while(core1_booting) ;
}

static void blinkLED(void)
{
    const uint32_t intervalMs = 1000;
    static uint32_t startMs = 0;

    static bool ledState = false;
    if ( millis() - startMs < intervalMs)
        return;
    startMs += intervalMs;

    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH:LOW); 
}

void loop() {
  myMidiHost.readAll();
  myMidiHost.writeFlushAll();
  blinkLED();
}


void process_midi_data(uint8_t *data, uint32_t length) {
  // Implement your MIDI data processing logic here
  // Example: Print MIDI data
  Serial.printf("MIDI RX");
  for (uint32_t idx = 0; idx < length; idx++) {
    Serial.printf("%02x ", data[idx]);
  }

  // Add your specific processing logic as needed
}

// core1's setup for USB host
void setup1() {
  while(!Serial);   // wait for native usb
  Serial.println("Core1 setup to run TinyUSB host with pio-usb\r\n");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
      delay(2000);   // wait for native usb
      Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
      Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
      while(1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;
 
 #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  /* Need to swap PIOs so PIO code from CYW43 PIO SPI driver will fit */
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
 #endif /* ARDUINO_RASPBERRY_PI_PICO_W */
 
  USBHost.configure_pio_usb(1, &pio_cfg);
  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing work done in core1 to free up core0 for other work


  myMidiHost.begin(&USBHost, 1, onMIDIconnect, onMIDIdisconnect);
  core1_booting = false;
  while(core0_booting) ;
}

// core1's loop
void loop1()
{
  USBHost.task();
}