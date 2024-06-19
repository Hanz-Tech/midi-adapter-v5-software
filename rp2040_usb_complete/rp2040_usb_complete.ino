#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include "usb_host.h"
#include "pio_usb.h"

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);

int latchPin = 17;      // Latch pin of 74HC595 is connected to Digital pin 17
int clockPin = 18;      // Clock pin of 74HC595 is connected to Digital pin 18 SRCLK
int dataPin = 16;       // Data pin of 74HC595 is connected to Digital pin 16 SER

byte leds = 0;         // Variable to hold the pattern of which LEDs are currently turned on or off

USB_HOST *usb_host;
extern void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets, MidiDataProcessor dataProcessor);
void process_midi_data(uint8_t *data, uint32_t length);

void setup()
{
  usb_host = new USB_HOST();

  Serial.begin(115200);
  delay(2000);   // wait for native usb

  Serial.println("TinyUSB MIDI Host Example");
  usb_host->tuh_midi_rx_cb(usb_host->get_midi_dev_addr(), usb_host->get_num_packets(), process_midi_data);
}

void loop()
{
  bool connected = usb_host->get_midi_dev_addr() != 0 && tuh_midi_configured(usb_host->get_midi_dev_addr());
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
  delay(2000);   // wait for native usb
  Serial.println("Core1 setup to run TinyUSB host with pio-usb");

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
 
  usb_host->usbh_instance.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  usb_host.usbh_instance.begin(1);
}

// core1's loop
void loop1()
{
  usb_host.usbh_instance.task();
}