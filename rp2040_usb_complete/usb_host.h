
/*!
 *  @file       usb_host.h
 *  Project     Pocket Operator MIDI Adapter v5
 *  @brief      Pocket Operator MIDI Adapter v5
 *  @author     Hanz Tech Inc
 *  @date       2024/06/17
 *  @license    MIT - Copyright (c) 2024 Hanz Tech Inc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef USB_HOST_H
#define USB_HOST_H

#include "Adafruit_TinyUSB.h"
#include "pio_usb.h"

// Add USB MIDI Host support to Adafruit_TinyUSB
#include "usb_midi_host.h"

typedef void (*MidiDataProcessor)(uint8_t *, uint32_t);

class USB_HOST {
  private:
   uint8_t midi_dev_addr;
   uint32_t num_packets;
    // holding device descriptor
    tusb_desc_device_t desc_device;
    void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance);
    void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx);
    void tuh_mount_cb (uint8_t daddr);
    void tuh_umount_cb(uint8_t daddr);

  public:
   USB_HOST();
   void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets, MidiDataProcessor dataProcessor);
   // USB Host object
   Adafruit_USBH_Host usbh_instance;
   uint8_t get_midi_dev_addr() { return midi_dev_addr};
   uint8_t get_num_packets() { return num_packets};
};


#endif