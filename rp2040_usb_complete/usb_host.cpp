/*!
 *  @file       usb_host.cpp
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
#include "usb_host.h"

USB_HOST::USB_HOST(){
  midi_dev_addr = 0;
}



void USB_HOST::tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets, MidiDataProcessor dataProcessor) {
  if (midi_dev_addr == dev_addr) {
    if (num_packets != 0) {
      uint8_t cable_num;
      uint8_t buffer[4];
      while (1) {
        uint32_t bytes_read = tuh_midi_packet_read(dev_addr, buffer);
        if (bytes_read == 0)
          return;

        // Call the provided function pointer to process MIDI data
        dataProcessor(buffer, bytes_read);
      }
    }
  }
}


