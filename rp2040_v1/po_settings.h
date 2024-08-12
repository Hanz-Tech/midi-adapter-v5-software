/*!
 *  @file       po_settings.cpp
 *  Project     Pocket Operator MIDI Adapter
 *  @brief      Pocket Operator MIDI Adapter
 *  @author     Hanz Tech Inc
 *  @date       2022/03/06
 *  @license    MIT - Copyright (c) 2022 Hanz Tech Inc
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

#ifndef PO_SETTINGS_H
#define PO_SETTINGS_H

//Shift Register Out mapping to PO button
#define PO_BUTTON_1 1
#define PO_BUTTON_2 2
#define PO_BUTTON_3 3
#define PO_BUTTON_4 4
#define PO_BUTTON_5 5
#define PO_BUTTON_6 6
#define PO_BUTTON_7 7
#define PO_BUTTON_8 8
#define PO_BUTTON_9 9
#define PO_BUTTON_10 10
#define PO_BUTTON_11 11
#define PO_BUTTON_12 12
#define PO_BUTTON_13 13
#define PO_BUTTON_14 14
#define PO_BUTTON_15 15
#define PO_BUTTON_16 16
#define PO_BUTTON_SOUND 17
#define PO_BUTTON_PATTERN 18
#define PO_BUTTON_WRITE 19
#define PO_BUTTON_PLAY 20
#define PO_BUTTON_FX 21
#define PO_BUTTON_BPM 22
#define PO_BUTTON_SPECIAL 23

//RP2040 GPIO out
#define CLOCKSYNCPIN 38
#define ESP32_ENABLE 37

//Midi channel and other options
#define PO_MIDI_CHANNEL 1 //MIDI channel to control the PO
#define DISABLE_TRANSPORT 1 //1 = Disable Tranport to Midi out, 0 = Enable Transport to midi_out
#define PO_CC_CONTROL 0 //0 = Disable PO CC Control, switch between differet modes
#define VOLCA_FM_VELOCITY 1
#define VOLCAFM_MIDI_CHANNEL_1 13
#define VOLCAFM_MIDI_CHANNEL_2 16

//Different Mode of the op, only used if PO_CC_CONTROL=True
#define PERF_MODE 0
#define NORMAL_MODE 1
#define FX_MODE 2
#define RECORD_MODE 3
#define WRITE_MODE 4

#endif
