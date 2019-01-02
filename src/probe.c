/*
  probe.c - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014-2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
  
#include "grbl.h"
#include "serial2.h"

// Inverts the probe pin state depending on user settings and probing cycle mode.
uint8_t probe_invert_mask;


// Probe pin initialization routine.
void probe_init() 
{
  PROBE_DDR &= ~(PROBE_MASK); // Configure as input pins
  #ifdef DISABLE_PROBE_PIN_PULL_UP
    PROBE_PORT &= ~(PROBE_MASK); // Normal low operation. Requires external pull-down.
  #else
    PROBE_PORT |= PROBE_MASK;    // Enable internal pull-up resistors. Normal high operation.
  #endif
  // probe_configure_invert_mask(false); // Initialize invert mask. Not required. Updated when in-use.
  serial2_init();
  uart_printf("probe init called\r\n");
}

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
#define byte uint8_t
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

void probe_set_limit(uint16_t limit) {

	char lowByte = (uint8_t)(0x00FF & limit);
	char highByte = (uint8_t)((0xFF00 & limit)>>8);

	uart_printf("sending %02x %02x to Attiny85\r\n", lowByte, highByte);

	serial2_write(0xAA);
	serial2_write(lowByte);
	serial2_write(highByte);
	serial2_write(0xCC);//CRC8((const byte*){lowByte,highByte},2) );

//	const char test[] = "das ist ein test";
//
//	for(int i = 0; i < 16; i++)
//		serial2_write(test[i]);

//	while(!serial2_tx_buffer_tail < 1);
	int resp = SERIAL_NO_DATA;
	unsigned long retryCnt = 0;
	int receivedBytes = 0;

//	while(((resp = serial2_read()) == SERIAL_NO_DATA) && (retryCnt < 1000000)) {
////		delay_us(1000);
//		retryCnt++;
//	}
//
//	receivedBytes++;

	delay_ms(1000);

#define NUM_EXP_BYTES 2 //OK for ok, NA for not ok
	uint8_t buf[5];
//	buf[0] = resp;
	buf[NUM_EXP_BYTES] = 0;
	retryCnt = 1;

	while(receivedBytes < NUM_EXP_BYTES && retryCnt < 10000) {
		resp = serial2_read();
		delay_us(100);
		if(resp != SERIAL_NO_DATA)
			buf[receivedBytes++] = resp;
		retryCnt++;
	}

	uart_printf("delay was %d, resp = %02X\r\n", retryCnt, resp);
	uart_printf("received %d chars from Attiny85\r\n", receivedBytes);
	uart_printf("Received from Attiny85: ");
	for(int i = 0; i < receivedBytes; i++)
		uart_printf("%02x ", buf[i]);
	uart_printf("\r\n");
}


// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to 
// appropriately set the pin logic according to setting for normal-high/normal-low operation 
// and the probing cycle modes for toward-workpiece/away-from-workpiece. 
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; // Initialize as zero.
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}


// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
uint8_t probe_get_state() { return((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); }


// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
void probe_state_monitor()
{
  if (sys_probe_state == PROBE_ACTIVE) {
    if (probe_get_state()) {
      sys_probe_state = PROBE_OFF;
      memcpy(sys.probe_position, sys.position, sizeof(sys.position));
      bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
    }
  }
}
