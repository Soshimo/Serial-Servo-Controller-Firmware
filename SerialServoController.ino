/*
  SerialServoControler Application
  Copyright (c) 2012 Scott R. McCain.  All right reserved.

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
  following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions
  of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.
*/

#include <Wire.h>
#include <i2cEEPROM.h>
#include <Servo.h>

#define NUMSERVOS 12

#define BUFFSIZE 24

// EEPROM related defines
// device id of eeprom (24ac512)
#define DEVICEID 0x50    
// address of user data
#define USERDATA 0x2000  
// address of frame data
#define FRAMEDATA 0x20   
// address of startup data
#define STARTUPDATA 0x00
// max number of frames that can be saved
#define MAXFRAMES 10    

// for the EEPROM routines
typedef struct _tagEEPROMData {
	char header[9]; /* EEPROMV2 */
	byte default_or_defined;
	byte startup_positions[NUMSERVOS];
} EEPROMData;

typedef struct _tagServoWrapper {
  int pin;
  int position;
  Servo servo;
  void disable() { servo.detach(); }
  void enable() { servo.attach(pin); }
  void update() { servo.write(position); }
} ServoWrapper;

typedef enum { start, attention, command, stream } state;
typedef enum { none, out, in } stream_direction;

typedef enum { idle, serial_control_write, serial_control_read, 
				define_startup_val, default_or_defined, enable_channel, 
				disable_channel, write_frame, play_sequence, user_stream_data, 
				version_read } command_type;


static ServoWrapper Servos[NUMSERVOS];
static EEPROMData servodata;

volatile byte _cmdbuffer[BUFFSIZE + 1];
volatile int _buffindex = 0;

volatile int _mode = start;
volatile int _commandtype = idle;

volatile unsigned int _channel = 0;
volatile unsigned int _angle = 0;

volatile int _stream_direction = none;
volatile int _stream_length = 0;
volatile int _stream_index = 0;

volatile int _frame_buffer_size;
volatile int _frame_buffer_index;

void updateServoData() {
  i2cEEPROM.writeBuffer(0, (byte *)&servodata, 0, sizeof(EEPROMData));
}

void writeDefaultOrDefined(byte val) {
  servodata.default_or_defined = val;
  updateServoData();
}

void writeStartupVal(int channel, int position) {
  if( channel >= 0 && channel < NUMSERVOS )
    servodata.startup_positions[channel] = (byte)position;
  updateServoData();
}

void disableChannel(int channel) {
  if( channel >= 0 && channel < NUMSERVOS )
    Servos[channel].disable();
}

void enableChannel(int channel) {
  if( channel >= 0 && channel < NUMSERVOS )
    Servos[channel].enable();
}

void startServo(struct _tagServoWrapper* servo, int pin) {
  servo->pin = pin;
  servo->enable();
}
 
void updateServo(struct _tagServoWrapper* servo, int position) {
  servo->position = position;
  servo->update();
}

void writeNack() {
  //Serial.write("NACK\n");
}

void writeAck() {
  //Serial.write("ACK\n");
}

void initializeMemory() {
  memset(&servodata, 0, sizeof(EEPROMData));
	
  servodata.header[0] = 'E';
  servodata.header[1] = 'E';
  servodata.header[2] = 'P';
  servodata.header[3] = 'R';
  servodata.header[4] = 'O';
  servodata.header[5] = 'M';
  servodata.header[6] = 'V';
  servodata.header[7] = '2';
	
	
  for(int ipos=0;ipos < NUMSERVOS; ipos++) {
  	servodata.startup_positions[ipos] = 90;
  }
	
  i2cEEPROM.writeBuffer(0, (byte *)&servodata, 0, sizeof(EEPROMData));
}

void checkMemory() {

  i2cEEPROM.readBuffer(0, (byte *)&servodata, 0, sizeof(EEPROMData));
	
  // check if dt isn't initialized
  servodata.header[8] = 0;
  if( strcmp(servodata.header, "EEPROMV2") ) {
    initializeMemory();
  }
	
  if( servodata.default_or_defined ) {
    // if 1 then we used defined values
		
    for(int idxservo = 0; idxservo<NUMSERVOS; idxservo++) {
      // set to startup position
      updateServo(&Servos[idxservo], servodata.startup_positions[idxservo]);
    }
  }
}

void streamFrameData(byte data) {
  // we are a frame of data, so waiting for that data
  // the first byte is the number of channels to store
  // the second byte is the frame to store
	
  // the next bytes is the frame data for each channel
  if( _buffindex == 0 ) {
    // frame index (0 - 11)
    _frame_buffer_index = data;
				
    if( _frame_buffer_index < 0 )
      _frame_buffer_index = 0;
    else if( _frame_buffer_index >= MAXFRAMES )
      _frame_buffer_index = MAXFRAMES - 1;
				
    _buffindex++;
  } else {
		
    // calculate offset into framedata
    int offset = FRAMEDATA + (NUMSERVOS * _frame_buffer_index);
				
    // write data byte, compensate for the buffer being off by one
    i2cEEPROM.writeByte(offset + _buffindex - 1, data);

    if(  _buffindex == NUMSERVOS ) {
      
      // we are done, so reset to start mode
      _mode = start;
      _buffindex = 0;
				
      Serial.write("Frame ");
      Serial.print(_frame_buffer_index);
      Serial.write(" stored\r\n");
      writeAck();
      
    } else {
      _buffindex++;
    }
  }
}

void setup() {

  // channel 2 is the first servo channel available
  for(int i=0; i <NUMSERVOS; i++) {
    startServo(&Servos[i], 2+i);
    updateServo(&Servos[i], 90);
  }

  Serial.begin(19200);
	
  i2cEEPROM.begin(DEVICEID);
	
  checkMemory();
}

void handleCommand(byte data) {

	switch(_commandtype) {
	
		case user_stream_data:
			if( _buffindex == 0 ) {
				// high byte first
				_stream_length = ((int)data << 8);
				_buffindex++;
			}
			else if( _buffindex == 1 ) {
				// low byte
				_stream_length |= ((int)data & 0x00ff);
			
				
				if( _stream_length > 0 )
					_mode = stream;
				else
					_mode = start;
					
				if( _mode == stream && _stream_direction == out ) {
					for(; _stream_index < _stream_length; _stream_index++) {
					// streaming out - send data out
						byte eepromdata = i2cEEPROM.readByte(USERDATA + _stream_index);
						Serial.write( eepromdata );
					}
					
					writeAck();
					_mode = start;
				}
				
				_buffindex = 0;
			}
			
			break;

			
		case serial_control_write:
			// channel data with angle movement
			if( _buffindex == 0 )
				_channel = data;
			else if( _buffindex == 1 )
				_angle = data;
				
			_buffindex++;
			if( _buffindex == 2 ) {
			
				if( _channel < 0 ) _channel = 0;
				if( _channel > NUMSERVOS - 1 ) _channel = NUMSERVOS - 1;
				
				if( _angle < 0 ) _angle = 0;
				if( _angle > 180 ) _angle = 180;
				
				updateServo(&Servos[_channel], _angle);
				
				_mode = start;
				_buffindex = 0;
				
				writeAck();
			}
			break;
			
		case define_startup_val:
			// channel data with angle movement
			if( _buffindex == 0 )
				_channel = data;
			else if( _buffindex == 1 )
				_angle = data;
				
			_buffindex++;
			if( _buffindex == 2 ) {
			
				if( _channel < 0 ) _channel = 0;
				if( _channel > NUMSERVOS - 1 ) _channel = NUMSERVOS - 1;
				
				if( _angle < 0 ) _angle = 0;
				if( _angle > 180 ) _angle = 180;
				
				writeStartupVal(_channel, _angle);
				
				_mode = start;
				_buffindex = 0;
				writeAck();
			}
			break;
			
		case default_or_defined:
			writeDefaultOrDefined(data); 
			
			_mode = start;
			_buffindex = 0;
			writeAck();
			
			break;
			
		case enable_channel:
			enableChannel(data);
			
			_mode = start;
			_buffindex = 0;
			writeAck();
			
			break;
			
		case disable_channel:
			disableChannel(data);
			
			_mode = start;
			_buffindex = 0;
			writeAck();
			
			break;
			
		case write_frame:
                        streamFrameData(data);
			break;
	}
}

void loop(void) {

	for(;;) {
	
		if( Serial.available() ) {
			byte b = Serial.read();
			
			switch(_mode) {
				case start:
				// normal mode, we are waiting for some data
				_cmdbuffer[_buffindex] = b;
				_buffindex++;
				
				if( _buffindex == 4 ) {
					_cmdbuffer[4] = 0;
					if( strcmp((char *)_cmdbuffer, "!ATT") == 0 ) {
						_mode = attention;
					} else {
						writeNack();
					}
					
					_buffindex = 0;
				}
				break;
				
				case attention:
				_cmdbuffer[_buffindex] = b;
				_buffindex++;
				
				if( _buffindex == 3 ) {
					_cmdbuffer[3] = 0;
					if( strcmp((char *)_cmdbuffer, "SCW") == 0 ) {
						
						_mode = command;
						_commandtype = serial_control_write;
					} else if( !strcmp((char *)_cmdbuffer, "DEF")) {
					
						_mode = command;
						_commandtype = define_startup_val;
					} else if( !strcmp((char *)_cmdbuffer, "EDD")) {
					
						_mode = command;
						_commandtype = default_or_defined;
					} else if( !strcmp((char *)_cmdbuffer, "PSE")) {
					
						_mode = command;
						_commandtype = enable_channel;
					} else if( !strcmp((char *)_cmdbuffer, "PSD")) {
					
						_mode = command;
						_commandtype = disable_channel;
					} else if( !strcmp((char *)_cmdbuffer, "STW")) {
						
						_mode = command;
						_commandtype = user_stream_data;
						
						// initialize stream
						_stream_direction = in;
						_stream_index = 0;
						_stream_length = 0;
						
					} else if( !strcmp((char *)_cmdbuffer, "STR")) {
						
						_mode = command;
						_commandtype = user_stream_data;
						
						// initialize stream
						_stream_direction = out;
						_stream_index = 0;
						_stream_length = 0;
						
					} else if( !strcmp((char *)_cmdbuffer, "WRF")) {
						
						_mode = command;
						_commandtype = write_frame;
					} else if( !strcmp((char *)_cmdbuffer, "PLY")) {
						
						byte savedpos[NUMSERVOS];
						for(int servoidx=0; servoidx<NUMSERVOS; servoidx++) {
							savedpos[servoidx] = (byte)Servos[servoidx].position;
						}
						
						//_mode = command;
						//_commandtype = write_frame;
						for(int frameidx=0;frameidx < MAXFRAMES; frameidx++) {
							int offset = FRAMEDATA + (NUMSERVOS * frameidx);
				
							// write data byte, compensate for the buffer being off by one
							byte framedata[NUMSERVOS];
							for(int servoidx=0;servoidx < NUMSERVOS; servoidx++) {
								framedata[servoidx] = i2cEEPROM.readByte(offset + _buffindex - 1);
							}

							// now write the servo data
							for(int servoidx=0;servoidx < NUMSERVOS; servoidx++) {
								updateServo(&Servos[servoidx], framedata[servoidx]);
							}

							delay(250);
						}
						
						Serial.write("Finished sequence.  ");
						Serial.print(MAXFRAMES);
						Serial.print( " frames played\r\n");
						
						for(int servoidx=0; servoidx<NUMSERVOS; servoidx++) {
							updateServo(&Servos[servoidx], savedpos[servoidx]);
						}
												
						// play a sequence of moves
						// ignore input until then
						// hopefully we don't overflow
						
						// send ack when done
						
					} else if( !strcmp((char *)_cmdbuffer, "VER")) {
					
						_mode = start;
						writeAck();
						
						Serial.write("VER1a\n");
					} else {

						_mode = start;
						writeNack();
					}
					
					_buffindex = 0;
				}
				break;
				
				case command:
					handleCommand(b);
					break;
					
				case stream:
					i2cEEPROM.writeByte(USERDATA + _stream_index, b);

					_stream_index++;
					if( _stream_index == _stream_length ) {
						_mode = start;
					}

					writeAck();
					break;
			}
			
		}
	}
}


