#include <MCP_DAC.h>
#include <SPI.h>
#include <SRAM_23LC.h>
#include <RF24.h>

const long RAM_MAX_LENGTH = 65534;

const uint8_t LED_PIN = 2;
const uint8_t BUTTON_PIN = 3;
const uint8_t MIC_PIN = A0;
const uint8_t DAC_CS_PIN = 4;
const uint8_t RAM_CS_PIN = 5;
const uint8_t DEBOUNCE_DELAY = 100;
const uint8_t RADIO_CE_PIN = 9;
const uint8_t RADIO_CS_PIN = 10;
const uint8_t BUFFER_SIZE = 32;

const uint8_t TIMER_INCREMENT = 204;
const uint16_t TIMER_CEILING = TIMER_INCREMENT * 16;

bool last_button_state = true;
unsigned long last_button_press_time = 0;

uint8_t adc_save; // Default ADC mode

volatile uint32_t incoming_buffer_index = 0;
volatile uint32_t recording_buffer_index = 0;
volatile uint32_t playback_buffer_index = 0;
volatile unsigned long last_incoming_transmission_time = 0;
volatile bool streaming = false;
volatile bool recording = false;
volatile bool readyForTransmission = false;

uint8_t transmit_buffer[BUFFER_SIZE + 1];

MCP4921 MCP;
SRAM_23LC SRAM(&SPI, RAM_CS_PIN, SRAM_23LCV1024);
RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);

const uint64_t pipes[2] = {0xABCDABCD71LL,
                           0x544d52687CLL};
 
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MIC_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(BUTTON_PIN, HIGH);

  MCP.begin(DAC_CS_PIN);
  SRAM.begin();

  // Set up Analog-to-Digital converter:
  analogReference(EXTERNAL); // 3.3V ref, MAX9814 has 2Vpp and 1.25V offset (min 0.25V - max 2.25V)
  adc_save = ADCSRA;         // Save ADC setting for restore later

  analogRead(MIC_PIN); // Do one reading to set the AREF properly

  setupRadio();
  setupTimers();
}

void setupTimers() {
  noInterrupts();          // Disable all interrupts

  // Reset Timer 1 and Timer 2 registers
  TCCR1A = 0;             
  TCCR1B = 0;              
  TCCR2A = 0;
  TCCR2B = 0;
  
  /*
  In CTC mode, Timer1 counts from 0 to OCR1A value. Below, we set the OCR1A to the following:
    (16MHz / 8 / 9.6kHz) * 16
  In this calculation, 16MHz is the Arduino clock speed, 8 is the prescaler (set with CS11 below)
  and 9.6kHz is our desired playback rate. We multiply by 16 because we wish to have the COMPA
  ISR trigger 1/16 of the playback speed.

  Then we set Timer2 to the actual playback speed of 9.6kHz via OCR2A.

  This means that Timer1 will count from 0 to 3328, and trigger COMPA every time it hits the ceiling.
  Timer2 will trigger every time it hits OCR2A.
  */

  OCR1A = TIMER_CEILING;  // COMPA triggers at 9.6kHz 1/16 for buffer refill from radio
  TCCR1B |= bit(WGM12);   // CTC mode
  TCCR1B |= bit(CS11);    // 8 prescaler 
  TIMSK1 |= bit(OCIE1A);  // Enable COMPA interrupt (radio)

  // Timer2 for playback
  TCCR2A = bit(WGM21);        // CTC mode
  OCR2A  = TIMER_INCREMENT;   // Count to (16MHz / 8 / 9.6kHz)
  TIMSK2 = bit(OCIE2A);       // Enable Timer2 Interrupt

  TCNT1 = 0;      // Set both counters to zero
  TCNT2 = 0;     

  // Reset prescalers
  GTCCR = bit(PSRASY);      // Reset prescaler now
  TCCR2B =  bit(CS21);      // Prescaler of 8

  interrupts();             // enable all interrupts
}

void disablePlaybackTimer() {
  TIMSK2 &= bit(OCIE2A);  // Disable playback interrupt
}

void enablePlaybackTimer() {
  TIMSK2 |= bit(OCIE2A);  // Enable radio interrupt
}

void disableRadioTimer() {
  TIMSK1 &= bit(OCIE1A);  // Disable radio interrupt
}

void enableRadioTimer() {
  TIMSK1 |= bit(OCIE1A);  // Enable radio interrupt
}

void disableTimers() {
  disablePlaybackTimer();
  disableRadioTimer();
}

void enableTimers() {
  enablePlaybackTimer();
  enableRadioTimer();
}

void setupRadio() {
  if (!radio.begin()) {
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    } // Abort
  }

  radio.setChannel(5);                                  // Set RF channel to 5
  radio.setDataRate(RF24_250KBPS);                      // Set data rate to 250Kbps
  radio.setCRCLength(RF24_CRC_8);                       // Set CRC to 1 byte for speed

  beginStreaming();
}

void loop() {
  if (readyForTransmission) {
    transmitData();
  }

  handleButton();
}

void handleButton() {
  if (readyForTransmission) return;

  bool button_state = digitalRead(BUTTON_PIN);

  if (button_state != last_button_state) {
    uint32_t interrupt_time = millis();
    
    if (interrupt_time - last_button_press_time > DEBOUNCE_DELAY) {
      if (recording && button_state == HIGH) {
        stopRecording();
      } else if (!recording && button_state == LOW) {
        beginRecording();
      }
      
      last_button_press_time = interrupt_time;
    }

    last_button_state = button_state;
  }
}

uint8_t vout;

void handlePlayback() {
  if (!streaming) return;

  if (playback_buffer_index < incoming_buffer_index) {
    vout = SRAM.readByte(playback_buffer_index);
  } else {
    vout = 125; // Mute outgoing sound
  }

  MCP.fastWriteA(vout * 4);

  playback_buffer_index++;

  if (playback_buffer_index >= RAM_MAX_LENGTH) {
    playback_buffer_index = 0;
  }
}

void handleRadio() {
  if (!streaming) return;
  if (!radio.available()) return;

  disableRadioTimer();

  digitalWrite(LED_PIN, HIGH);

  unsigned long current_time = millis();

  if (current_time - last_incoming_transmission_time > 1000) {
    incoming_buffer_index = 0; // Reset incoming memory index to zero
    playback_buffer_index = 0; // Reset playback buffer back to zero
  }

  last_incoming_transmission_time = current_time;

  uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
  radio.read(&transmit_buffer, bytes); // Read into active buffer

  for (uint8_t i = 0; i < bytes; i++) {
    SRAM.writeByte(incoming_buffer_index + i, transmit_buffer[i]);
  }

  incoming_buffer_index += bytes;

  if (incoming_buffer_index >= RAM_MAX_LENGTH) {
    incoming_buffer_index = 0;
  }

  digitalWrite(LED_PIN, LOW);

  enableRadioTimer();
}

void beginRecording() {
  streaming = false;
  recording = true;

  disableTimers();

  digitalWrite(LED_PIN, HIGH);

  recording_buffer_index = 0;
  
  // Start up ADC in free-run mode for audio sampling:
  DIDR0 |= _BV(ADC0D);  // Disable digital input buffer on ADC0
  ADMUX  = _BV(ADLAR);  // Left-shift since we only read high-byte
  ADCSRB = 0;           // Free-run mode
  ADCSRA = _BV(ADEN) |  // Enable ADC
    _BV(ADSC)  |        // Start conversions
    _BV(ADATE) |        // Auto-trigger enable
    _BV(ADIE)  |        // Interrupt enable
    _BV(ADPS2) |        // 128:1 prescale...
    _BV(ADPS1) |        //  ...yields 125 KHz ADC clock...
    _BV(ADPS0);         //  ...13 cycles/conversion = ~9615 Hz
}

void beginStreaming() {
  streaming = true;

  radio.flush_rx();
  radio.openWritingPipe(pipes[0]);         // Set up reading and writing pipes
  radio.openReadingPipe(1, pipes[1]);
  radio.startListening();                  // Exit sending mode
}

void stopRecording() {
  ADCSRA = adc_save; // Disable ADC interrupt
  readyForTransmission = true;
  recording = false;

  digitalWrite(LED_PIN, LOW);

  enableTimers();
}

void transmitData() {
  disableTimers();

  digitalWrite(LED_PIN, HIGH);
  
  radio.openWritingPipe(pipes[1]);       // Set up reading and writing pipes
  radio.openReadingPipe(1, pipes[0]);
  radio.stopListening();                 // Enter transmit mode on the radio

  radio.flush_tx();

  uint8_t failures = 0;
  uint32_t transmit_index = 0;
  
  while (transmit_index < recording_buffer_index) {
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
      transmit_buffer[i] = SRAM.readByte(RAM_MAX_LENGTH + transmit_index + i);
    }

    if (!radio.writeFast(&transmit_buffer, BUFFER_SIZE)) {
      failures++;

      radio.reUseTX();
    } else {
      transmit_index += BUFFER_SIZE;
    }

    if (failures >= 100) {
      break;
    }
  }

  recording_buffer_index = 0;
  readyForTransmission = false;
  
  digitalWrite(LED_PIN, LOW);

  beginStreaming();

  enableTimers();
}

// Interrupts

// ADC Conversion timer
ISR(ADC_vect, ISR_BLOCK) {
  // SRAM.writeByte(recording_buffer_index, ADCL);
  SRAM.writeByte(RAM_MAX_LENGTH + recording_buffer_index, ADCH);

  recording_buffer_index++;

  // We reached buffer limit, stop recording
  if (recording_buffer_index >= RAM_MAX_LENGTH) {
    stopRecording();
  }
}

// Timer1 radio interrupt – Runs at 1/16 speed of audio playback
ISR(TIMER1_COMPA_vect) {
  handleRadio();
}

ISR (TIMER2_COMPA_vect) {
  handlePlayback();
}
