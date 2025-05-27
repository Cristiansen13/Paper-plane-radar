extern "C" {
  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include <util/delay.h>
  #include <stdlib.h>
}
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

#define NUM_SENSORS 4
#define THRESHOLD_CM 40
LiquidCrystal_I2C lcd(0x27, 16, 2);

volatile uint32_t timer2_ticks = 0;
uint32_t last_drop_time = 0;
uint8_t last_drop_pair = 255;

uint16_t last_distance[NUM_SENSORS] = {0};

const uint8_t trig_pins[NUM_SENSORS] = {PD4, PD5, PD6, PD7};
const uint8_t echo_pins[NUM_SENSORS] = {PB0, PB1, PB2, PB4};

volatile uint8_t current_sensor = 0;
volatile uint16_t echo_start[NUM_SENSORS] = {0};
volatile uint16_t echo_duration[NUM_SENSORS] = {0};
volatile uint8_t echo_flag[NUM_SENSORS] = {0};

void uart_init(unsigned int ubrr) {
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_send(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void uart_print(const char* str) {
  while (*str) uart_send(*str++);
}

void uart_print_uint16(uint16_t value) {
  char buf[10];
  itoa(value, buf, 10);
  uart_print(buf);
}

void trigger_sensor(uint8_t index) {
  PORTD |= (1 << trig_pins[index]);
  _delay_us(10);
  PORTD &= ~(1 << trig_pins[index]);
  echo_flag[index] = 0;
}

ISR(TIMER1_COMPA_vect) {
  trigger_sensor(current_sensor);
  current_sensor = (current_sensor + 1) % NUM_SENSORS;
}

ISR(TIMER2_OVF_vect) {
  timer2_ticks++;
}

ISR(PCINT0_vect) {
  uint8_t pin_state = PINB;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (echo_flag[i] == 0 && (pin_state & (1 << echo_pins[i]))) {
      echo_start[i] = TCNT1;
      echo_flag[i] = 1;
    }
    else if (echo_flag[i] == 1 && !(pin_state & (1 << echo_pins[i]))) {
      uint16_t end = TCNT1;
      echo_duration[i] = (end >= echo_start[i]) ? (end - echo_start[i]) : (0xFFFF - echo_start[i] + end);
      echo_flag[i] = 2;
    }
  }
}

uint32_t get_time_ms() {
  uint32_t ticks_copy;
  uint8_t tcnt2_copy;
  cli();
  ticks_copy = timer2_ticks;
  tcnt2_copy = TCNT2;
  sei();
  return ((ticks_copy * 256UL + tcnt2_copy) * 64UL) / 1000UL;
}
void buzzer_stop() {
  TCCR2A = 0;
  TCCR2B = 0;
  PORTD &= ~(1 << PD3);

  TCNT2 = 0;
  timer2_ticks = 0;

  TCCR2A = 0;
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);

  TIMSK2 |= (1 << TOIE2);
}

void buzzer_start(uint16_t frequency, uint8_t duty_cycle) {
  if (frequency == 0) {
    buzzer_stop();
    return;
  }

  const uint16_t prescaler = 8;

  uint16_t top = (F_CPU / (prescaler * frequency)) - 1;
  if (top > 255) top = 255;

  DDRD |= (1 << PD3);

  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22) | (1 << CS21);

  OCR2A = top;
  OCR2B = (top * duty_cycle) / 100;

  TIMSK2 &= ~(1 << TOIE2);
}



void setup() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    DDRD |= (1 << trig_pins[i]);

  DDRB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB4));
  PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB4);

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB4);

  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
  OCR1A = 5000;
  TIMSK1 |= (1 << OCIE1A);

  TCCR2A = 0;
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << TOIE2);

  uart_init(103);

  Wire.begin();
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting for ");
  lcd.setCursor(0, 1);
  lcd.print("       planes...");
}

int main() {
  init();
  setup();
  sei();

  uart_print("4 sensors with Timer1 trigger + Timer2 chrono + INT echo\r\n");

  while (1) {
    for (uint8_t i = 0; i < NUM_SENSORS; i += 2) {
      if (echo_flag[i] == 2 && echo_flag[i + 1] == 2) {
        uint16_t duration_us_1 = echo_duration[i] * 4;
        uint16_t distance_cm_1 = duration_us_1 / 58;
        echo_flag[i] = 0;

        uint16_t duration_us_2 = echo_duration[i + 1] * 4;
        uint16_t distance_cm_2 = duration_us_2 / 58;
        echo_flag[i + 1] = 0;

        bool dropped = false;
        if (last_distance[i] > distance_cm_1 &&
            last_distance[i] - distance_cm_1 > THRESHOLD_CM)
          dropped = true;

        if (last_distance[i + 1] > distance_cm_2 &&
            last_distance[i + 1] - distance_cm_2 > THRESHOLD_CM)
          dropped = true;

        if (dropped) {
          uint32_t current_time = get_time_ms();
          uint8_t current_pair = i / 2;

          uart_print_uint16((uint16_t)current_time);
          uart_print(" ms - ALERT la Perechea ");
          uart_send('1' + current_pair);
          uart_print(" a detectat o SCADERE brusca!\r\n");

          if (last_drop_time != 0 && last_drop_pair != current_pair) {
            uint32_t elapsed_time = current_time - last_drop_time;
            uart_print_uint16((uint16_t)elapsed_time);
            uart_print(" ms durata intre alerte!\r\n");

            char buf[16];
            float speed = 300.0f / elapsed_time;

            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Speed:");

            lcd.setCursor(0,1);
            dtostrf(speed, 5, 2, buf);
            lcd.print(buf);
            lcd.print(" m/s");

            uint16_t freq = (uint16_t)(speed * 150 + 200);
            
            uart_print_uint16(freq);
            uart_print(" Hz - Buzzer ON!\r\n");
            buzzer_start(freq, 50);
            _delay_ms(200);
            buzzer_stop();

            last_drop_time = 0;
            last_drop_pair = 255;
          } else {
            last_drop_time = current_time;
            last_drop_pair = current_pair;
          }
        }

        last_distance[i] = distance_cm_1;
        last_distance[i + 1] = distance_cm_2;
      }
    }
  }
}