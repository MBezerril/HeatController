#include <DHT.h>
#include "U8glib.h"

#define coolerFan 3
#define resistencePin 4
#define pinButtonUpTemp 9
#define pinButtonDownTemp 10
#define pinButtonPower 11
#define DHT11PIN 7

int led = HIGH;
bool isActivated = false;
volatile int targetTemperature = 30;
int actualTemperature = 0;
long contador = 0;
volatile uint8_t portbHistory = 0xFF; // padrão é alto por causa do pull-up
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send AC

void setup() {
  cli();//stop interrupts
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15625;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // limpa os pinos PB0, PB1, PB2 (D8,D9,D10 na placa do arduino)
  DDRB &= ~((1 << DDB0) | (1 << DDB1) | (1 << DDB2));
  // PB0,PB1,PB2 (PCINT0, PCINT1, PCINT2) agora são entradas

  PORTB |= ((1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2)); // Ativa o Pull-up
  // PB0, PB1 e PB2 agora são entradas com pull-up habilitado

  PCICR |= (1 << PCIE0); // aciona PCIE0 para habilitar o scanneamento do PCMSK0 
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2); // aciona PCINT0 to trigger an interrupt on state change 
  sei(); // turn on interrupts
  Serial.begin(9600);
  pinMode(coolerFan, OUTPUT);
  pinMode(resistencePin, OUTPUT);
  contador = millis();
  Serial.println("START");
}

void loop() {
  if(millis() - contador >250){
    contador = millis();
    Serial.print("Temp Target: ");
    Serial.println(targetTemperature);
    u8g.firstPage();  
    do
    {
      draw();
    } while( u8g.nextPage() );
  }
}

ISR(TIMER1_COMPA_vect){
  led = led == LOW ? HIGH : LOW;
  digitalWrite(coolerFan, led);
  if (isActivated) {
    if (actualTemperature < targetTemperature) {
      digitalWrite(coolerFan, LOW); //desliga o cooler
      digitalWrite(resistencePin, HIGH); //aciona a resistencia de aquecimento
    } else if (actualTemperature > targetTemperature) {
      digitalWrite(coolerFan, HIGH); //liga o cooler
      digitalWrite(resistencePin, LOW); //desliga a resistencia de aquecimento
    } else {
      digitalWrite(coolerFan, LOW); //desliga o cooler
      digitalWrite(resistencePin, LOW); //desliga a resistencia de aquecimento
    }
  }
}

ISR(PCINT0_vect) {
  uint8_t changedbits;

  changedbits = PINB ^ portbHistory; //detecta quem mudou
  portbHistory = PINB; //salva qual o ultimo estado de cada pino
  if (changedbits & (1 << PINB0)) {
    targetTemperature++;
  }

  if (changedbits & (1 << PINB1)&& PINB & (1<<PINB1)) {
    isActivated = !isActivated;
    if (!isActivated) {
      digitalWrite(coolerFan, LOW); //desliga o cooler
      digitalWrite(resistencePin, LOW); //desliga a resistencia de aquecimento
    }
  }

  if (changedbits & (1 << PINB2)) {
    targetTemperature--;
  }
}

void draw() 
{
  //Comandos graficos para o display devem ser colocados aqui
  //Seleciona a fonte de texto
  u8g.setFont(u8g_font_8x13B);
  //Linha superior - temperatura 
  String tempTarget = String(targetTemperature, DEC);
  if(isActivated){
    tempTarget += "/";
    tempTarget += String(actualTemperature, DEC);
  }
  tempTarget += "C";
  u8g.drawStr( 5, 15, tempTarget.c_str());
  //Hora
  u8g.setFont(u8g_font_fub30);
  u8g.drawStr( 10, 57, "09:35");
  //Texto - AM
  u8g.setFont(u8g_font_5x7);
  u8g.drawStr( 115, 33, "AM");
  //moldura relogio
  u8g.drawRFrame(0,18, 128, 46, 4);
}
