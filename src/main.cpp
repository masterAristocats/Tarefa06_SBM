#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define set_bit(reg, pin)    (reg |= (1<<pin))
#define clr_bit(reg, pin)    (reg &= ~(1<<pin))
#define toggle_bit(reg, pin) (reg ^= (1<<pin))
#define tst_bit(reg, pin)    (reg & (1<<pin))


#define NUM_SENSORS 2
#define FAST        1
#define NORMAL      0


#define FCLOCK 16000000UL

#define SIZEBUFF    20


void setup(void);
void loop(void);
void f_timers(void);
void read_keyb(void);
void mux_display(void);
uint16_t adc_read(void);
void adc_conversion_ch(uint8_t channel);
void adc_maq(void);
void uart_setup(uint32_t bps, uint8_t mode);
void uart_send_char(uint8_t chr);
void uart_send_string(uint8_t *str);
void uart_send_dec(uint16_t data);
void uart_send_hex(uint8_t valor);
uint8_t uart_receive(void);
int8_t uart_count(void);

int16_t milliseconds = 0;
uint16_t AD[NUM_SENSORS] = {0};


uint8_t hex_numbers[] = 
{
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01100111, // 9
  0b01110111, // A
  0b01111100, // b
  0b01011000, // c
  0b01011110, // d
  0b01111001, // E
  0b01110001  // F
};


volatile uint8_t usart_run = 1;                //1 para usart rodando, 0 para usart ocupada
volatile char rx_buffer[SIZEBUFF] = {0x00};    //buffer para recepção serial
volatile uint16_t rx_buff = 0;                 //contador de bytes
uint8_t byte_sent = 0;


ISR(USART_RX_vect) 
{
    static uint16_t cnt_buff = 0;                //contador local de endereço para o buffer rx
  
    //if(byte_sent)  //envio somente de caractere
    //{
    //    cnt_buff = 0;
    //    byte_sent = 0;
    //}
    rx_buffer[cnt_buff] = UDR0;                  //armazena o byte recebido no endereço atual do buffer de rx
    rx_buff++;                                   //incrementa o contador global de bytes recebidos
    cnt_buff++;                                  //incrementa o contador local de endereços para o buffer de rx
    
    if(cnt_buff>=SIZEBUFF)
      cnt_buff = 0;
}

ISR(USART_TX_vect)
{
  usart_run = 1;                               //ocorrência da transmissão	
}

ISR(PCINT1_vect) //houve alteracao de nivel logico
{
  read_keyb();
}

ISR(ADC_vect)
{
  adc_maq();
}

ISR(TIMER0_OVF_vect)
{
  TCNT0 = 6;
  f_timers();
}

int main()
{
  setup();

  while(1) loop();
}

void setup(void)
{
  cli(); //desabilta a chave geral de interrupcao
  DDRD   = 0b11111111;  //PD2 e PD3 como entrada
  DDRB   = 0b00111110; 
  DDRC = 0xfd; //PC0 e PC1 como entradas
  PORTD  = 0;  //inicia apagado
  PORTB  = 0;
  ADMUX  = 0b01000000; //referencia AVCC de 5V, justificado a direita nenhum canal selecionado (ADC0 selecionado por padrao)
  ADCSRA = 0b10101101; //AD habilitado, interrupcao habilitada, PS de 32 para ADC Clock de 250 kHz
  ADCSRB = 0x00; //modo livre
  TCCR0B = 0x03; // 0b00000011 //3
  TCNT0 = 6;
  TIMSK0 = 0x01; //habilita a interrupcao do timer 0
  TCCR1A = 0b10100011;
  TCCR1B = 0b00001010; //PWM fast mode de 10 bits comprescaler de 8
  PCICR = 0x02; //00000010
  PCMSK1 = (1<<PC0); //habilito a interrupcao na porta PC0
  adc_maq();
  uart_setup(115200, FAST);
  sei(); //habilta a chave geral de interrupcao
}


void uart_setup(uint32_t bps, uint8_t mode)
{
  if(mode == FAST)
  {
    UCSR0A |= (1<<U2X0);
    UBRR0 = (FCLOCK/(8UL*bps))-1;
  }

  else
  {
    UCSR0A &= ~(1<<U2X0);
    UBRR0 = (FCLOCK/(16UL*bps))-1;
  }

  UCSR0B = 0b11011000; //habilito interrupcao de TX e RX e habilito TX e RX

  UCSR0C = 0b00000110; //8 bits de dados, 1 stop bit e sem bit de paridade
}

void loop(void)
{
  
}

void f_timers(void)
{
  static uint16_t counter0 = 1, counter1 = 1, counter2 = 1;
  static uint8_t buffer[SIZEBUFF];
  //static uint8_t i = 0;
  static uint8_t i = 0;
  static uint8_t data_rx = 0;

  if(counter0 < 400)
  {
    counter0++;
  }

  else
  {
    
    if(uart_count() > 0)
    {
      data_rx = uart_receive();
      i = 0;
    }


    switch (data_rx)
    {
      case '0':
        PORTD = 1;
        break;

      case '1':
        PORTD = 255;
        break;

      case '2':
        PORTD = i;
        i++;
        break;

      case '3':
        PORTD = (1<<i);
        i++;
        if(i > 7) i = 0;
        break;

      case '4':
        PORTD = (1<<i);
        if(!i) i = 7;
        else
          i--;
        break;


      default:
        break;
    }

    // uart_send_dec(AD[0]);
    // uart_send_char('\n');
    // sprintf((char *)buffer, "%d\n", AD[0]);
    // uart_send_string(buffer);
    // uart_send_char(i+48);
    // uart_send_char('\n');
    // i++;
    // if(i>9)
    //   i = 0;

    counter0 = 1;
  }

  if(counter1 < 10)
  {
    counter1++;
  }

  else
  {
    counter1 = 1;
  }

  if(counter2 < 2000)
  {
    counter2++;
  }

  else
  {
    //i ^= 1;
    counter2 = 1;
  }

}

void read_keyb(void)
{
  static uint8_t memory_button1 = 0, button1 = 0;

  if(tst_bit(PINC, PC0))
  {
    button1 = 1;
  }

  else
  {
    button1 = 0;
  }

  if(button1 < memory_button1)
  {   

  }

  memory_button1 = button1;
}

uint16_t adc_read(void)
{
  unsigned int dado = (ADCH<<8) | ADCL;
  return dado;
}

void adc_conversion_ch(uint8_t channel)
{
  ADMUX &= 0xf0;
  ADMUX |= (channel & 0x0f);
  
  ADCSRA |= 0x40;//inicio a conversao
}

void adc_maq(void)
{
  static uint8_t estado = 10;
  
  switch (estado) {
      
    case 0:
      estado = 1;
      AD[0] = adc_read();
      adc_conversion_ch(1);
      break;
        
    case 1:
      estado = 0;
      AD[1] = adc_read();
      adc_conversion_ch(0);
      break;

    default:
      estado = 0;
      adc_conversion_ch(1);
      AD[0] = adc_read();
      break; 
  }    
}

void uart_send_char(uint8_t chr)
{
  UDR0 = chr;
    //Aguarda o buffer ser desocupado
  while (!(UCSR0A & (1<<UDRE0)));                 //com polling

  // while(!usart_run);                           //aguarda usart liberar  //por interrupção
  // usart_run = 0;                               //ocupa usart
  // UDR0 = chr;
}

void uart_send_string(uint8_t *str)
{
  uint16_t i=0;
  for(i=0;str[i]!='\0';i++)
    uart_send_char(str[i]);
}

void uart_send_dec(uint16_t data)
{
  static uint8_t cenmil = 0, dezmil = 0, mil = 0, cen = 0, dez = 0, uni = 0;

  cenmil = data / 100000;
  dezmil = (data%100000) / 10000;
  mil = (data % 10000) / 1000;
  cen = (data%1000) / 100;
  dez = (data%100)/10;
  uni = (data%10);
  
  uart_send_char(cenmil + 48);
  uart_send_char(dezmil + 48);
  uart_send_char(mil + 48);
  uart_send_char(cen + 48);
  uart_send_char(dez + 48);
  uart_send_char(uni + 48);
}

void uart_send_hex(uint8_t valor)
{   
  uint8_t i,temp;
  for (i=0; i<2; i++)
  {
    temp = (valor & 0xF0)>>4;
    if ( temp <= 9)
      uart_send_char('0' + temp);
    else
      uart_send_char('A' + temp - 10);
    valor = valor << 4;    
  }
}

uint8_t uart_receive(void)
{
  static uint16_t buff_local = 0;
  char byte_rx = 0;

  byte_rx = rx_buffer[buff_local];
  buff_local++;
  rx_buff--;
  
  if(buff_local>=SIZEBUFF)
    buff_local = 0;
  
  return byte_rx;
}

int8_t uart_count(void)
{   
  return rx_buff;
}