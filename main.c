#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>


/*----------------------------------------------------------------------
 Настройки скорости передачи данных по UART
----------------------------------------------------------------------*/
#define USART_BAUDRATE  ( 9600 )
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

/*----------------------------------------------------------------------
 Концигурирование UART
 TX на порту PE1, RX на порту PE0
 Включено прерывание по RX.
 Длина пакета 8 бит + 1 стоп бит.
----------------------------------------------------------------------*/
void init_usart()
{
    DDRE &= ~(1 << PE0);        // RX
    DDRE |=  (1 << PE1);        // TX

    UBRR0H = (uint8_t)(BAUD_PRESCALE >> 8);
    UBRR0L = (uint8_t) BAUD_PRESCALE;

    // Включить приемник и передатчик. Включить прерывание по RX
    UCSR0B =  (1 << TXEN0)
            | (1 << RXEN0)
            | (1 << RXCIE0);

    // Set frame format: 1 stop bit(USBS0), 8 data(UCSZ00)
    UCSR0C =   (0 << UMSEL01) | (0 << UMSEL00) //Async UART
             | (0 << UPM01)   | (0 << UPM00)   //
             | (0 << USBS0)   //0 - one stop bit, 1 - two stop bits
             | (0 << UCSZ02)  | (1 << UCSZ01) | (1 << UCSZ00);  //8 bit
}

/*----------------------------------------------------------------------
 Запись в регистр UDR0 -> передача сообщения
----------------------------------------------------------------------*/
uint8_t USART0_TX (uint8_t aByte)
{
    //UART not ready to TX
    if (!(UCSR0A & (1 << UDRE0)))
    {
        return -1;
    }

    UDR0 = aByte;
    return 0;
}

#define is_signal(bit) (PINE & (1 << bit))

#define RX_1_PORT PE6
#define RX_2_PORT PE7
#define ALL_RX_HIGH ((1 << RX_1_PORT) | (1 << RX_2_PORT))

#define TX_1_PORT PE4
#define TX_2_PORT PE5

#define D_1_PORT PE2
#define D_2_PORT PE3

static       uint8_t numOfPeople = 0; // max 255

uint8_t firstState;
uint8_t currentState = 0;
uint8_t lastState = 0;

/*----------------------------------------------------------------------
Инициализация приемника
----------------------------------------------------------------------*/
void init_receiver()
{
    DDRE &= ~(1 << RX_1_PORT);
    DDRE &= ~(1 << RX_2_PORT);
}
/*----------------------------------------------------------------------
 Инициализация излучателя
----------------------------------------------------------------------*/
void init_transiver()
{
    DDRE |= (1 << TX_1_PORT);
    DDRE |= (1 << TX_2_PORT);
    PORTE &= ~(1 << TX_1_PORT);
    PORTE &= ~(1 << TX_2_PORT);
}
/*----------------------------------------------------------------------
Инициализация светодиодов. С их помощью проводилась проверка работоспособности
----------------------------------------------------------------------*/
void init_diod()
{
    DDRE |= (1 << D_1_PORT);
    DDRE |= (1 << D_2_PORT);
    PORTE &= ~(1 << D_1_PORT);
    PORTE &= ~(1 << D_2_PORT);
}
/*----------------------------------------------------------------------
 Включение порта
----------------------------------------------------------------------*/
void set_port(uint8_t port) {
    PORTE |= (1 << port);
}
/*----------------------------------------------------------------------
Отключение порта
----------------------------------------------------------------------*/
void clr_port(uint8_t port) {
    PORTE &= ~(1 << port);
}
/*----------------------------------------------------------------------
 Регистрация сквозного прохождения
----------------------------------------------------------------------*/
void registerEvent() {
    if (firstState & (1 << RX_1_PORT) && !(lastState & (1 << RX_1_PORT))) numOfPeople++;
    else if (!( firstState & (1 << RX_1_PORT)) && lastState & (1 << RX_1_PORT)) numOfPeople--;
    if (numOfPeople & 1) set_port(D_1_PORT);
    else clr_port(D_1_PORT);
    if (numOfPeople & 2) set_port(D_2_PORT);
    else clr_port(D_2_PORT);
}

int main(void)
{
    init_receiver();
    init_transiver();
    init_diod();
    init_usart();


/*----------------------------------------------------------------------
 Переменное включение двух независимых лучей излучателя
----------------------------------------------------------------------*/
    while (1)
    {
        set_port(TX_1_PORT);
        _delay_us(200);

        currentState = PINE & (1 << RX_1_PORT);

        clr_port(TX_1_PORT);

        _delay_us(100);
        set_port(TX_2_PORT);
        _delay_us(200);

        currentState |= PINE & (1 << RX_2_PORT);

        clr_port(TX_2_PORT);
        _delay_us(1500);
/*----------------------------------------------------------------------
Проверка первого и последнего состояний приемника, на основе которых
регистрируем событием
----------------------------------------------------------------------*/
        if (currentState != ALL_RX_HIGH && lastState == ALL_RX_HIGH)
            firstState = currentState;
        if (currentState == ALL_RX_HIGH && lastState != ALL_RX_HIGH)
            registerEvent();
        lastState = currentState;
    }
}




