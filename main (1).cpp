#include <Arduino.h>
#include <ESP32SPISlave.h>
#include <LiquidCrystal.h>
#include <cstring>
#include <Wire.h>

//definiciones
#define I2C_DEV_ADDR 0x55 

#define LR 17
#define LG 16
#define LB 0

#define rs 32
#define en 33
#define d4 25
#define d5 26
#define d6 27
#define d7 13

#define QUEUE_SIZE 1 //procesar un valor a la vez
ESP32SPISlave slave; // definicion como esclavo

#define POT1 15


// Variables globales
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
volatile char menu = 0;
volatile char menu2 = 0;
volatile char indicador1 = 0;
volatile uint8_t indicador2 = 0;
int32_t tiempo = 0;

static constexpr size_t BUFFER_SIZE = 6;
uint8_t tx_buf[BUFFER_SIZE]; // buffer transmitir
uint8_t rx_buf[BUFFER_SIZE]; //buffer recibir

volatile uint32_t TiempoU = 0; 
volatile uint32_t TiempoEn = 0;

volatile int PT1 = 0;

float MAP = 0;
volatile int32_t MAP2 = 0;
float Volt = 0;
volatile int32_t i2c_tx_value = 0;


//ISRs - Rutinas de interrupcion

void onRequest(); //cuando Master pide datos
void onReceive(int len); //cuando el Master envia datos


void setup() {

    Serial.begin(115200);
    lcd.begin(16,2); 
    //comunicacion I2C 
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
    Wire.begin((uint8_t)I2C_DEV_ADDR);
    //...........................
    pinMode(LR, OUTPUT);
    pinMode(LG, OUTPUT);
    pinMode(LB, OUTPUT);
    //Configuarcion SPI
    slave.setDataMode(SPI_MODE0);
    slave.setQueueSize(QUEUE_SIZE);

    slave.begin(VSPI);

    tx_buf[0] = indicador2;

    slave.queue(tx_buf, rx_buf, BUFFER_SIZE); //espero a que Master inicie la comunicacion
    slave.trigger(); //espera
    //LCD
    lcd.setCursor(0, 0);
    lcd.print("PT1:");
    lcd.setCursor(6, 0);
    lcd.print("PT1V:");
    lcd.setCursor(12, 0);
    lcd.print("RGB:");
}


void loop() {

    if (indicador2 == 1) { //si se recibe un 1 del master
        if (millis() - TiempoEn >= tiempo) { //se comprubea si ta paso el timepo del encendido que se pedia ya paso y apaga las leds
            digitalWrite(LR, LOW);
            digitalWrite(LG, LOW);
            digitalWrite(LB, LOW);
            indicador2 = 0; //resetea la bandera
        }
    }

    //comprueba si el Master ha enviado datos y si se completo la comunicacion
    if (slave.hasTransactionsCompletedAndAllResultsReady(QUEUE_SIZE)) {
        Serial.println("Se completaron todas las transacciones");

        size_t received_bytes = slave.numBytesReceived();
        //obtiene cuantos bytes se recibieron
        if (received_bytes >= BUFFER_SIZE) { //si si se recibieron los bytes que se deseaban
            menu = rx_buf[0]; //primero que vamos hacer en este caso si es 1 sequieren controlar las leds
            indicador1 = rx_buf[1]; //cual led vamos a prender
            
            tiempo = 0; //se recontruye por completo cual ese el tiempo que se mando pues este se parte en bytes 
            tiempo |= ((int32_t)rx_buf[2] << 24);
            tiempo |= ((int32_t)rx_buf[3] << 16);
            tiempo |= ((int32_t)rx_buf[4] << 8);
            tiempo |= ((int32_t)rx_buf[5] << 0);
        }

        if (menu == 1) { //si se recibio un 1 de la parte anterior
            if (indicador1 == 1) { //se compara cual led es la que se desea
                indicador2 = 1;
                TiempoEn = millis(); //guarda el tiempo que se encendio la led y este se compara en la funcion anterior
                digitalWrite(LR, HIGH);
                digitalWrite(LG, LOW);
                digitalWrite(LB, LOW);
                
            } else if (indicador1 == 2) {
                indicador2 = 1;
                TiempoEn = millis();
                digitalWrite(LR, LOW);
                digitalWrite(LG, HIGH);
                digitalWrite(LB, LOW);

            } else if (indicador1 == 3) {
                indicador2 = 1;
                TiempoEn = millis();
                digitalWrite(LR, LOW);
                digitalWrite(LG, LOW);
                digitalWrite(LB, HIGH);
            }
            menu=0; //reseteo de banderas
            lcd.setCursor(12, 1);
            lcd.print("   ");
            lcd.setCursor(12, 1);
            lcd.print((int)indicador1); //indicacion de que led se encendio
        }

    }


    if (slave.hasTransactionsCompletedAndAllResultsHandled()) { //comprueba si el mensaje anterior ya fue manejado
        
        tx_buf[0] = indicador2; //nuevamente para ver que se recibe
        slave.queue(tx_buf, rx_buf, BUFFER_SIZE);
        slave.trigger(); //se vuelve a preparar para el siguiente mensaje
    }
    //siempre se lee el mensaje
    PT1 = analogRead(POT1);
    MAP = map(PT1, 0, 4095, 0, 3300); //cambio para poder transformarlo a V
    MAP2 = map(PT1, 0, 4095, 0, 255); //rango 0-255
    Volt = MAP / 1000.0; //con esta division se consigue el rango de 0-3.30 V
    
    if (menu2 == 2) { //si el master envio 2
        uint32_t currentTime = millis(); //se obtiene el tiempo actual

        if (currentTime - TiempoU > 200) { //actualizadr la LCD

            lcd.setCursor(0, 1);
            lcd.print("   ");
            lcd.setCursor(0, 1);
            lcd.print(MAP2);

            lcd.setCursor(6, 1);
            lcd.print("   ");
            lcd.setCursor(6, 1);
            lcd.print(Volt);

            TiempoU = currentTime;
            menu2=0;
        }
    }
}

void onRequest() { //el master pide dato
    Wire.write((uint8_t*)&MAP2, sizeof(MAP2)); //se escribe el dato del valor del potenciometro para mandarlo al master
    Serial.println(MAP2); //debugeo
}
void onReceive(int len) { //mientras haya bytes disponibles en el buffer de I2C
    while (Wire.available()) {
        menu2 = Wire.read(); //lee el byte de comando y lo guarda y estoa activa la seccion de esta comunicacion
 }
}