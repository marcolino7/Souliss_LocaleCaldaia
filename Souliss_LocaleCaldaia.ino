/**************************************************************************
	--- Souliss ---
    -- #BUILD:4 --
	-- Souliss Friariello -- 

        Controllo locale Caldaia
        - Servomotore per la temperatura dell'acqua
        - Uscita per Abilitare il Servomotore
		- Ingresso Digitale per il controllo del termostato
		- Uscita Digitale per il controllo della linea termostato della caldaia
        - NTC1 temperatura acqua mandata all'impianto
        - NTC2 temperatura acqua ritorno dall'impianto
		- Relè controllo ventola aspiratore
		- Sensore fughe GAS

		D8: Servo Enable
		D9: Controllo Servomotore
		D6: Pulsante Aspiratore
		D7: Controllo Aspiratore


		'Ciseco Remote Programming
		'Node Address = 02
		'Channel Offset = 3
		'BaudRate = 57600

***************************************************************************/

#define USARTDRIVER_INSKETCH
#define	USARTDRIVER				Serial	//Dico al driver vNet di usare la seriale 0 dell'UNO
#define USART_TXENABLE			0
#define USART_TXENPIN			3

#define USART_DEBUG  			0

#include "bconf/StandardArduino.h"			// Use a standard Arduino
#include "conf/usart.h"

#include "Souliss.h"
#include <SPI.h>

#include <Servo.h>
#include <Termistore.h>
#include <EEPROM.h>
#include <dht.h>

// network addresses
#define myvNet_address		0xCE02
#define myvNet_subnet		0xFF00
#define myvNet_supern		0x0000

#define LEDCONTROL		0   //T19 ID Logico del Led del Tipico 19
#define SERVOPOS		1   //T19 ID Logico del PWM del Tipico 19
#define TERMOSTATOIN    2	//T13 ID Logico dell'ingresso Termostato
#define TERMOSTATOOUT	3	//T12 ID Logico dell'uscita controllo caldaia
#define ANALOGDAQ1		4   //T52 ID Logico del NTC 1 acqua uscita Caldaia
#define ANALOGDAQ2		6	//T52 ID Logico del NTC 2 acqua ritorno impianto	
#define FAN				8   //T11 ID Logico della Linea che alimenta l'aspiratore
#define GASDETECTOR		9	//T13 ID Logico dell'ingresso Rilevatore del gas
#define DHT22_TEMP	   10	//T52 ID Logico della lattura della temperatura sul DHT22
#define DHT22_HUMI	   12	//T53 ID Logico della lettura dell'umidità sul DHT22

#define DEADBAND      0.05 //Se la variazione è superio del 5% aggiorno
#define DEADBANDNTC   0.01 //Se la variazione è superio del 1% aggiorno
#define DEADBANDLOW	  0.005

//Definisco i vari PIN di Arduino
#define PIN_SERVOBUTT		2
#define PIN_SERVOEN			8
#define PIN_SERVO			9
#define PIN_TERMOSTATO		5
#define PIN_CALDAIABUTT		3
#define PIN_CALDAIAEN		4
#define PIN_FANBUTT			6
#define	PIN_FANEN			7
#define	PIN_GASDETEC		16	//A2 IN DIGITALE
#define PIN_A_NTC1			0	//A0
#define PIN_A_NTC2			1	//A1
#define PIN_DHT22			18	//A4 in Digitale

//Posizioni EEPROM
#define EPR_SERVO			1

// Identify the sensor, in case of more than one used on the same board	
dht DHT;

//Definisco il primo termistore sulla porta A0 e il seconcdo sulla A1
//E creo le variabili per contenere il ritorno
Termistore ntc1(PIN_A_NTC1);
Termistore ntc2(PIN_A_NTC2);
float t_ntc1;
float t_ntc2;

// create servo object to control a servo
// and variable to store the servo position
Servo myservo;
byte pos = 0;
//Variabile contenete la posizione del servo per gestire la EEPROM
//e la imposto a Zero
uint8_t servo_eeprom_pos=0;

//Variabile per memorizzare il valore dell'ingresso termostato
//E la persistenza della lettura
byte b_tin = 0;
byte b_persist = 0;

//Variabile per memorizzare il valore dell'ingresso Allarme fughe GAS
//E la persistenza della lettura
byte b_gasall = 0;
byte b_gaspers = 0;

//Inizializzo il DHT22
//ssDHT22_Init(PIN_DHT22, DHT_id1);

// flag 
U8 data_chg = 1;

void setup()
{	
	Souliss_SetAddress(myvNet_address, myvNet_subnet, myvNet_supern);		

	// Tipico T19 per il controllo del Servomotore
	Souliss_SetT19(memory_map, LEDCONTROL);
	
	// Tipico T52 per la lettura della sonda NTC1 e NTC2
	Souliss_SetT52(memory_map, ANALOGDAQ1);
	Souliss_SetT52(memory_map, ANALOGDAQ2);

	//Tipico T12 controllo caldaia
	Souliss_SetT12(memory_map, TERMOSTATOOUT);

	//Tipico T13 per l'ingresso analogico
	Souliss_SetT13(memory_map, TERMOSTATOIN);

	// Tipico T11 per il controllo della ventola
	Souliss_SetT11(memory_map, FAN);

	//Tipico T13 rilevatore di GAS
	Souliss_SetT13(memory_map, GASDETECTOR);


	//Ingresso DHT22
	//pinMode(PIN_DHT22, INPUT_PULLUP);
	//digitalWrite(PIN_DHT22,HIGH);

	//T52 Temperatur DHT
	Souliss_SetT52(memory_map, DHT22_TEMP);
	//T53 Umidità
	Souliss_SetT53(memory_map, DHT22_HUMI);

	//ssDHT_Begin(DHT_id1);


	//-------Servomotore
	pinMode(PIN_SERVOBUTT, INPUT);      // NON USATO - Pulsante di accensione T19 Hardware pulldown required
	pinMode(PIN_SERVOEN, OUTPUT);		// Abilita servomotore
	myservo.attach(PIN_SERVO);		// attaches the servo on pin 9 to the servo object
	//-------Ingresso Termostato
	pinMode(PIN_TERMOSTATO, INPUT);
	//-------Controllo Linea Termostato
	pinMode(PIN_CALDAIABUTT, INPUT);		//Pulsante HW Termostato
	pinMode(PIN_CALDAIAEN, OUTPUT);		//Controllo Caldaia
	//------Aspiratore
	pinMode(PIN_FANBUTT, INPUT);      // Pulsante HW Fan
	pinMode(PIN_FANEN, OUTPUT);     // Fan
	//-------Ingresso Rilevatore di GAS
	pinMode(PIN_GASDETEC, INPUT);
	//Ingresso DHT22
	//pinMode(PIN_DHT22, INPUT);
		
	//Leggo dalla EEPROM il la posizione del servomotore e lo imposto in souliss
	servo_eeprom_pos = EEPROM.read(EPR_SERVO);
	pos = servo_eeprom_pos;
	mOutput(SERVOPOS) = servo_eeprom_pos;

}

void loop()
{
	EXECUTEFAST() {						
		UPDATEFAST();
		FAST_50ms() {	// We process the logic and relevant input and output every 50 milliseconds
			//-------- T19 Controllo Servo
			// Use Pin2 as ON command, Pin2 as ON-OFF command
			Souliss_DigIn(PIN_SERVOBUTT, Souliss_T1n_ToogleCmd, memory_map, LEDCONTROL);
			// Execute the logic that handle the LED
			Souliss_Logic_T19_Bis(memory_map, LEDCONTROL, &data_chg);
            //Souliss_Logic_T19(memory_map, LEDCONTROL, &data_chg);
			// Use the output values to control the PWM
			// metto nella variabile pos, la posizione dello slide
			pos = mOutput(SERVOPOS);
			Souliss_DigOut(PIN_SERVOEN, Souliss_T1n_Coil, memory_map, LEDCONTROL);

			//-------- T12 Controllo Caldaia pin 3 pulsante pin 4 Out
			Souliss_DigIn(PIN_CALDAIABUTT, Souliss_T1n_ToogleCmd, memory_map, TERMOSTATOOUT);		
			Souliss_DigOut(PIN_CALDAIAEN, Souliss_T1n_Coil, memory_map, TERMOSTATOOUT);	

			//-------- T11 Aspiratore pin 6 pulsante, pin 7 Out
			Souliss_DigIn(PIN_FANBUTT, Souliss_T1n_ToogleCmd, memory_map, FAN);		
			Souliss_DigOut(PIN_FANEN, Souliss_T1n_Coil, memory_map, FAN);	
		}

		FAST_70ms() {
			//Muovo il servomotore
			move_servo();
		}

		FAST_90ms() { 
			//Esegui Logic per il controllo caldaia
			Souliss_Logic_T12(memory_map, TERMOSTATOOUT, &data_chg);
			//Logica Ingresso Termostato
			Souliss_Logic_T13(memory_map, TERMOSTATOIN,&data_chg);
			//Logica Ingresso Allarme GAS
			Souliss_Logic_T13(memory_map, GASDETECTOR,&data_chg);

			// Esegui Logic per la linea FAN
			Souliss_Logic_T11(memory_map, FAN, &data_chg);

            // Compare the acquired input with the stored one, send the new value to the
            // user interface if the difference is greater than the deadband
            Souliss_Logic_T52(memory_map, ANALOGDAQ1, DEADBANDNTC, &data_chg);
			Souliss_Logic_T52(memory_map, ANALOGDAQ2, DEADBANDNTC, &data_chg);
		}

		FAST_110ms() {
			// Retreive data from the communication channel
			Souliss_CommunicationData(memory_map, &data_chg);
		}

		FAST_510ms() {	// We retrieve data from the node with index 1 (peervNet_address)

			//-------- T13 Lettura Ingresso Termostato
			Souliss_LowDigIn2State(PIN_TERMOSTATO,Souliss_T1n_OnCmd,Souliss_T1n_OffCmd,memory_map,TERMOSTATOIN);		

			//Codice per gestire il termostato solo su cambio di stato
			b_tin = mOutput(TERMOSTATOIN);
			if (b_tin != b_persist) {	//L'ingresso è cambiato e gestisco il cambio del termostato
				if (b_tin == 0) {		//Mando il comando di OFF alla caldaia sul Tipico 12
					mInput(TERMOSTATOOUT) = Souliss_T1n_OffCmd;
				} 
				else if (b_tin == 1)	//Mando il comando di ON alla caldaia sul Tipico 12
				{
					mInput(TERMOSTATOOUT) = Souliss_T1n_OnCmd;
				}
				b_persist = b_tin;				
			}

			//T13 Lettura Stato Allarme GAS
			Souliss_LowDigIn2State(PIN_GASDETEC,Souliss_T1n_OnCmd,Souliss_T1n_OffCmd,memory_map,GASDETECTOR);		
			//Codice per gestire l'allarme GAS sul Cambio di Stato
			b_gasall = mOutput(GASDETECTOR);
			if (b_gasall != b_gaspers) {	//L'ingresso è cambiato e gestisco
				if (b_gasall == 0) {		//Mando il comando di OFF alla Ventola
					mInput(FAN) = Souliss_T1n_OffCmd;
					//Ritardo del Timer
					//Souliss_Input(memory_map, FAN)= 0x31;
				} 
				else if (b_gasall == 1)	//Mando il comando di ON alla ventola
				{
					mInput(FAN) = Souliss_T1n_OnCmd;
				}
				b_gaspers = b_gasall;				
			}


		}

		FAST_1110ms() {
			
			Souliss_Logic_T52(memory_map, DHT22_TEMP, DEADBANDLOW, &data_changed);
			Souliss_Logic_T53(memory_map, DHT22_HUMI, DEADBANDLOW, &data_changed);
			
			//Souliss_T11_Timer(memory_map, FAN);
		
		}

        FAST_2110ms() {
            //Salvo la posizione del servo nella EEPROM, solo se viene variato.
            if((servo_eeprom_pos != pos) && (abs(servo_eeprom_pos-pos) > 10)) 
            {
                //Souliss_Input(memory_map,TERMOSTATOOUT) = Souliss_T1n_ToogleCmd;
                EEPROM.write(EPR_SERVO, pos);
                servo_eeprom_pos=pos;
            }
        }

}
	
	EXECUTESLOW() {
		UPDATESLOW();

		SLOW_10s() {		// We handle the light timer with a 10 seconds base time
			NTCRead();	//Leggo gli NTC e scrivo le varie variabili
			// Timer ger gestire il ritardo di spegnimento della ventola
			// Con la combinazione di 0x32 e 10s, ottngo circa 7 minuti e mezzo
			//Souliss_T11_Timer(memory_map, FAN);
			DHTRead();
		}
	}		
}

void NTCRead() {
			// Acquire data from Thermistors
			t_ntc1 = ntc1.getTemp();
			t_ntc2 = ntc2.getTemp();
			// Acquire data from the microcontroller ADC
			Souliss_ImportAnalog(memory_map, ANALOGDAQ1, &t_ntc1);
			Souliss_ImportAnalog(memory_map, ANALOGDAQ2, &t_ntc2);
}

void DHTRead() {
	int chk = DHT.read22(PIN_DHT22);
	if (chk == DHTLIB_OK) {
		float temperature = DHT.temperature;
		Souliss_ImportAnalog(memory_map, DHT22_TEMP, &temperature);

		float humidity = DHT.humidity;
		Souliss_ImportAnalog(memory_map, DHT22_HUMI, &humidity);
	}
}

void move_servo() {

			byte pos_servo=0;
			//Map Slide position to Servo Degrees
			pos_servo = map(pos, 0, 254, 0, 180);
			// Servo positiong
			myservo.write(pos_servo);
}


/**************************************************************************
/*
	T19 logic bis, to handle a servo motor
*/	
/**************************************************************************/
void Souliss_Logic_T19_Bis(U8 *memory_map, U8 slot, U8 *trigger)
{
    // Look for input value, update output. If the output is not set, trig a data
    // change, otherwise just reset the input
    
    if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_ToogleCmd)        // Toogle Command
    {
        // Toogle the actual status of the light
        if(memory_map[MaCaco_OUT_s + slot] == Souliss_T1n_OffCoil)        
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_OnCmd;            
        else if(memory_map[MaCaco_OUT_s + slot] == Souliss_T1n_OnCoil)
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_OffCmd;
        else
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;
    }
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_OffCmd)        // Off Command
    {
        // Trigger the change and save the actual color
        if(memory_map[MaCaco_OUT_s + slot] != Souliss_T1n_OffCoil)  
        {                
            memory_map[MaCaco_OUT_s + slot] = Souliss_T1n_OffCoil;        // Switch off the light state
            *trigger = Souliss_TRIGGED;                                    // Trig the change
        }
 
        // Once is off, reset
        if((memory_map[MaCaco_OUT_s + slot + 1] == 0))
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;        // Reset
    }
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_OnCmd)
    {
        if(memory_map[MaCaco_OUT_s + slot] != Souliss_T1n_OnCoil)  
            *trigger = Souliss_TRIGGED;    
    
        memory_map[MaCaco_OUT_s + slot] = Souliss_T1n_OnCoil;            // Switch on the output
        
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset
    }
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_BrightUp)        // Increase the light value 
    {
        // Increase the light value
        if(memory_map[MaCaco_OUT_s + slot + 1] < 255 - Souliss_T1n_BrightValue) 
            memory_map[MaCaco_OUT_s + slot + 1] += Souliss_T1n_BrightValue;
        
        memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset
    }
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_BrightDown)                // Decrease the light value
    {
        // Decrease the light value
        if(memory_map[MaCaco_OUT_s + slot + 1] > Souliss_T1n_BrightValue) 
            memory_map[MaCaco_OUT_s + slot + 1] -= Souliss_T1n_BrightValue;
            
        memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset
    }    
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_Set)
    {    
        // Set the new color
        memory_map[MaCaco_OUT_s + slot + 1] = memory_map[MaCaco_IN_s + slot + 1];
        memory_map[MaCaco_AUXIN_s + slot + 1] = memory_map[MaCaco_OUT_s + slot + 1];
        memory_map[MaCaco_IN_s + slot + 1] = Souliss_T1n_RstCmd;
        
        memory_map[MaCaco_AUXIN_s + slot] = Souliss_T1n_Timed;            // Set a timer for the state notification        
        memory_map[MaCaco_OUT_s + slot] = Souliss_T1n_OnCoil;            // Switch on the output
        memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset        
    }
    else
    {    // There is no command
        
        if(memory_map[MaCaco_AUXIN_s + slot] > Souliss_T1n_Set)            // Decrese the timer value
            memory_map[MaCaco_AUXIN_s + slot]--;
        else if(memory_map[MaCaco_AUXIN_s + slot] > 0)                    // If we not getting new commands, the burst        
        {                                                                // is done, send the actual state 
            memory_map[MaCaco_AUXIN_s + slot] = 0;
            *trigger = Souliss_TRIGGED;                                    
        }    
    }    
}
