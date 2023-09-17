/******************************************************************************************************************************************
 ******************************************************************************************************************************************
 ******************************************************************************************************************************************  
 ******************************************************************************************************************************************  
 *                    
 *                    REBALANCING SYSTEM FOR REDOW FLOW BATTERY
 *                    
      Author: MIGUEL CANTERA - UNIVERSITY OF BURGOS (SPAIN)

      This code version uses timers. Future versions will use a Real Time Clock.
  
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <TimerOne.h>
#include <TimerThree.h>

const int PCSERIALSPEED = 9600;
const float factorEscalaADC = 0.0001250;
const int MILLISINSEC = 1000;
const long TIMER1TIME = 2000000;
const long TIMER3TIME = 500000;
const int TIMEINTERVALMS = 2000;
const int NUMELEMMEDIA = 4;
const int NUMELEMSOC = 10;
const int NUMCYCLESLIMIT = 750;
const int PERCENTAGEPERIODMIN = 95;
const long DELAYTIMEREBALANCEON = 600000;
const long BALONINTERVALTIME = 720000;
const long LASTINTERVALTIME = 600000;
const float VOLTAGELIMIT = 1.4200;
const float EPSILON = 0.0001;

// state machine to detect if it is a charge cycle or discharge cycle
enum transicionesSoc {
INICIO,
CARGA,
DESCARGA,
};

// state machine for the rebalancing device, it activates the output relay
enum transicionesBal {
INIT,
DELAY,
LAST10,
BALANCING,
};

transicionesSoc estadoSoc = INICIO;
transicionesBal estadoBal = INIT;

//******************************************
// **********     INPUTS       **********
//******************************************
Adafruit_ADS1115 ads1115_Bateria;

//******************************************
// **********     VARIABLES      **********
//******************************************
unsigned long tiempoEjecucion = 0;
//ADC ADS1115 in differential mode returns a 16 bit integer
float previoADCValue;
float ADCValue;
float arrayMedia[NUMELEMMEDIA];
float media = 0;
float mediavslimit = 0;
char mediaStr[6];
float previamedia = 0;
float derivadaSimple = 0;
float derivadaMedia = 0;
int contadorMedia = 0;
int contadorSOC = 0;
int tiempoentremuestras = 0;
int SOCState = 0; // 0 -> init; 1 -> charge; 2 -> discharge
int numCiclosUp = 0;
int numCiclosDown = 0;
unsigned long epocaInicioCarga = 0;
unsigned long epocaInicioDescarga = 0;
unsigned long periodoCicloCargaActual = 0;
unsigned long periodoCicloDescargaActual = 0;
unsigned long periodoCicloCargaPrevio = 0;
unsigned long periodoCicloDescargaPrevio = 0;
int periodosCarga[NUMCYCLESLIMIT]; 
int periodosDescarga[NUMCYCLESLIMIT];
unsigned long currentMillis = 0;
unsigned long prevDelayBalOnMillis = 0;
unsigned long prevIntervalBalOnMillis = 0;
unsigned long inicioUltimoSemicicloCarga = 0;
int numSemiciclosCarga = 0;
int numSemiciclosDescarga = 0;
int longestDischargePosition = 0;
int longestDischargePeriod = 0;
int longestChargePosition = 0;
int longestChargePeriod = 0;
int lastBalChargeSemicycle = 0;
int lastBalDischargeSemicycle = 0;
float fpercdiff = 0.0;
int ipercdiff = 0;
bool EstadoBalanceador = false; 
float previousVoltage = 0;
float currentVoltage = 0;
float diffV = 0;
float deltaV = 0;
float ciendeltaV = 0;
int ideltaV = 0;
int numTotalBal = 0;
int DuracionUltimoPeriodoCarga = 0;
bool printInitialDateAndTime = true;
bool EnableTest3;
bool ConditionTest3;
bool fromDischargeToCharge = false;
bool fromChargeToDischarge = false;

//******************************************
// **********     OUTPUTS       **********
//******************************************
// The Relay 1.8 corresponds to PIN 42 of the Arduino Mega board
int SALIDA_BALANCEADOR = 42;

//This function enables/disables the output relay when needed
void Rebalancing() {

  // Only this condition is finally used
  ConditionTest3 =  EnableTest3 &&
                    fromChargeToDischarge &&
                    ipercdiff < PERCENTAGEPERIODMIN;

  switch (estadoBal) {

    case INIT:

      digitalWrite(SALIDA_BALANCEADOR,LOW);
      EstadoBalanceador = false;

      if (ConditionTest3) {

        prevDelayBalOnMillis = currentMillis;

        // For debugging purposes
        Serial.print("\nTEST 3: ");
        Serial.print("\n");
        Serial.print("\nMaquina Estados Balanceador: ");
        Serial.print(estadoBal);
        Serial.print("\nEl porcentaje de capacidad ");
        Serial.print(ipercdiff); 
        Serial.print(" es menor que el limite ");
        Serial.print(PERCENTAGEPERIODMIN);
        Serial.print("% , por lo que se inicia un delay de ");
        Serial.print(DELAYTIMEREBALANCEON/60000);
        Serial.print(" minutos antes de encender el rebalanceador");
        Serial.print("\n");
        Serial.print("\nMinutos inicio: ");
        Serial.print(currentMillis/60000);
        Serial.print("\n");
          
        estadoBal = DELAY;
      }  

      break;

    case DELAY:

      digitalWrite(SALIDA_BALANCEADOR,LOW);
      EstadoBalanceador = false;
      
      if ((currentMillis - prevDelayBalOnMillis) >= DELAYTIMEREBALANCEON) {
        
        prevIntervalBalOnMillis = currentMillis; 

        // For debugging purposes
        Serial.print("\nAcaban de pasar los ");
        Serial.print(DELAYTIMEREBALANCEON/60000);
        Serial.print(" minutos del delay. ");
        Serial.print("\n¡¡¡¡¡¡¡¡    INICIO DE REBALANCEO DE ");
        Serial.print(BALONINTERVALTIME/60000);
        Serial.print(" minutos !!!! ");
        Serial.print("\nMaquina Estado Balanceador: ");
        Serial.print(estadoBal);
        Serial.print("\ncurrent min: ");
        Serial.print(currentMillis/60000); 
        Serial.print("\nprev Delay min: ");
        Serial.print(prevDelayBalOnMillis/60000);
        Serial.print("\n");
        
        estadoBal = BALANCING;
      }
      
      break;

    case LAST10:

      digitalWrite(SALIDA_BALANCEADOR,LOW);
      EstadoBalanceador = false;

      DuracionUltimoPeriodoCarga = periodosCarga[numSemiciclosCarga];
      
      if (currentMillis >= (DuracionUltimoPeriodoCarga - LASTINTERVALTIME - inicioUltimoSemicicloCarga)) {

        prevIntervalBalOnMillis = currentMillis;
       
        Serial.print("\n Quedan ");
        Serial.print((DuracionUltimoPeriodoCarga - LASTINTERVALTIME - inicioUltimoSemicicloCarga)/60);
        Serial.print(" minutos para que finalice el periodo de carga. ");
        Serial.print("\nMaquina Estados Balanceador: ");
        Serial.print(estadoBal);
        Serial.print("\nDuracion ultimo Periodo de Carga: ");
        Serial.print(DuracionUltimoPeriodoCarga/60000);
        Serial.print("\nTiempo inicio del ultimo ciclo de carga: ");
        Serial.print(inicioUltimoSemicicloCarga/60000);
        Serial.print("\nTiempo pasado desde el inicio: ");
        Serial.print(currentMillis/60000);
        
        estadoBal = BALANCING;
      }
      
      break;
      
    case BALANCING:

      digitalWrite(SALIDA_BALANCEADOR,HIGH);
      EstadoBalanceador = true;

      if ((currentMillis - prevIntervalBalOnMillis) >= BALONINTERVALTIME) {

        numTotalBal = numTotalBal + 1;
        
        Serial.print("\n¡¡¡¡Tiempo de balanceo de ");
        Serial.print(BALONINTERVALTIME/60);
        Serial.print(" minutos FINALIZADO!!! ");
        Serial.print("\nMaquina Estados Balanceador: ");
        Serial.print(estadoBal);
        Serial.print("\nTiempo pasado en minutos desde inicio de programa: ");
        Serial.print(currentMillis/60000);
        Serial.print("\nMinutos de duracion de balanceo:");
        Serial.print(prevIntervalBalOnMillis/60000);
        Serial.print("\nNumero total de balanceos desde inicio del programa: ");
        Serial.print(numTotalBal);
        Serial.print("\n");

        estadoBal = INIT;
      }
      
      break;

    default:
      ;
  } 
  
}

// This function detects if it is a charge or discharge cycle and computes its corresponding period (Tch and Tdch)
void MaquinaEstadosSoc() {
  
  switch (estadoSoc) {

    case INICIO:

      fromDischargeToCharge = false;
      fromChargeToDischarge = false;
    
      if (media > 0.5){
        numCiclosUp++;

        numCiclosDown = 0; 
        if (numCiclosUp == 2){ 
          numCiclosUp = 0;
          SOCState = 1;
          epocaInicioCarga = tiempoEjecucion;       
          estadoSoc = CARGA;
        }
      }
      
      if (SOCState == 2){
        epocaInicioDescarga = tiempoEjecucion;
        estadoSoc = DESCARGA;
      }
      
      break;
    // Semiciclo de carga
    case CARGA:

      // Reseteamos el flag
      fromDischargeToCharge = false;
      
      if (mediavslimit > EPSILON){

        numCiclosDown++;

        numCiclosUp = 0;
        
        if (numCiclosDown == 2){ 
          numCiclosDown = 0;
          SOCState = 2; 

          fromChargeToDischarge = true;
        
          periodoCicloCargaPrevio = periodoCicloCargaActual;
          periodoCicloCargaActual = tiempoEjecucion - epocaInicioCarga;
          periodosCarga[numSemiciclosCarga] = periodoCicloCargaActual;
        
          numSemiciclosCarga = numSemiciclosCarga +1;    
          
          if (longestChargePeriod != 0 ){
            fpercdiff = ((float)periodoCicloCargaActual/(float)longestChargePeriod)*100.0;
            ipercdiff = (int)fpercdiff;
      
            // For debugging purposes
            Serial.print("% diferencia Ta vs Tm int: ");
            Serial.print(ipercdiff);
            Serial.print("\n");
            Serial.print("% diferencia Ta vs Tm float: ");
            Serial.print(fpercdiff);
            Serial.print("\n");
          }
          else {
            Serial.print(" Todavia no se ha almacenado un periodo diferente de cero");
            Serial.print("\n");  
          }
  
          epocaInicioDescarga = tiempoEjecucion;
          
          estadoSoc = DESCARGA;
        }
    
      }
      
      break;
      
    case DESCARGA:

      fromChargeToDischarge = false;
      
      if (media < 0.6){

        SOCState = 1;  // descarga

        fromDischargeToCharge = true;
        
        periodoCicloDescargaPrevio = periodoCicloDescargaActual;
        periodoCicloDescargaActual = tiempoEjecucion - epocaInicioDescarga;
        periodosDescarga[numSemiciclosDescarga] = periodoCicloDescargaActual;
        
        numSemiciclosDescarga = numSemiciclosDescarga +1;
        
        epocaInicioCarga = tiempoEjecucion;
        
        estadoSoc = CARGA;
        
      }
        
      break;
      
    default:
      ;
  }
}

// Store values every 0.5 seconds using a timer
void ISR_Store4values() {

  if(contadorMedia < NUMELEMMEDIA){
    arrayMedia[contadorMedia] = ADCValue;
    contadorMedia ++;
  }
  else
  {  
    for(int i = 0; i < (NUMELEMMEDIA-1); i++){
      arrayMedia[i] = arrayMedia[i + 1];
    }
    arrayMedia[(NUMELEMMEDIA-1)] = ADCValue;

  }
}

// Print information every 2 seconds using another timer
void ISR_Compute() {
 
  /////////////////////////////////////////////////////////////
  // Cálculo de la media
  /////////////////////////////////////////////////////////////
  media = 0;
  for(int i = 0; i < NUMELEMMEDIA; i++){
    media = media + arrayMedia[i];
  }
  media = media / NUMELEMMEDIA;

  ////////////////////////////////////////////////////////////////////////////
  // Print information
  ////////////////////////////////////////////////////////////////////////////
  Serial.print(tiempoEjecucion);
  Serial.print(",");
  Serial.print(media,4);
  Serial.print(",");
  Serial.print(deltaV);
  Serial.print(",");
  Serial.print(ideltaV);
  Serial.print(",");
  Serial.print(SOCState);
  Serial.print(",");
  Serial.print(EstadoBalanceador);
  Serial.print("\n");

  diffV = media - previousVoltage;
  previousVoltage = media;
  deltaV = ((diffV)/media)*100;
  ciendeltaV = 100 * deltaV;
  if (ciendeltaV > 0) {
    ideltaV = int(ciendeltaV + 0.5);  
  }
  else {
    ideltaV = int(ciendeltaV - 0.5);  
  }

  mediavslimit = media - VOLTAGELIMIT;

  MaquinaEstadosSoc();

  previoADCValue = ADCValue;
  previamedia = ADCValue; 
}
  
void setup() {
  
  pinMode(SALIDA_BALANCEADOR,OUTPUT);

  Serial.begin (PCSERIALSPEED);

  ads1115_Bateria.begin();
   
  if (!ads1115_Bateria.begin()) {
    Serial.println (F("Error al inicializar el ADC"));
    while (1);
  }

  //Set PGA of the ADS1115 (scale factor of 0.0001250 for maximum voltage of 4,096V)
  ads1115_Bateria.setGain(GAIN_ONE);

  ads1115_Bateria.setDataRate(7);

  /***************************************************/
  /***   TIMER 1   ****/
  Timer1.initialize(TIMER1TIME); 
  Timer1.attachInterrupt(ISR_Compute);

  /***************************************************/
  /***   TIMER 3     ****/
  Timer3.initialize(TIMER3TIME); 
  Timer3.attachInterrupt(ISR_Store4values); 

  // Reference time handoced to 20 minutes (1200 seconds)
  longestChargePeriod = 1200;

}

void loop() {

  // Program exectuion time is printed in the serial monitor
  currentMillis = millis();
  tiempoEjecucion = millis()/MILLISINSEC; 

  // Get the ADC ADS1115 measurement
  ADCValue = factorEscalaADC * ads1115_Bateria.readADC_Differential_0_1();

  //////////////////////////////////////////////////////////////////////////
  // Compute the derivative between samples
  tiempoentremuestras = TIMEINTERVALMS / MILLISINSEC;
  derivadaSimple = abs(ADCValue - previoADCValue)/tiempoentremuestras;
  derivadaMedia = abs(media - previamedia)/tiempoentremuestras;
  //////////////////////////////////////////////////////////////////////////

  Rebalancing();

}
