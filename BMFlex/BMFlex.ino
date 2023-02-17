/*******************************************************
    ________    _______  __    ________  __________ 
   / ____/ /   / ____/ |/ /   / ____/ / / / ____/ / 
  / /_  / /   / __/  |   /   / /_  / / / / __/ / /  
 / __/ / /___/ /___ /   |   / __/ / /_/ / /___/ /___
/_/   /_____/_____//_/|_|  /_/    \____/_____/_____/
v1.0 - Bouletmarc
This program will sample a 50-150hz signal depending on ethanol 
content, and output a 0-5V signal via DAC Converter.

This program will also sample a 0-100Duty signal depending on ethanol 
temperature, and output a 0-5V signal via DAC Converter. (-40 to 80 celcius)

Connect 0-5V output to the desired OBD1 ECU Input (Honda), and tune
the "FLEX FUEL" parameter tab accordingly.

NOTE:
DOES NOT REQUIRE Lowpass filter to be used on output!
It use instead DAC Converter's (MCP4725)

****************************
**LINEAR CONVERTION TABLE**
0.5V = 0%   Ethanol - 50Hz
0.9V = 10%  Ethanol - 60Hz
1.7V = 30%  Ethanol - 80Hz
3.9V = 85%  Ethanol - 135Hz
4.5V = 100% Ethanol - 150Hz
********************************************************/

#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 MCP4725;
Adafruit_MCP4725 MCP4725_Temp;
byte I2CByte = 0x60;      //Detected Automatically (Decimal 96 (Byte 0x60) are the default for A0 Grounded)
byte I2CByteTemp = 0x61;  //Detected Automatically

int inpPin = 2; 
int SignalPin = A0;     //not used
int SignalTempPin = A1; //not used
int StatusPin = A3;

//Define global variables
uint16_t pwm_output  = 0;     //integer for storing PWM value (0-255 value)
uint16_t pwm_outputTemp  = 0; //integer for storing PWM value (0-255 value)
int HZ = 0;                   //unsigned 16bit integer for storing HZ input
int ethanol = 0;              //Store ethanol percentage here

float freq;
int duty;           //Duty cycle (0.0-100.0)
float period;       //Store period time here (eg.0.0025 s)
float temperature = 0;  //Store fuel temperature here
int cels = 0;
int fahr = 0;

static long highTime = 0;
static long lowTime = 0;

int Correction = 0;
int CorrectionTemp = 0;
int input_voltage = 0.0;
int input_voltageTemp = 0.0;

//#####################################################################
//long previousMillis = 0;
//long interval = 1000;
//volatile unsigned long _freq;
//volatile unsigned long _pulseCount;
//volatile unsigned int _tickCount;
//volatile uint8_t _ready; // becomes 1 when gate time is elapsed

/*#if F_CPU == 20000000
#define GATETIME_CLICKS 1000
#elif F_CPU == 16000000
#define GATETIME_CLICKS 800
#else
#define GATETIME_CLICKS 400
#endif*/

/*ISR(PCINT0_vect)
{
  _pulseCount++;
}*/

/*unsigned long count_frequency(uint8_t pin)
{
  noInterrupts();

  _tickCount = 0;
  _pulseCount = 0;
  _ready = 0;

  GIMSK = 0b00100000;
  //PCMSK |= _BV(pin);
  PCMSK1 |= _BV(pin);

  // Set up Timer1 for gate time measurement.
  //TCCR1 = _BV(CTC1);            // CTC mode
  TCCR1A = _BV(CTC1);            // CTC mode

#if F_CPU == 20000000 || F_CPU == 16000000 || F_CPU == 8000000
  //TCCR1 |= _BV(CS12);           // prescale: 8
  TCCR1A |= _BV(CS12);           // prescale: 8
#elif F_CPU == 1000000
  //TCCR1 |= _BV(CS10);           // no prescale
  TCCR1A |= _BV(CS10);           // no prescale
#endif

  OCR1C = 250;
  TIMSK |= _BV(OCIE1A);         // enable Timer1 interrupt
  interrupts();

  while (_ready == 0) {}        // Wait for gate time

  PCMSK = 0;
  GIMSK = 0;

  return _freq; 
}

ISR(TIM1_COMPA_vect)
{
  if (_tickCount == GATETIME_CLICKS)
  {
    // On ATTtiny, PCI is triggered on both rising and falling edge,
    // use only half of the pulseCount      
      
    _freq = _pulseCount * 5;    // based on 100ms
    _ready = 1;

    TIMSK &= ~_BV(OCIE1A);      // disable Timer1 Interrupt
  }

  _tickCount++;
}*/
//#####################################################################

/*void setupTimer()   // setup timer1
{           
  TCCR1A = 0;      // normal mode
  TCCR1B = 132;    // (10000100) Falling edge trigger, Timer = CPU Clock/256, noise cancellation on
  TCCR1C = 0;      // normal mode
  TIMSK1 = 33;     // (00100001) Input capture and overflow interupts enabled
  TCNT1 = 0;       // start from 0
}

ISR(TIMER1_CAPT_vect)    // PULSE DETECTED!  (interrupt automatically triggered, not called by main program)
{
  revTick = ICR1;      // save duration of last revolution
  TCNT1 = 0;       // restart timer for next revolution
}

ISR(TIMER1_OVF_vect)    // counter overflow/timeout
{ revTick = 0; }        // Ticks per second = 0
*/

void setup()
{
  pinMode(inpPin,INPUT);
  pinMode(StatusPin,OUTPUT);
  
  pinMode(SignalPin,INPUT);
  pinMode(SignalTempPin,INPUT);
  //pinMode(SignalPin,OUTPUT);
  //pinMode(SignalTempPin,OUTPUT);
  
  //setPwmFrequency(outPin,1); //Modify frequency on PWM output
  //setupTimer();

  ScanForI2C();
  
  Serial.begin(38400);
  FlashI2CCode();
  
  MCP4725.begin(I2CByte);
  MCP4725_Temp.begin(I2CByteTemp);
  
  MCP4725.setVoltage(405, false);
  MCP4725_Temp.setVoltage(405, false);
}

void FlashI2CCode() {
  if (I2CByte == 0x60 && I2CByteTemp == 0x61) {
    Serial.println("MCP4725 Addresse's set correctly!");
  }
  
  if (I2CByte != 0x60) {
    Serial.println(String(I2CByte));
    
    //Quick Flash
    digitalWrite(StatusPin, HIGH);
    delay(250);
    digitalWrite(StatusPin, LOW);
    delay(1000);
    //###########
      
    int Code10 = I2CByte / 10;
    int Code1 = I2CByte - (Code10 * 10);
    for (byte i = 0; i < Code10; i++) {
      digitalWrite(StatusPin, HIGH);
      delay(500);
      digitalWrite(StatusPin, LOW);
      delay(500);
    }
    
    delay(500);
    for (byte i = 0; i < Code1; i++) {
      digitalWrite(StatusPin, HIGH);
      delay(250);
      digitalWrite(StatusPin, LOW);
      delay(250);
    }
  }

  //########################################
  if (I2CByteTemp != 0x61) {
    Serial.println(String(I2CByteTemp));
    
    //Long Flash
    digitalWrite(StatusPin, HIGH);
    delay(500);
    digitalWrite(StatusPin, LOW);
    delay(1000);
    //###########
      
    int Code10 = I2CByteTemp / 10;
    int Code1 = I2CByteTemp - (Code10 * 10);
    for (byte i = 0; i < Code10; i++) {
      digitalWrite(StatusPin, HIGH);
      delay(500);
      digitalWrite(StatusPin, LOW);
      delay(500);
    }
    
    delay(500);
    for (byte i = 0; i < Code1; i++) {
      digitalWrite(StatusPin, HIGH);
      delay(250);
      digitalWrite(StatusPin, LOW);
      delay(250);
    }
  }
}

void ScanForI2C() {
  // Leonardo: wait for serial port to connect
  while (!Serial) { }

 //scan
  byte count = 0;
  bool DoneFirst = false;
  Wire.begin();
  
  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      if (!DoneFirst) 
      {
        if (i != I2CByte) 
        {
          I2CByte = i;
          I2CByteTemp = I2CByte + 1;
          DoneFirst = true;
        }
        else {
          DoneFirst = true;
        }
      }
      else {
        if (i != I2CByteTemp) 
        {
          I2CByteTemp = i;
        }
      }
      count++;
    }
  }
}
 
void loop()
{
  highTime = pulseIn(inpPin,HIGH);
  lowTime = pulseIn(inpPin,LOW);
  period = highTime+lowTime;
  freq = 1000000.0/period;

  //duty = (highTime/period)*100;
  duty = ((100*(highTime/(double (period))))); //Calculate duty cycle (integer extra decimal)

  int CorrectionForFrequency = map(freq, 50, 300, 0, 13);
  freq -= CorrectionForFrequency;
  
  HZ = (int) freq;
  getfueltemp();

  //calculate ethanol percentage
  if (HZ > 50) {
    ethanol = HZ-50;
  }
  else {
    ethanol = 0;
  }
  if (ethanol > 100) {
    ethanol = 100;
  }

  //Get 0-4096 signals
  uint32_t MCP4725_value = map(ethanol, 0, 100, 405, 3720);     //Voltage range from 0-4096(0V-5V), that mean 3720 should equal 4.5V and 405 equal 0.5V
  uint32_t MCP4725_Temp_value = map(cels, -40, 125, 405, 3720);
  MCP4725.setVoltage(MCP4725_value + Correction, false);
  MCP4725_Temp.setVoltage(MCP4725_Temp_value + CorrectionTemp, false);

  //###########################################
  delay(100);
  uint32_t CurrentVoltage = analogRead(SignalPin);
  uint32_t CurrentVoltageTemp = analogRead(SignalTempPin);
  uint32_t DesiredVoltage = map(ethanol, 0, 100, 500, 4500); //Convert 0.5V-4.5V range to 0-1024 range
  uint32_t DesiredVoltageTemp = map(cels, -40, 125, 500, 4500); //Convert 0.5V-4.5V range to 0-1024 range
  
  input_voltage = (int) (((CurrentVoltage * 5.0) / 1024.0) * 1000.0); 
  input_voltageTemp = (int) (((CurrentVoltageTemp * 5.0) / 1024.0) * 1000.0);

  int CorrectionForVoltageInput = map(input_voltage, 500, 4500, 0, 3) * 10;
  int CorrectionForVoltageInputTemp = map(input_voltageTemp, 500, 4500, 0, 3) * 10;
  input_voltage -= CorrectionForVoltageInput;
  input_voltageTemp -= CorrectionForVoltageInputTemp;

  float PercentOff = 0;
  float PercentOffTemp = 0;
  if (DesiredVoltage > input_voltage) {
    PercentOff = (DesiredVoltage - input_voltage);
    PercentOff = (PercentOff * 100.0) / 4500.0;
    PercentOff = PercentOff /2;
    Correction += ((PercentOff * 3720) / 100);
  }
  else {
    PercentOff = (input_voltage - DesiredVoltage);
    PercentOff = (PercentOff * 100.0) / 4500.0;
    PercentOff = -PercentOff;
    PercentOff = PercentOff /2;
    Correction += ((PercentOff * 3720) / 100);
  }

  if (DesiredVoltageTemp > input_voltageTemp) {
    PercentOffTemp = (DesiredVoltageTemp - input_voltageTemp);
    PercentOffTemp = (PercentOffTemp * 100.0) / 4500.0;
    PercentOffTemp = PercentOffTemp /2;
    CorrectionTemp +=  ((PercentOffTemp * 3720) / 100);
  }
  else {
    PercentOffTemp = (input_voltageTemp - DesiredVoltageTemp);
    PercentOffTemp = (PercentOffTemp * 100.0) / 4500.0;
    PercentOffTemp = -PercentOffTemp;
    PercentOffTemp = PercentOffTemp /2;
    CorrectionTemp +=  ((PercentOffTemp * 3720) / 100);
  }
  
  //##############
  if (Correction > 100) {
    Correction = 100;
  }
  if (Correction < -100) {
    Correction = -100;
  }
  if (CorrectionTemp > 100) {
    CorrectionTemp = 100;
  }
  if (CorrectionTemp < -100) {
    CorrectionTemp = -100;
  }
  //###########################################

  Serial.println("freq:" + String(HZ) + ", ethanol%:" + String(ethanol) + "(" + String(MCP4725_value) + "), temp:" + String(cels) + "(" + String(MCP4725_Temp_value) + "), Correction:" + String(PercentOff) + "%, CurrentV:" + String((input_voltage / 1000.0)) + ", DesiredV:" + String((DesiredVoltage / 1000.0)));
  //delay(100);
}

void getfueltemp()
{
  //read fuel temp from input duty cycle
  float T = (float(1.0/float(HZ)));             //Calculate total period time
  float period2 = float(100-duty)*T;            //Calculate the active period time (100-duty)*T
  float temp2 = float(10) * float(period2);     //Convert ms to whole number
  temperature = ((40.25 * temp2)-81.25);        // Calculate temperature for display (1ms = -40, 5ms = 80)
  cels = int(temperature);
  //cels = cels*0.1;
  
  //float fahrtemp = ((temperature*1.8)+32);
  //fahr = fahrtemp*0.1;
  
  if (cels < -40) {
    cels = -40;
  }
  if (cels > 125) {
    cels = 125;
  }
}

/*void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}*/
