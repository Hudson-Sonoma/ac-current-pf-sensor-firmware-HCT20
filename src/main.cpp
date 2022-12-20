#include <Arduino.h>
#include <Wire.h>
#include <Logic.h>
#include <Event.h>
#include <twi_log.h>
#include <checksum.hpp>
#include <sampling.hpp>
#include <hardware.hpp>

// Calibrate using a know resistive load, this value could be 0 to SAMPLES_PER_60HZ_CYCLE/2
#define PF_VOLTAGE_PHASE_CAL 60

//  SUPPORTED COMMANDS from SHT20
#define SHT2x_GET_TEMPERATURE_NO_HOLD   0xF3
#define SHT2x_GET_HUMIDITY_NO_HOLD      0xF5
// #define SHT2x_SOFT_RESET                0xFE
// #define SHT2x_WRITE_USER_REGISTER       0xE6
// #define SHT2x_READ_USER_REGISTER        0xE7
#define SHT2x_ADDRESS                   0x40

void receiveHandler(int numbytes);
void requestHandler();
//void twi_printlog( twi_trace_t * ttrace);
void voltageSenseISR();
void setupLogic();
//uint8_t sht20_crc8(const uint8_t *data, uint8_t len);
volatile uint8_t g_i2c_command;
volatile uint16_t g_temperature;
volatile uint16_t g_humidity;
volatile bool VoltageIsHigh = false;


ADC_Channel chA(PIN_PA4,PIN_PA7); 
ADC_Channel chB(PIN_PA5,PIN_PA7); 
ADC_Channel chC(PIN_PA6,PIN_PA7);  

void setup() {
  Serial.begin(115200, SERIAL_8N1 | SERIAL_TX_ONLY);

  /* Bus Error Detection circuitry needs Master enabled to work */
  //TWI0.CTRLA = TWI_FMPEN_bm
  TWI0.MCTRLA = TWI_ENABLE_bm; // | 0b1100 ; // timeout 200us enabled.
  TWI0.MSTATUS = 0x01; // IDLE bus state

  setupLogic();

  ADC_init();

  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
  Wire.begin(SHT2x_ADDRESS);

}

void loop() {
  // put your main code here, to run repeatedly:

  // Syncronise with mains voltage
  // wait until global variable VoltageIsHigh is set to false by ISR, or break out of loop if it takes longer than 25ms
  uint32_t timeout = millis() + 25;
  while (!VoltageIsHigh && millis() < timeout) {
    // do nothing
  }
  // wait until global variable VoltageIsHigh is set to true by ISR, or break out of loop if it takes longer than 25ms
  timeout = millis() + 25;
  while (VoltageIsHigh && millis() < timeout) {
    // do nothing
  }
  delay(7);

  chA.reset();
  chB.reset();
  chC.reset();
  PORTB.OUTSET = PIN3_bm; delayMicroseconds(5); PORTB.OUTCLR = PIN3_bm;
  hardware_fast_sample_chABC(MAX_SAMPLES,chA,chB,chC);
  //sample_slow_chABC(SAMPLES_PER_60HZ_CYCLE*3/2,chA,chB,chC);

  uint16_t Araw = chA.getRMS_RAW();
  uint16_t Braw = chB.getRMS_RAW();
  uint16_t Craw = chC.getRMS_RAW();
  uint16_t AMPSraw = Araw + Braw + Craw;

  float temp = CALCS_FACTOR * Braw;//((float)AMPSraw)*CALCS_FACTOR;
  uint16_t phase_count = chB.get_phase_count();
  float degrees = ((float)phase_count - PF_VOLTAGE_PHASE_CAL)*(360.0/SAMPLES_PER_60HZ_CYCLE);
  float pf = cos(degrees*(3.14159/180.0));     // phase_count - 80 for swapped LUT input
  uint16_t temperatureRaw=(uint16_t)(((temp) + 46.85)*65536.0 / 175.72);
  uint16_t humidityRaw=(uint16_t)((fabs(pf)*100.0 + 6.0) * 65536.0 / 125.0 );

  noInterrupts();
  g_temperature = temperatureRaw;
  g_humidity = humidityRaw;
  interrupts();

  delay(1000);

  Serial.print("debug current:");Serial.print(temp);
  Serial.print(" phase count:");Serial.print(phase_count);
  Serial.print(" degrees:");Serial.print(degrees);
  Serial.print(" pf ");Serial.println(pf);

  //twi_printlog(&_ttrace); // to Serial.

}


void receiveHandler(int numbytes) 
{
  Wire.getBytesRead(); // reset count of bytes read. We don't do anything with it here, but a write is going to reset it to a new value.

  LOG_TWI(_ttrace,TWI0.MSTATUS,TWI0.SSTATUS,3,"  RC",numbytes);
  if (numbytes > 0) {
    // Called on a Write address, data
    g_i2c_command = Wire.read();
  }
}
/* This one is weirder. The way the onRequest behaves and what it needs to do is very counter-intuitive (and this model is probably part
 * of WHY all Arduino I2C slave devices use wire like "serial with a clock" instead of like a civilised device....)
 *
 * the handler registered with onRequest is called when the slave gets a packet that matches it's address, set to read; This is called
 * once, and prints out all of the data that the master *might* request. It then does not fire again until the next start condition
 * followed by a matching address. There is a chance that the master might NACK a transmission before it was completed. The Arduino API does
 * not allow to track how many bytes were actually written by the Slave. This is fixed with getBytesRead().
 *
 * Without the getBytesRead() extension, there is no way for a slave written through the Arduino API to react to whether the master has
 * read something something - and that's a very common behaviour in commercial I2C devices.
 *
 * Another thing to consider is that, if the there is a buffer underflow, the TWI will keep the SDA lines released, making the master think
 * a 0xFF was transmitted.
 */
void requestHandler() 
{
  // We will start reading from the pointer.
  // But if there was a previous read, and the master then started a
  // second read, we want the pointer to pick up where they left off.
  uint8_t bytes_read = Wire.getBytesRead();

  LOG_TWI(_ttrace,TWI0.MSTATUS,TWI0.SSTATUS,3,"  RQ",g_i2c_command);
 if ((g_i2c_command == SHT2x_GET_TEMPERATURE_NO_HOLD) ) {
      Wire.write((uint8_t)(g_temperature >> 8)); 
      Wire.write((uint8_t) g_temperature);
      uint8_t buf[] = { (uint8_t)(g_temperature >> 8), (uint8_t) g_temperature };
      Wire.write(sht20_crc8(buf, 2));
  } else if ((g_i2c_command == SHT2x_GET_HUMIDITY_NO_HOLD) ) {
      Wire.write((uint8_t)(g_humidity >> 8)); 
      Wire.write((uint8_t) g_humidity);
      uint8_t buf[] = { (uint8_t)(g_humidity >> 8), (uint8_t) g_humidity };
      Wire.write(sht20_crc8(buf, 2));
  }

}



void twi_printlog( twi_trace_t * ttrace) 
{
    if (ttrace->count > 0) {
        char mstatus_msg[80];
        char sstatus_msg[80];

        for (uint8_t i = 0; i < ttrace->count; i++) {
          uint32_t t1 = ttrace->entry[i].t1;
          uint8_t mstatus = ttrace->entry[i].mstatus;
          uint8_t sstatus = ttrace->entry[i].sstatus;
          uint8_t step=ttrace->entry[i].step;
          char    char1=ttrace->entry[i].char1;
          char    char2=ttrace->entry[i].char2;
          char    char3=ttrace->entry[i].char3;
          char    char4=ttrace->entry[i].char4;

          uint8_t val = ttrace->entry[i].val;

          sprintf(mstatus_msg,BYTE_TO_TWI_MSTATUS_FMT,BYTE_TO_TWI_MSTATUS(mstatus));
          sprintf(sstatus_msg,BYTE_TO_TWI_SSTATUS_FMT,BYTE_TO_TWI_SSTATUS(sstatus));
        //   if (step == 100) {  // sleep
        //         // normal, 
        //     Serial.print(i); Serial.print("\t"); 
        //     Serial.print(t1); Serial.print("\t");
        //     Serial.print(mstatus_msg); Serial.print(" "); Serial.print(sstatus_msg); 
        //     Serial.println();
        //   } else {

            Serial.print(i); Serial.print("\t"); 
            Serial.print(t1); Serial.print("\t");
            Serial.print(mstatus_msg); Serial.print(" "); Serial.print(sstatus_msg); 
            Serial.print(" STEP: "); Serial.print(step);
            Serial.print(" ACTION: "); Serial.print(char1);Serial.print(char2);Serial.print(char3);Serial.print(char4);
            Serial.print(" "); Serial.printHexln(val);
         // }
        }
        ttrace->count = 0;

    }

}


void setupLogic()
{

  pinMode(PIN_PA0,INPUT);
  pinMode(PIN_PA1,INPUT);
  pinMode(PIN_PA2,INPUT);
  pinMode(PIN_PA3,OUTPUT);  // LED
  digitalWrite(PIN_PA3,LOW);
  pinMode(PIN_PB3,OUTPUT);  // LED
  digitalWrite(PIN_PB3,LOW);

  // put your setup code here, to run once:
  Logic0.enable = true;               // Enable logic block 0
  Logic0.input0 = in::masked;         // PA0 masked
  Logic0.input1 = in::masked;            // PA1 TX1.  pin PA1 cannot be grounded or connected to a cap.  Make a loop of wire, PA2 - PA1 with 1M resistors.
  Logic0.input2 = in::pin;         // PA2 RX1 masked
  Logic0.output = out::disable;        // Enable logic block 0 output pin PA4 (ATtiny3224))
  Logic0.filter = filter::disable;    // No output filter enabled
  Logic0.truth = 0x01;                // Set truth table - HIGH only if input low
  //Logic0.truth = 0x02;                // Set truth table - HIGH only if input HIGH
  Logic0.edgedetect = edgedetect::enable;       // Enable edge detection
  Logic0.attachInterrupt(&voltageSenseISR,CHANGE);
  Logic0.init();
  //pinModeFast(PIN_PA4,OUTPUT);

  // Start the AVR logic hardware
  Logic::start();

}


void voltageSenseISR() 
{
  if (VoltageIsHigh) {
    /* transition HIGH to LOW */
    PORTA.OUTCLR = PIN3_bm;
    VoltageIsHigh = false;

  } else {
    /* transition LOW to HIGH */
    PORTA.OUTSET = PIN3_bm;
    VoltageIsHigh = true;

    chA.phase_mark();
    chB.phase_mark();
    chC.phase_mark();
  }

}


/*
Power factor can be numerically calculated from time-series of voltage and current measurements.  Assume an array of 256 voltage measurements named Varr, and an array of 256 current measurements named Iarr.
*/

// void calc_power_factor()
// {

// // Calculate the power factor
// float pf = 0;
// float sumP = 0;
// float sumS = 0;
// for (int i = 0; i < 256; i++) {
//   sumP += Varr[i] * Iarr[i];
//   sumS += Varr[i] * Varr[i] + Iarr[i] * Iarr[i];
// }
// pf = sumP / sqrt(sumS);

// }


// float calcPowerFactor(float Varr[], float Aarr[], int len)
// {
//     // Initialize variables
//     float apparent_power = 0;
//     float reactive_power = 0;

//     // Loop through the arrays and calculate apparent and reactive power
//     for (int i = 0; i < len; i++)
//     {
//         apparent_power += Varr[i] * Aarr[i];
//         reactive_power += Varr[i] * Aarr[i] * sin(atan2(Aarr[i], Varr[i]));
//     }

//     // Calculate and return power factor
//     return apparent_power / reactive_power;
// }



