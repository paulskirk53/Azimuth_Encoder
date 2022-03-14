// code formatter id shift alt f - don't use it unless it can be modded to meet the standards used in my code
//  note the todo items in this file and make the changes indicated
/*

 Note Note Note Note Note Note Note
*/

// This code has been modified to remove the LCD
//  and incorporates the Monitor program.


//  Name:       Merged-Box-Azimuth_Encoder
//   Created:  5-10-21
//   Author:    Paul Kirk

// if it's required to set a home point or park point marker A_Counter is the variable which must be set.
// see below for north, east, south, west values for A_Counter
// for any position X degrees the tick position to set for A_counter is ticksperDomeRev / (360/ x )

// there are 26.25 rotations of the encoder wheel for one complete rotation of the pulsar dome.
// in a previous version without the toothed wheel around the perimeter, 26 encoder wheel revolutions  equalled 10413 encoder ticks
// from which it can bee calculated that 1 encoder wheel rev equates to 400.5 ticks.
// so in the pulsar system with the toothed wheel around the perimeter, it takes 26.25 revs of the encoder wheel for 1 dome rotation
// In terms of ticks therefore, the total number of ticks for a Pulsar dome revolution is 26.25 * 400.5 = 10513

// North = 0
// East = ticksperDomeRev/4
// South = ticksperDomeRev/2
// West = ticksperDomeRev*3/4
#include <avr/cpufunc.h> /* Required header file */
#include <Arduino.h>
#include <SPI.h> // SET UP AS SPI SLAVE
// from microchip example below
#include <avr/io.h>
#include <avr/interrupt.h>

// function declarations

void encoder();

void interrupt();
void NorthSync();
void EastSync();
void SouthSync();
void WestSync();
bool PowerForCamera(bool State);
void resetViaSWR();
void lightup();
static void SPI0_init(void);

// end function declarations

// encoder:
#define A_PHASE 2 // USES PINS 2 AND 3 for encoder interrupt
#define B_PHASE 3
#define NorthPin 18
#define EastPin 28 // changed for SPI
#define SouthPin 20
#define WestPin 29 // changed for spi
#define CameraPower 10
#define off false
#define on true
#define ledPin 12 // change for spi

//
#define ASCOM Serial
#define Stepper Serial1
#define Monitor Serial2
// encoder:
// should this be set for 261 degrees? Otherwise the driver will request a move to 261 which will move the aperture out of alignment with the parked dome (and scope)
//  the position for 261 is set in Setup() below to reflect this.

volatile long A_Counter; // volatile because it's used in the interrupt routine
volatile bool homeFlag;
// General
String pkversion = "4.0";
float Azimuth;           // The data type is important to avoid integer arithmetic in the encoder() routine
uint16_t integerAzimuth; // this is what is returned to the stepper routine by SPI - just to avoid the complication of sending float over SPI
                         // and also because we really don't need fractional degrees for dome movement.
float SyncAz;
volatile int azcount;
long Sendcount = 0;
long pkstart = 0;
float ticksperDomeRev = 10513; // this was worked out empirically by counting the number of encoder wheel rotations for one dome rev. 11-9-21
long calltime = 0;
bool cameraPowerState = off;

// to be returned via SPI to the Stepper routine - the bytes represent the Azimuth
volatile byte highByteReturn;
volatile byte lowByteReturn;

// to hold the incoming request from master
volatile char SPIReceipt;

void setup()
{

  pinMode(NorthPin, INPUT_PULLUP); // these are 4 microswitches for syncing the encoder
  pinMode(EastPin, INPUT_PULLUP);
  pinMode(SouthPin, INPUT_PULLUP);
  pinMode(WestPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  // todo - the line below will need uncommenting and change to ensure it acts on the Rx lne for the seril line between Stepper and encoder
  pinMode(9, INPUT_PULLUP); // SEE THE github comments for this code - it pulls up the Rx line to 5v and transforms the hardware serial2 link's efficiency

  //  notes for serial comms -
  ASCOM.begin(19200);   // with ASCOM driver refer to DIP 40 pinout to get correct pin numbers for all the serial ports - see the google doc - 'Pin config for Radio Encoder MCU'
  Stepper.begin(19200); // with stepper MCU - change this to Serial1 when coding for 4809, see google doc - Pin config for Radio Encoder MCU
  Monitor.begin(19200); // with monitor program Change to Serial2 when coding for 4809,    see google doc - Pin config for Radio Encoder MCU
 // delay(10000);
// ASCOM.print("here");
  // set up the LCD's number of columns and rows:
  // lcd.begin(16, 2);

  // encoder:
  pinMode(A_PHASE, INPUT);
  pinMode(B_PHASE, INPUT);

  // pins 2,3,18,19,20,21 are the only pins available to use with interrupts on the mega2560 (no pin limit)restrictions on 4809)
  attachInterrupt(digitalPinToInterrupt(A_PHASE), interrupt, RISING); // interrupt for the encoder device

  // interupts for the azimuth syncs below
  // attachInterrupt(digitalPinToInterrupt(NorthPin), NorthSync, RISING);
  attachInterrupt(digitalPinToInterrupt(EastPin), EastSync, RISING);
  // attachInterrupt(digitalPinToInterrupt(SouthPin), SouthSync, RISING);
  attachInterrupt(digitalPinToInterrupt(WestPin), WestSync, FALLING);       // changed to falling as the pin is active low todo if need to change back 14-3-22

  // lcd.setCursor(0, 0);
  // lcd.print("Az MCU Ver " + pkversion);                 //16 char display
  // delay(1000);                                   //so the message above can be seen before it is overwritten

  azcount = 0;

  A_Counter = ticksperDomeRev / (360.0 / 261.0); //  the position of due west - 261 for the dome when the scope is at 270.

  PowerForCamera(off); // camera power is off by default
                       // delay(15000);            //THIS DELAY is set to give the operator time to open com12 (ASCOM port) to check if the message below arrives tested ok 22/11/21
                       // ASCOM.print("MCU RESET");

  // SPI stuff - NOTE that the encoder is the SPI slave

  SPI0_init();

  // interrupt for SPI servicing
  // SPI.attachInterrupt();
  // send data on MISO
  // pinMode (MISO, OUTPUT);

  sei(); /* Enable Global Interrupts */

  // the code below flashes LED five times causing 5 second delay TODO - REMOVE COMMENTS
   lightup();

} // end setup

void loop()
{
  // Serial.println("HERE");
  encoder();

  // Serial.println(String(Azimuth) + "#  ");
  // Serial.println(String(remA_counter));
  // delay(1000);
  if (ASCOM.available() > 0) // request from ASCOM Driver
  {
    // Serial.print("A_Counter ");
    //  Serial.println(A_Counter);
    encoder();

    String ReceivedData = "";
    if (digitalRead(ledPin) == LOW)
    {
      digitalWrite(ledPin, HIGH);
      // ASCOM.println("Setting HIGH");
    }
    else
    {
      digitalWrite(ledPin, LOW);
      // ASCOM.println("Setting LOW");
    }
    ReceivedData = ASCOM.readStringUntil('#');
    // Serial.print("received ");
    // Serial.println(ReceivedData );

    if (ReceivedData.indexOf("AZ", 0) > -1) //
    {

      ASCOM.print(String(Azimuth) + "#");
    }

    // Sync to Azimuth code below:
    if (ReceivedData.indexOf("STA", 0) > -1) //
    {

      ReceivedData.remove(0, 3); // strip 1st three chars
      SyncAz = ReceivedData.toFloat();
      if ((SyncAz > 0.0) && (SyncAz <= 360.0)) // check for a valid azimuth value
      {
        // now work out the tick position for this azimuth

        A_Counter = ticksperDomeRev / (360 / SyncAz);
      } // endif
    }   // endif
  }

  if (Stepper.available() > 0) // ser1 is encoder with stepper MCU
  {
    encoder();
    String ReceivedData = "";

    ReceivedData = Stepper.readStringUntil('#');
    // Serial1.print("received ");
    // Serial1.println(ReceivedData );
    if (ReceivedData.indexOf("AZ", 0) > -1)
    {
      azcount++;
      Stepper.print(String(Azimuth) + "#");
    } // endif

    if (azcount > 250)
    {
      azcount = 0;
    } // endif

  } // endif

  if (Monitor.available() > 0) // Monitor is comms with the windows forms arduino monitoring app
  {
    String MonitorData = Monitor.readStringUntil('#');
    //todo remove line below
    // ASCOM.print(MonitorData);

        if (MonitorData.indexOf("encoder", 0) > -1)
    {
      Monitor.print("encoder#");
    }

    if (MonitorData.indexOf("reset", 0) > -1) //
    {
      resetViaSWR();
    }

    if (MonitorData.indexOf("CAMON", 0) > -1) //
    {
      PowerForCamera(on);
    }

    if (MonitorData.indexOf("CAMOFF", 0) > -1) //
    {
      PowerForCamera(off);
    }

    if (MonitorData.indexOf("EncoderRequest", 0) > -1)
    {

      if (azcount > 240)
      {
        azcount = 0;
      } // endif
      
      Monitor.print(String(Azimuth) + "#" + String(azcount) + "#"); // write the two monitoring values to the windows forms Arduino Monitor program

      // Monitor.print(String(azcount) + "#");
      // check status of the power to the camera and print to monitor program
      if (cameraPowerState)
      {
        Monitor.print("ON#");
      }
      else
      {
        Monitor.print("OFF#");
      }

    } // Endif

  } // endif

/*
if (homeFlag==true)
{
  ASCOM.println("homeflag is LOW");    // this worked properly when the westpin was grounded temporarily
}

*/

  homeFlag = false;             // this is set true by the westsync interrupt, so at the end of each loop, set it false


} // end void loop

void encoder()
{
  // Encoder:

  if (A_Counter < 0)
  {
    A_Counter = A_Counter + ticksperDomeRev; // set the counter floor value
  }

  if (A_Counter > ticksperDomeRev) // set the counter ceiling value
  {
    A_Counter = A_Counter - ticksperDomeRev;
  }

  Azimuth = float(A_Counter) / (ticksperDomeRev / 360.0); // (ticks for one dome rev) / 360 (degrees) - about 29
  // i.e number of ticks per degree

  // some error checking
  if (Azimuth < 1)
  {
    Azimuth = 1.0;
  }

  if (Azimuth > 360.0)
  {
    Azimuth = 360.0;
  }
  // now store the float azimuth into an uint16_t and for the highbyte and lowbyte for spi transmission to the Stepper routine
  integerAzimuth = Azimuth; // REMEMBER Azimuth needs to be float due to effects of integer arithmetic.
  lowByteReturn = lowByte(integerAzimuth);
  highByteReturn = highByte(integerAzimuth);

} // end void encoder

// interrupt used by the encoder:
void interrupt() // Interrupt function
{

  char i, j;
  i = digitalRead(B_PHASE);
  j = digitalRead(A_PHASE);
  if (i == j)
  {
    A_Counter -= 1;
  }

  else
  {

    A_Counter += 1; // increment the counter which represents increasing dome angle

  } // end else clause
} // end void interrupt

void NorthSync()
{
  A_Counter = 0;
}
void EastSync()
{
  A_Counter = ticksperDomeRev / (360.0 / 90.0);     // on reflection these positions are not true i.e. 90.0 is not the right value to use for dome east
}
void SouthSync()
{
  A_Counter = ticksperDomeRev / 2.0;                // same comment as above
}
void WestSync()                              //this acts as the home position
{
  A_Counter = ticksperDomeRev / (360.0 / 261.0);     // set the azimuth to 261 degrees (west)
  homeFlag= true;                                    // set this for transmission to the stepper when home is found
}

bool PowerForCamera(bool State)
{
  if (State)
  {
    digitalWrite(CameraPower, HIGH);
    cameraPowerState = on;
  }
  else
  {
    digitalWrite(CameraPower, LOW);
    cameraPowerState = off;
  }
}
void resetViaSWR()
{
  _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
}
void lightup()
{
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
  }
}

// SPI interrupt routine
ISR(SPI0_INT_vect) // was this in arduino -> (SPI_STC_vect)
{
  SPIReceipt = SPI0.DATA;

  if (SPIReceipt == 'A') // a dummy transaction which loads the SPDR with the low byte
  {                      // in readiness for the 'L' transaction
    SPI0.DATA = lowByteReturn;
  }
  if (SPIReceipt == 'L') // low byte is returned and SPDR is loaded with the high byte
  {                      // in readiness for the 'H' transaction
    SPI0.DATA = highByteReturn;
  }

  if (SPIReceipt == 'H')    // High byte is returned and SPDR is loaded with homeflag
  {                         // in readiness for the 'S' transaction
    SPI0.DATA = homeFlag;   // sends status of the home position true if at the home position i.e. the sensor has been activated
    
  }
  
    if (SPIReceipt == 'S')  // Homeflag is returned and SPDR is loaded with zero
  {                         // in readiness for the next 'A' transaction
    SPI0.DATA = 0x00;       // fill spdr with 0
    azcount++;              // counter is sent to the monitor program as an indication that SPI comms between stepper and encoder are live
  }

  SPI0.INTFLAGS = SPI_IF_bm; /* Clear the Interrupt flag by writing 1 */

} // end of interrupt routine SPI_STC_vect

static void SPI0_init(void)
{
  PORTA.DIR &= ~PIN4_bm; /* Set MOSI pin direction to input */
  PORTA.DIR |= PIN5_bm;  /* Set MISO pin direction to output */
  PORTA.DIR &= ~PIN6_bm; /* Set SCK pin direction to input */
  PORTA.DIR &= ~PIN7_bm; /* Set SS pin direction to input */

  SPI0.CTRLA = SPI_DORD_bm               /* LSB is transmitted first */
               | SPI_ENABLE_bm           /* Enable module */
                     & (~SPI_MASTER_bm); /* SPI module in Slave mode */

  SPI0.INTCTRL = SPI_IE_bm; /* SPI Interrupt enable */
}