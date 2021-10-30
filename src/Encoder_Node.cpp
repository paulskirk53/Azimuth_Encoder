// code formatter id shift alt f
//  note the todo items in this file and make the changes indicated
/*

Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note

This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch argh, argh, argh
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch
This is the Merged-Box-Azimuth_Encoder branch

Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note
*/

// This code has been modified to remove the LCD
//  and incorporates the Monitor program.

// NB the changes need testing as of this note 4-10-21 - remove this once tested
// Oct 20th 21, following from the above line, the encoder has been tested with the code and AZ# and STA999# work reliably.

//  Name:       Merged-Box-Azimuth_Encoder
//   Created:  5-10-21
//   Author:     DESKTOP-OCFJAV9\Paul

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

#include <Arduino.h>

// function declarations

void encoder();
void interrupt();
void NorthSync();
void EastSync();
void SouthSync();
void WestSync();
bool PowerForCamera(bool State);

// end function declarations

// encoder:
#define A_PHASE      2 // USES PINS 2 AND 3 for encoder interrupt
#define B_PHASE      3
#define NorthPin    18
#define EastPin      6
#define SouthPin    20
#define WestPin      4
#define CameraPower 10
#define off false
#define on  true
//
#define ASCOM   Serial
#define Stepper Serial1
#define Monitor Serial2
// encoder:
// should this be set for 261 degrees? Otherwise the driver will request a move to 261 which will move the aperture out of alignment with the parked dome (and scope)
//  the position for 261 is set in Setup() below to reflect this.

volatile long A_Counter; // volatile because it's used in the interrupt routine

// General
String pkversion      = "4.0";
float Azimuth;                  // to be returned when a TX call is processed by this MCU
float SyncAz;
long azcount;
long Sendcount        = 0;
long pkinterval       = 0;
long pkstart          = 0;
float ticksperDomeRev = 10513;   // this was worked out empirically by counting the number of encoder wheel rotations for one dome rev. 11-9-21
long calltime         = 0;

void setup()
{

  pinMode(NorthPin, INPUT_PULLUP); // these are 4 microswitches for syncing the encoder
  pinMode(EastPin,  INPUT_PULLUP);
  pinMode(SouthPin, INPUT_PULLUP);
  pinMode(WestPin,  INPUT_PULLUP);

  // todo - the line below will need uncommenting and change to ensure it acts on the Rx lne for the seril line between Stepper and encoder
  pinMode(9, INPUT_PULLUP); // SEE THE github comments for this code - it pulls up the Rx line to 5v and transforms the hardware serial2 link's efficiency
  // pinMode(13, INPUT_PULLUP);                   // see the notes in github. this pulls up the serial Rx pin to 5v.
  //  notes for serial comms -
  ASCOM.begin  (19200);   // with ASCOM driver refer to DIP 40 pinout to get correct pin numbers for all the serial ports - see the google doc - 'Pin config for Radio Encoder MCU'
  Stepper.begin(19200); // with stepper MCU - change this to Serial1 when coding for 4809, see google doc - Pin config for Radio Encoder MCU
  Monitor.begin(19200); // with monitor program Change to Serial2 when coding for 4809,    see google doc - Pin config for Radio Encoder MCU

  // set up the LCD's number of columns and rows:
  // lcd.begin(16, 2);

  // encoder:
  pinMode(A_PHASE, INPUT);
  pinMode(B_PHASE, INPUT);

  // pins 2,3,18,19,20,21 are the only pins available to use with interrupts on the mega2560 (no pin limit)restrictions on 4809)

  attachInterrupt(digitalPinToInterrupt(A_PHASE), interrupt, RISING); // interrupt for the encoder device

  // interupts for the azimuth syncs below
  attachInterrupt(digitalPinToInterrupt(NorthPin), NorthSync, RISING);
  attachInterrupt(digitalPinToInterrupt(EastPin),  EastSync,  RISING);
  attachInterrupt(digitalPinToInterrupt(SouthPin), SouthSync, RISING);
  attachInterrupt(digitalPinToInterrupt(WestPin),  WestSync,  RISING);

  // lcd.setCursor(0, 0);
  // lcd.print("Az MCU Ver " + pkversion);                 //16 char display
  // delay(1000);                                   //so the message above can be seen before it is overwritten

  azcount   = 0;

  A_Counter = ticksperDomeRev / (360.0 / 261.0); //  the position of due west - 261 for the dome when the scope is at 270.

  PowerForCamera(off); // camera power is off by default

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

    if (azcount > 999)
    {
      azcount = 0;
    } // endif

  } // endif

  if (Monitor.available() > 0) // ser2 is comms with the windows forms arduino monitoring app
  {
    String MonitorData = Monitor.readStringUntil('#');

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

      Monitor.print(String(Azimuth) + "#"); // write the two monitoring values to the windows forms Arduino Monitor program
      Monitor.print(String(azcount) + "#");

    } // Endif

  } // endif

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
  A_Counter = ticksperDomeRev / 4.0;
}
void SouthSync()
{
  A_Counter = ticksperDomeRev / 2.0;
}
void WestSync()
{
  A_Counter = ticksperDomeRev * 0.75;
}

bool PowerForCamera(bool State)
{
  if (State)
  {
    digitalWrite(CameraPower, HIGH);
  }
  else
  {
    digitalWrite(CameraPower, LOW);
  }
}
