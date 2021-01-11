//Version 3.0 - change the variable too version number introduced Jan 2020
//  Name:       Azimuth Encoder
//   Created:  28/11/2018 08:46:37
//   Author:     DESKTOP-OCFJAV9\Paul
// Modified  to be modulo 360 by PK on 8-2-19
// Modified to respond to Serial1 Tx on 8-1-2020 - changed to Serial2 on 4-8-20

// if it's required to set a home point or park point marker A_Counter is the variable which must be set.
// see below for north, east, south, west values for A_Counter
// for any position X degrees the tick position to set for A_counter is 10253 / (360/ x )


//there are 25.6 rotations of the encoder wheel for one complete rotation of the dome.
// in a previous version without the toothed wheel around the perimeter, 26 encoder wheel revolutions  equalled 10413 encoder ticks
// from which it can bee calculated that 1 encoder wheel rev equates to 400.5 ticks.
//so in the new system with the toothed wheel around the perimeter, it takes 25.6 revs of the encoder wheel for 1 dome rotation
//In terms of ticks therefore, the total number of ticks for a dome revolution is 25.6 * 400.5 = 10253
//North = 0
//East = 10253/4   = 2563
//South = 10253/2  = 5127
//West = 10253*3/4 = 7690





// Radio is no longer in this routine, it operates via USB cable. Bye Bye Radio....

#include <LiquidCrystal.h>


//encoder:
#define  A_PHASE  2      // USES PINS 2 AND 3 for encoder interrupt
#define  B_PHASE  3
#define  NorthPin 18
#define  EastPin  19
#define  SouthPin 20
#define  WestPin  21

//liquid crystal two lines below
const int rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
// LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//encoder:
//should this be set for 261 degrees? Otherwise the driver will request a move to 261 which will move the aperture out of alignment with the parked dome (and scope)
// the position for 261 is 7434 and is changed below to reflect this.

volatile long int A_Counter ;


long int flag_B = 0;

//General
String pkversion = "3.0";
String blankline = "                ";
String lcdazimuth;
double Azimuth;                                         // to be returned when a TX call is processed by this arduino board
float  SyncAz;
long   azcount;
long   Sendcount   = 0;
long   pkinterval    = 0;
long   pkstart       = 0;

long calltime = 0;


void setup()
{

  pinMode(NorthPin,   INPUT_PULLUP);             // these are 4 microswitches for syncing the encoder
  pinMode(EastPin,    INPUT_PULLUP);
  pinMode(SouthPin,   INPUT_PULLUP);
  pinMode(WestPin,    INPUT_PULLUP);

  pinMode(17,         INPUT_PULLUP);             //SEE THE github comments for this code - it pulls up the Rx line to 5v and transforms the hardware serial2 link's efficiency

  Serial.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);


  // set up the LCD's number of columns and rows:
  // lcd.begin(16, 2);


  //encoder:
  pinMode(A_PHASE, INPUT);
  pinMode(B_PHASE, INPUT);

  // pins 2,3,18,19,20,21 are the only pins available to use with interrupts on the mega2560

  attachInterrupt(digitalPinToInterrupt( A_PHASE),  interrupt, RISING); //Interrupt trigger mode: RISING - really?
  attachInterrupt(digitalPinToInterrupt( NorthPin), NorthSync, RISING);
  attachInterrupt(digitalPinToInterrupt( EastPin),  EastSync,  RISING);
  attachInterrupt(digitalPinToInterrupt( SouthPin), SouthSync, RISING);
  attachInterrupt(digitalPinToInterrupt( WestPin),  WestSync,  RISING);

  //lcd.setCursor(0, 0);
  //lcd.print("Az MCU Ver " + pkversion);                 //16 char display
  //delay(1000);                                   //so the message above can be seen before it is overwritten

  azcount = 0;

  A_Counter = 7433  ;      // THIS IS 10253 / (360 / 261) - the position of due west - 261 for the dome when the scope is at 270.


}    // end setup


void loop()
{

  encoder();

  if (Serial.available() > 0)
  {
    encoder();
    String ReceivedData = "";

    ReceivedData = Serial.readStringUntil('#');
    // Serial.print("received ");
    // Serial.println(ReceivedData );
    if (ReceivedData.indexOf("AZ", 0) > -1) //
    {

      Serial.print(String(Azimuth) + "#");
    }



    // Sync to Azimuth code below:
    if (ReceivedData.indexOf("STA", 0) > -1) //
    {

      ReceivedData.remove(0, 3);  //strip 1st three chars
      SyncAz = ReceivedData.toFloat();
      if ((SyncAz > 0.0) && (SyncAz <= 360.0))   // check for a valid azimuth value
      {
        // now work out the tick position for this azimuth
        // Number of ticks per degree is 28.481 - see comments at top of this code
        A_Counter = SyncAz * 28.481;
      } // endif
    }  // endif

    //WAS HERE    LCDUpdater();

  }
  // LCDUpdater();//now here

  if (Serial2.available() > 0)   // ser2 is encoder with stepper
  {
    encoder();
    String ReceivedData = "";

    ReceivedData = Serial2.readStringUntil('#');
    // Serial.print("received ");
    // Serial.println(ReceivedData );
    if (ReceivedData.indexOf("AZ", 0) > -1)
    {
      azcount++;
      Serial2.print(String(Azimuth) + "#");
    } // endif

    if (azcount > 999)
    {
      azcount = 0;
    } // endif

    //  LCDUpdater();

  } // endif

  if (Serial3.available() > 0)  // ser3 is comms with the windows forms arduino monitoring app
  {
    String Ser3Data = Serial3.readStringUntil('#');
    if (Ser3Data.indexOf("EncoderRequest", 0) > -1)
    {
      Serial3.print(String(Azimuth) + "#");     // write the two monitoring values to the windows forms Arduino Monitor program
      Serial3.print(String(azcount) + "#");

    } // Endif

  } // endif

}  // end void loop

void encoder()
{
  //Encoder:

  if (A_Counter < 0)
  {
    A_Counter =  A_Counter + 10253;     // set the counter floor value
  }

  if (A_Counter > 10253)   // set the counter ceiling value
  {
    A_Counter = A_Counter -  10253;
  }

  Azimuth = float(A_Counter) / 28.481;    // 28.481 is 10253 (counts for 25.6 revs) / 360 (degrees)
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



  // Serial2.print (String(Azimuth, 2)); //call the function and print the angle returned to serial
  // Serial2.println("#");               // print the string terminator
  // receivedData = "";

}  // end void encoder

//interrupt used by the encoder:
void interrupt()               // Interrupt function
{

  char i , j;
  i = digitalRead( B_PHASE);
  j = digitalRead( A_PHASE);
  if (i == j)
  {
    A_Counter -= 1;

  }

  else
  {

    A_Counter += 1;   // increment the counter which represents increasing dome angle


  }  // end else clause
}  // end void interrupt


void NorthSync()
{
  A_Counter = 0;
}
void EastSync()
{
  A_Counter = 2563;
}
void SouthSync()
{
  A_Counter = 5127;
}
void WestSync()
{
  A_Counter = 7690;
}



/*
  void lcdprint(int col, int row, String mess )
  {
  //lcd.clear();
  lcd.setCursor(col, row);
  lcd.print(mess);

  }

  void LCDUpdater()
  {

  // in new version azcount is the var to send to the monitor code
  // the LCDUpdater will be renamed and doesn't need the timesincelastupdate funstion as updates will be requested via
  // a serial connection

  long timesincelastupdate;

  timesincelastupdate = millis() - calltime;
  if (timesincelastupdate > 500)
  {
    //Update Code here


    lcdprint(0, 0, blankline);
    lcdprint(0, 1, blankline);
    lcdprint(0, 0, "rad:step " + String(Sendcount) + ":" );
    lcdprint(13, 0, String(azcount));
    lcdprint(0, 1, "Azimuth: " + String(Azimuth, 0) );

    calltime = millis();
  }

  }     //end void updater
*/
