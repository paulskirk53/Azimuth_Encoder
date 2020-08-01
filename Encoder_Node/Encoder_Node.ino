//Version 2.0 - change the variable too version number introduced Jan 2020
//  Name:       Azimuth Encoder
//   Created:  28/11/2018 08:46:37
//   Author:     DESKTOP-OCFJAV9\Paul
// Modified  to be modulo 360 by PK on 8-2-19
// Modified to respond to Serial1 Tx on 8-1-2020

// if it's required to set a home point or park point marker A_Counter is the variable which must be set.
// see below for north, east, south, west values for A_Counter
// for any position X degrees the tick position to set for A_counter is 10253 / (360/ x )




//there are 25.6 rotations of the encoder wheel for one complete rotation of the dome.
// in a previous version without the toothed wheel around the perimeter, 26 encoder wheel revolutions  equalled 10413 encoder ticks
// from which it can bee calculated that 1 encoder wheel rev equates to 400.5 ticks.
//so in the new system with the toothed wheel around the perimeter, it takes 25.6 revs of the encoder wheel for 1 dome rotation
//In terms of ticks therefore, the total number of ticks for a dome revolution is 25.6 * 400.5 = 10253
//North = 0, East = 10253/4, South = 10253/2 West = 10253*3/4





// Radio is no longer in this routine, it operates via USB cable. Bye Bye Radio....

#include <LiquidCrystal.h>


//encoder:
#define  A_PHASE 2      // USES PINS 2 AND 3 in this version
#define  B_PHASE 3

//liquid crystal two lines below
const int rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//encoder:
//should this be set for 261 degrees? Otherwise the driver will request a move to 261 which will move the aperture out of alignment with the parked dome (and scope)
// the position for 261 is 7434 and is changed below to reflect this.

volatile long int A_Counter = 10253 / (360/ 261); // this is the position of due west - 261 for the dome when the scope is at 270.


long int flag_B = 0;

//General
String pkversion = "2.0";
String blankline = "                ";
String lcdazimuth;
double Azimuth;                                         // to be returned when a TX call is processed by this arduino board
long   azcount;
long   Sendcount   = 0;
long   pkinterval    = 0;
long   pkstart       = 0;

long calltime = 0;


void setup()
{

  pinMode(19, INPUT_PULLUP);             //SEE THE github comments for this code - it pulls up the Rx line to 5v and transforms the hardware serial1 link's efficiency


  Serial.begin(19200);
  Serial1.begin(19200);


  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);


  //encoder:
  pinMode(A_PHASE, INPUT);
  pinMode(B_PHASE, INPUT);

  attachInterrupt(digitalPinToInterrupt( A_PHASE), interrupt, RISING); //Interrupt trigger mode: RISING

  lcd.setCursor(0, 0);
  lcd.print("Az MCU Ver " + pkversion);                 //16 char display
  delay(1000);                                   //so the message above can be seen before it is overwritten
  
  azcount = 0;

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
      azcount++;
      Serial.print(String(Azimuth) + "#");
    }

    if (azcount > 999)
    {
      azcount = 0;
    }

    LCDUpdater();

  }


  if (Serial1.available() > 0)
  {
    encoder();
    String ReceivedData = "";

    ReceivedData = Serial1.readStringUntil('#');
    // Serial.print("received ");
    // Serial.println(ReceivedData );
    if (ReceivedData.indexOf("AZ", 0) > -1) 
    {
      azcount++;
      Serial1.print(String(Azimuth) + "#");
    }
    if (azcount > 999)
    {
      azcount = 0;
    }

    LCDUpdater();

  }


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



  // Serial1.print (String(Azimuth, 2)); //call the function and print the angle returned to serial
  // Serial1.println("#");               // print the string terminator
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

void lcdprint(int col, int row, String mess )
{
  //lcd.clear();
  lcd.setCursor(col, row);
  lcd.print(mess);

}

void LCDUpdater()
{
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
