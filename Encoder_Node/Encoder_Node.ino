//  Name:       Two_Way_Encoder - with radio error checking
//   Created:  28/11/2018 08:46:37
//   Author:     DESKTOP-OCFJAV9\Paul
// Modified  to be modulo 360 by PK on 8-2-19
// Modified to respond to Serial1 Tx on 8-1-2020
// Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
//there are 25.6 rotations of the encoder wheel for one complete rotation of the dome.
// in a previous version without the toothed wheel around the perimeter, 26 encoder wheel revolutions  equalled 10413 encoder ticks
// from which it can bee calculated that 1 encoder wheel rev equates to 400.5 ticks.
//so in the new system with the toothed wheel around the perimeter, it takes 25.6 revs of the encoder wheel for 1 dome rotation
//In terms of ticks therefore, the total number of ticks for a dome revolution is 25.6 * 400.5 = 10253
//North = 0, East = 10253/4, South = 10253/2 East = 10253*3/4


// NB - start this MCU before the master radio so it is ready waiting for the comms check
// Dec 19 implemented  write counter to show how many radio writes of Azimuth are actually sent


#include <SPI.h>

#include <RF24.h>
#include <LiquidCrystal.h>

//for nrf to work, pin 10 must be high if it is not used as an nrf connecton
#define PIN10  10

//encoder:
#define  A_PHASE 2      // USES PINS 2 AND 3 in this version
#define  B_PHASE 3

RF24 radio(7, 8); // CE, CSN

//liquid crystal two lines below
const int rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const byte thisNodeaddress[6]   = "encod";            // 00001 the address of this arduino board/ transmitter
const byte masterNodeaddress[6] = "mastr";          // the address of the Master

const int channel  = 115;

char message[9]    = ""  ;                              // this data type must be used for radio.write
char commstest[17] = "Encoder online  ";

//encoder:
volatile long int A_Counter = 10253 * 0.75; // this is the position of due west


long int flag_B    = 0;

//General
String pkversion   = "2";
String blankline   = "                ";
String lcdazimuth;
double Azimuth;                                         // to be returned when a TX call is processed by this arduino board
long azcount;
long Sendcount     = 0;
long PKinterval    = 0;
long PKstart       = 0;
long PKcurrentTime = 0;
long calltime      = 0;


void setup()
{

  pinMode(19, INPUT_PULLUP);             //SEE THE github comments for this code - it pulls up the Rx line to 5v and transforms the hardware serial1 link's efficiency
  pinMode(PIN10, OUTPUT);                 // this is an NRF24L01 requirement if pin 10 is not used
  digitalWrite (PIN10, HIGH);            //NEW**********************
  Serial.begin(19200);
  Serial1.begin(19200);
  SPI.begin();

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  ConfigureRadio();       // start the radio with the settings we need

  //encoder:
  pinMode(A_PHASE, INPUT);
  pinMode(B_PHASE, INPUT);

  attachInterrupt(digitalPinToInterrupt( A_PHASE), interrupt, RISING); //Interrupt trigger mode: RISING

  lcd.setCursor(0, 0);
  lcd.print("Az MCU Ver " + pkversion);                 //16 char display
  delay(1000);                                   //so the message above can be seen before it is overwritten
  lcdprint(0, 0, "If comms work  a");
  lcdprint(0, 1, "counter shows.  ");
  delay(2000);                                   //so the message above can be seen before it is overwritten

  azcount = 0;
  PKstart = 0;   //interval timer for lcd update

}    // end setup


void loop()
{
  encoder();


  dtostrf(Azimuth, 7, 2, message); // convert double to char total width 7 with 2 dp for use by radio.write


  LCDUpdater();  // note timer is included in the routine

 

  if (Sendcount > 999)  //reset the radio send counts to zero
  {
    Sendcount = 0;
  }

  if (radio.available())
  {
    char text[32] = "";             // used to store what the master node sent e.g AZ hash SA hash

    radio.read(&text, sizeof(text));

    if (text[0] == 'A' && text[1] == 'Z' && text[2] == '#')
    {
      //note the radio does not write a # mark terminator - this is added in the two way radio code before send to driver
      radio.stopListening();
      radio.write(&message, sizeof(message));
     //test for timeout
     // test for lost config
     // test for radio always available

      radio.startListening();
      Sendcount++;
    }

    //end new
    // Serial.print("The text received from Master was: ");
    // Serial.println(text);
    // Serial.print("the Azimuth value returned to the master is ");
    // Serial.println(message);
    // Serial.println("--------------------------------------");

  }  //endif radio available


  if (Serial1.available() > 0)
  {
    encoder();
    String ReceivedData = "";

    ReceivedData = Serial1.readStringUntil('#');
    // Serial.print("received ");
    // Serial.println(ReceivedData );
    if (ReceivedData.indexOf("AZ", 0) > -1) //
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

void ConfigureRadio()
{

  radio.begin();
  radio.setChannel(channel);
  radio.enableAckPayload();            // enable ack payload - slaves reply with data using this feature
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.enableDynamicPayloads();
  radio.openWritingPipe(masterNodeaddress);
  radio.openReadingPipe(1, thisNodeaddress);    // "MASTR" the address the master writes to when communicating with this encoder node
  radio.startListening();

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
