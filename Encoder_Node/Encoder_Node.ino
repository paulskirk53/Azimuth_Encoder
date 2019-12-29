
//  Name:       Two_Way_Encoder
//   Created:	28/11/2018 08:46:37
//   Author:     DESKTOP-OCFJAV9\Paul
// Modified  to be modulo 360 by PK on 8-2-19
// Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
//there are 25.6 rotations of the encoder wheel for one complete rotation of the dome.
// in a previous version without the toothed wheel around the perimeter, 26 encoder wheel revolutions  equalled 10413 encoder ticks
// from which it can bee calculated that 1 encoder wheel rev equates to 400.5 ticks.
//so in the new system with the toothed wheel around the perimeter, it takes 25.6 revs of the encoder wheel for 1 dome rotation
//In terms of ticks therefore, the total number of ticks for a dome revolution is 25.6 * 400.5 = 10253


//NB - start this MCU before the master radio so it is ready waiting for the comms check
// Dec 19 implemented  write counter to show how many radio writes of Azimuth are actually sent

#include <SoftwareSerial.h>
#include <SPI.h>
#include <nRF24L01.h>
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

const byte thisNodeaddress[6] = "encodr";            // 00001 the address of this arduino board/ transmitter
const byte masterNodeaddress[6] = "mastr";          // the address of the Master
char message[9] = ""  ;                              // this data type must be used for radio.write
char commstest[17] = "Encoder online   ";

//encoder:
volatile long int A_Counter = 8000;  // this is the position of due north
// the starting point of the dome must be facing north

long int flag_B = 0;

//General
String receivedData = "";
String lcdazimuth;
double Azimuth;                                         // to be returned when a TX call is processed by this arduino board
long   Sendcount = 0;



void setup()
{
  pinMode(PIN10, OUTPUT);                 // this is an NRF24L01 requirement if pin 10 is not used
  digitalWrite (PIN10, HIGH);            //NEW**********************
  Serial.begin(115200);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);


  radio.begin();
  radio.setChannel(100);
  radio.enableAckPayload();            // enable ack payload - slaves reply with data using this feature
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.enableDynamicPayloads();
  radio.openWritingPipe(masterNodeaddress);
  radio.openReadingPipe(1, thisNodeaddress);    // "000001" the address the master writes to when communicating with this encoder node
  radio.startListening();


  //encoder:
  pinMode(A_PHASE, INPUT);
  pinMode(B_PHASE, INPUT);
  Serial.begin(9600);   //Serial Port Baudrate: 9600
  attachInterrupt(digitalPinToInterrupt( A_PHASE), interrupt, RISING); //Interrupt trigger mode: RISING

  lcd.setCursor(0, 0);
  lcd.print("Azimuth MCU OK  ");                 //16 char display
  delay(4000);                                   //so the message above can be seen before it is overwritten
  // lcdprint(0, 0, "If comms work  a");
  // lcdprint(0, 1, "counter shows.  ");
  delay(4000);                                   //so the message above can be seen before it is overwritten
}    // end setup


void loop()
{
  //radio.startListening();
  // delay(20);                    // just in case hardware needs time before next instuction exec

  while (!radio.available())
  {
    encoder();
    dtostrf(Azimuth, 7, 2, message); // convert double to char total width 7 with 2 dp for use by radio.write

    // lcdazimuth = String(message);
    // set the cursor to column 0, line 0
    // (note: line 1 is the second row, since counting begins with 0):
    lcdprint(0, 0, "                ");
    lcdprint(0, 1, "                ");
    lcdprint(0, 0, "Call count: " + String(Sendcount));

    lcdprint(0, 1, "Azimuth: " + String(Azimuth, 0));



  }

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
      radio.startListening();
      Sendcount++;
    }

    //new code
    if ((text[0] == 'T') && (text[1] == 'S') && (text[2] == 'T') && (text[3] == '#'))
    {
      //note the radio does not write a # mark terminator - this is added in the two way radio code before send to driver
      radio.stopListening();
      radio.write(&commstest, sizeof(commstest));
      radio.startListening();
      lcdprint(0, 1, "                ");
      lcdprint(0, 0, "                ");
      lcdprint(0, 0, "Responding to   ");
      lcdprint(0, 1, "Comms check...  ");
      delay(5000);
      lcdprint(0, 0, "                ");
      lcdprint(0, 1, "                ");
      Sendcount++;
    }


    //end new
    // Serial.print("The text received from Master was: ");
    // Serial.println(text);
    // Serial.print("the Azimuth value returned to the master is ");
    // Serial.println(message);
    // Serial.println("--------------------------------------");

  }  //endif radio available

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
  if (Azimuth < 0)
  {
    Azimuth = 0.0;
  }

  if (Azimuth > 360.0)
  {
    Azimuth = 360.0;
  }



  // Serial1.print (String(Azimuth, 2)); //call the function and print the angle returned to serial
  // Serial1.println("#");               // print the string terminator
  receivedData = "";

}

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

void lcdprint(int col, int row, String mess)
{
  //lcd.clear();
  lcd.setCursor(col, row);
  lcd.print(mess);

}
