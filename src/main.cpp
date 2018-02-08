#include <Arduino.h>

// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption, and Automatic Transmission Control
// Passes through any wireless received messages to the serial port & responds to ACKs
// It also looks for an onboard FLASH chip, if present
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <Adafruit_INA219.h>
#include <Wire.h>
#define STREAM_ID_INFO_STR "\x1"
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        5    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************
#define SERIAL_BAUD   38400

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

#define GATEWAYID 1

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

class SerialInputBuffer
{
  public:
    uint8_t m_dataBuffer[62];
    char m_charPair[3];    
    uint8_t m_lastDataByte;

    // have serial number for the current message.
    bool m_haveSerialNumber;

    // the serial number of the previous message
    int8_t m_prevSerialNumber;

    int m_dataDigitCount;
    int m_dataByteCount;
    int m_errorCount;
    bool m_error;
    bool m_complete;

    SerialInputBuffer()
    {
      m_haveSerialNumber = false;
      m_errorCount = 0;
      startNewMessage();
    }

    void startNewMessage()
    {
      if (m_haveSerialNumber) 
      {
        m_prevSerialNumber = m_lastDataByte;
      }

      m_haveSerialNumber = false;
      m_charPair[2] = '\0';
      m_lastDataByte = 0;
      m_dataDigitCount = 0;
      m_dataByteCount = 0;
      m_error = false;
      m_complete = false;
    }

    // c is a hex digit
    void updateLastByteBuffer(char c)
    {
      m_lastDataByte >>= 4;
      uint8_t cAsInt = c - '0';
      m_lastDataByte |= cAsInt << 4;
    }

    // c is a hex digit
    void takePairs(char c)
    {
      if ((m_dataDigitCount % 2) == 1)
      {
        // This is the second of a pair of digits
        m_charPair[1] = c;
        m_dataBuffer[m_dataByteCount] = strtoul(m_charPair, 0, 16);
        m_dataByteCount++;
      }
      else
      {
        m_charPair[0] = c;
      }

      m_dataDigitCount++;
    }

    void addXDigit(char c)
    {
      updateLastByteBuffer(c);
      takePairs(c);
    }

    void endMessage()
    {
      // Message termination received.  Is the message any good?

      if (m_dataDigitCount >= 2)
      {
        // Interpret the last two digits as serial number even if they are out of alignment and the message is in error.
        m_haveSerialNumber = true;
      }

      // The message is bad if odd number of data digits received.
      if ((m_dataDigitCount % 2) == 1)
      {
              m_error = true;
      }
      else 
      {
        // Message is good if not empty and the serial number is correct.
        if (m_dataByteCount > 0)
        {
            // Reduce the data size by one byte because the last byte is the serial number.
            m_dataByteCount -= 1;
            m_dataDigitCount -= 2;

            // Verify good serial number
            if (m_lastDataByte == (m_prevSerialNumber + 1))
            {
              // All is good.
              // At this point the client will consume the message and notify the sender.
                m_complete = true;
            }
            else
            {
              m_error = true;
            }
        }
        else
        {
           // Empty message.  Continue as if nothing was received.
        }
      }
    }

    // Returns true when the end of a good message is detected.
    bool TakeFromSerial()
    {
      if (m_complete)
      {
        // The client already consumed the completed message before calling TakeFromSerial.
        startNewMessage();
      }

      // Pull data from serial until non available or we have complete message.
      while(!m_complete && Serial.available())
      {
        char c = (char)Serial.read();

        if (isxdigit(c))
        {
          addXDigit(c);
        }

        if ((c == '\n') || (c == '\r'))
        {
          endMessage();

          if (m_error)
          {
            m_errorCount++;
            startNewMessage();
          }
        }
      }

      return m_complete;
    }
};

SerialInputBuffer serialInputBuffer;

unsigned long timeOfNextPowerMessage;
Adafruit_INA219 ina219;
void setup() {
  ina219.begin(); 
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);
  //radio.setFrequency(919000000); //set frequency to some custom frequency
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  if (flash.initialize())
  {
    Serial.print("SPI Flash Init OK. Unique MAC = [");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      if (i!=8) Serial.print(':');
    }
    Serial.println(']');
    
    //alternative way to read it:
    //byte* MAC = flash.readUniqueId();
    //for (byte i=0;i<8;i++)
    //{
    //  Serial.print(MAC[i], HEX);
    //  Serial.print(' ');
    //}
  }
  else
    Serial.println("SPI Flash MEM not found (is chip soldered?)...");
    
#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif
    timeOfNextPowerMessage=millis()+5000;
}

void Blink(byte PIN, int DELAY_MS);


byte ackCount=0;
uint32_t packetCount = 0;

void printDataAsHex(uint8_t* dataBuffer, int len)
{
    for(int i = 0; i < len; ++i)
    {
      char twoChar[3];

      sprintf(twoChar, "%02x", dataBuffer[i]);
      Serial.print(twoChar);
    }
}

void mySerialEvent();

void loop() {

  unsigned long time = millis();
  mySerialEvent();

  // Data received from the radio gets delivered to Serial as 
  // hexidecimal 2 chars per byte no spaces, followed by EOL.
  if (radio.receiveDone())
  {
    printDataAsHex((uint8_t*)radio.DATA, radio.DATALEN);
    Serial.print('\n');

    if (radio.ACKRequested())
    {
      radio.sendACK();
    }
  }

  // Process any available serial input before doing other things that might take some time.
  mySerialEvent();

  if (time > timeOfNextPowerMessage)
  { 
    float busvoltage = 0;  
    float current_mA = 0;   
    busvoltage = ina219.getBusVoltage_V();  
    current_mA = ina219.getCurrent_mA();  

    char buff[62];
    char busVStr[6] = "     ";
    char currentStr[6] = "     ";
    dtostrf((double)busvoltage,5,3,busVStr);
    dtostrf((double)current_mA/1000.0,5,3,currentStr);

    sprintf(buff, STREAM_ID_INFO_STR "V:%s A:%s", busVStr, currentStr);
        // send over radio
    radio.send(GATEWAYID, buff, strlen(buff));
    timeOfNextPowerMessage = time + 60000;
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

const int REQUEST_SERIAL_SYNC = 13;
const int RETURN_SERIAL_SYNC = 12;

void SendSerialSyncResponse(uint8_t serialNumber)
{
  uint8_t data[2];
  data[0] = RETURN_SERIAL_SYNC;
  data[1] = serialNumber;
  printDataAsHex(data, 2);
  Serial.print('\n');
}

// For debug.  TODO add this to a status message sent over the radio.
uint32_t g_serialSyncCount = 0;

void mySerialEvent()
{
   if (Serial.available())
   {
      pinMode(LED, OUTPUT);
      digitalWrite(LED,HIGH);
   }

   // Keep processing messages until no data available.
   while (serialInputBuffer.TakeFromSerial())
   {
     // If the message is serial_sync, then respond on the serial port only and don't send over the radio.
     if (serialInputBuffer.m_dataBuffer[0] == REQUEST_SERIAL_SYNC && serialInputBuffer.m_dataByteCount == 1)
     {
       // keep a count for debugging.  Serial sync request should be infrequent.
       g_serialSyncCount++;
     }
     else 
     {
        // Send message over the radio. The serial number in the last byte was already removed from the data byte count.
        // Choosing to send over the radio *before* giving the sender the ok to send more data.  
        // This is because, if we spend too long inside radio.send, then the internal serial input buffer can overflow.
        radio.send(GATEWAYID, serialInputBuffer.m_dataBuffer, serialInputBuffer.m_dataByteCount);

     }
     // Tell the sender that we are done and therefore ok to send more.
     // Serial sync response is always sent.
     SendSerialSyncResponse(serialInputBuffer.m_lastDataByte);
   }

  digitalWrite(LED,LOW);
}