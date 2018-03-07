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
//#include <avr/power.h>
//#include <avr/sleep.h>
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
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

#undef SERIAL_EN
#ifdef SERIAL_EN
#define DEBUG2(input1,input2) {Serial.print(input1,input2);}
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#define DEBUGFlush() { Serial.flush(); }
#else
#define DEBUG2(input1,input2);
#define DEBUG(input);
#define DEBUGln(input);


#define DEBUGFlush();
#endif

#define BATTERY_SAMPLE_INTERVAL_MS 2000
#define BATTERY_TRANSMIT_INTERVAL_MS 120000
#define WAIT_FOR_RECEIVE_TIME_MS 10000

#define GATEWAYID 1

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

void sendToGateway(void* buffer, int byteCount);
void printDataAsHex(uint8_t* dataBuffer, int len);
void mySerialEvent();
void Blink(byte PIN, int DELAY_MS);

class SerialInputBuffer
{
  public:
    uint8_t m_dataBuffer[63];
    uint8_t m_lastDataByte;

    // have serial number for the current message.
    bool m_haveSerialNumber;

    // the serial number of the previous message
    uint8_t m_prevSerialNumber;

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

      m_dataBuffer[0] = 0;
      m_haveSerialNumber = false;
      m_lastDataByte = 0;
      m_dataDigitCount = 0;
      m_dataByteCount = 0;
      m_error = false;
      m_complete = false;
    }

    // c is a hex digit
    uint8_t xDigitToNumber(char c)
    {
      c = toUpperCase(c);
      if (c >= 'a')
      {
        return 10 + c - 'a';
      }
      if (c >= 'A')
      {
        return 10 + c - 'A';
      }
      return c - '0';
    }

    // c is a hex digit
    void updateLastByteBuffer(char c)
    {
      m_lastDataByte <<= 4;
      m_lastDataByte |= xDigitToNumber(c);
      DEBUGln(m_lastDataByte);
    }

    // c is a hex digit
    void takePairs(char c)
    {
      if ((m_dataDigitCount % 2) == 1)
      {
        // This is the second (lsb) of a pair of digits
        m_dataBuffer[m_dataByteCount] |= xDigitToNumber(c);
        m_dataByteCount++;
        // Clear out the next byte
        m_dataBuffer[m_dataByteCount] = 0;
      }
      else
      {
        m_dataBuffer[m_dataByteCount] |= (xDigitToNumber(c) << 4);
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
//DEBUG("m_dataDigitCount="); DEBUGln(m_dataDigitCount);
//DEBUG("m_dataByteCount="); DEBUGln(m_dataByteCount);
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
//DEBUG("m_lastDataByte="); DEBUGln(m_lastDataByte);
//DEBUG("m_prevSerialNumber="); DEBUGln(m_prevSerialNumber);
            // Verify good serial number
            if (m_lastDataByte == (uint8_t)(m_prevSerialNumber + 1))
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
//        DEBUGln("Start new message");
        // The client already consumed the completed message before calling TakeFromSerial.
        startNewMessage();
      }

      // Pull data from serial until non available or we have complete message.
      while(!m_complete && Serial.available())
      {
        char c = (char)Serial.read();

  //      DEBUG("read=");DEBUG(c);DEBUGln("");
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


//---------------------------------------------------------------------------------------------------
//  POWER MONITORING
//---------------------------------------------------------------------------------------------------
class MinMaxAvg
{
public:
    MinMaxAvg(float weightOfMostRecent)
    : m_min(0)
    , m_max(0)
    , m_EWMAverage(0)
    , m_noPreviousInput(true)
    {
      m_weightOfRecent = weightOfMostRecent;
      m_weightOfHistory = 1-weightOfMostRecent;
    }

    float m_weightOfRecent;
    float m_weightOfHistory;
    float m_min;
    float m_max;
    /// Exponential weighted moving average
    float m_EWMAverage;
    bool m_noPreviousInput;

    char m_minStr[6];
    char m_maxStr[6];
    char m_avgStr[6];

    void input(float d)
    {
      if (m_noPreviousInput)
      {
        m_min = d;
        m_max = d;
        m_EWMAverage = d;
        m_noPreviousInput = false;
      }
      else
      {
        if (m_max < d)
        {
          m_max = d;
        }
        if (m_min > d)
        {
          m_min = d;
        }

        m_EWMAverage = m_weightOfRecent * d + m_weightOfHistory * m_EWMAverage;
      }
    }

    void reset()
    {
      m_noPreviousInput = true;
    }

    static char* toString(char* buf, float val, double scalar)
    {
      memset(buf, 0, 6);
      dtostrf((double)val*scalar, 5, 3, buf);
      return buf;
    }

    char* minToString(double scalar = 1.0)
    {
      return toString(m_minStr, m_min, scalar);
    }

    char* maxToString(double scalar = 1.0)
    {
      return toString(m_maxStr, m_max, scalar);
    }

    char* avgToString(double scalar = 1.0)
    {
      return toString(m_avgStr, m_EWMAverage, scalar);
    }
};

class PowerMonitor
{
  Adafruit_INA219 m_ina219;

  public:
    MinMaxAvg m_busVoltage;
    MinMaxAvg m_current_mA;

  PowerMonitor()
  : m_busVoltage(0.2)
  , m_current_mA(0.2)
  {

  }

  void setup()
  {
    m_ina219.begin();
  }

  void sample()
  {
    float v = m_ina219.getBusVoltage_V();  
    float mA = m_ina219.getCurrent_mA();  

    m_busVoltage.input(v);
    m_current_mA.input(mA);
  }
};

class RecurringAction
{
public:
    RecurringAction(unsigned long intervalMS)
    {
      m_intervalMS = intervalMS;
    }

    virtual void doAction()
    {}

    void startTimer(unsigned long now)
    {
      DEBUG("start");
      m_time = now + m_intervalMS;
      DEBUG(now); DEBUG(" "); DEBUGln(m_time);
      m_started = true;
    } 

    void doActionIfTime(unsigned long now)
    {
        if (!m_started)
        {
          startTimer(now);
        }
        else
        {
          if (now > m_time)
          {
            doAction();
            startTimer(now);
          }
        }
    }

    // the time when the event will be signaled next
    unsigned long m_time;

    // the millisecond interval between occurrences
    unsigned long m_intervalMS;
    bool m_started;

    RecurringAction()
    {
      m_started = false;
    }
};


class XmitBatteryStatus: public RecurringAction
{
  PowerMonitor& m_pm;

  public:

  XmitBatteryStatus(PowerMonitor& pm)
  : RecurringAction(BATTERY_TRANSMIT_INTERVAL_MS)
  , m_pm(pm)
  {}

  virtual void doAction()
  {
 DEBUGln("XmitBatteryStatus");
    MinMaxAvg& busV = m_pm.m_busVoltage;
    MinMaxAvg& mA = m_pm.m_current_mA;

    char buff[62];
    sprintf(buff, STREAM_ID_INFO_STR "VMIN:%s AMIN:%s VMAX:%s AMAX:%s", 
        busV.minToString(), mA.minToString(1.0/1000.0),
        busV.maxToString(), mA.maxToString(1.0/1000.0)
        );
    sendToGateway(buff, strlen(buff));

    sprintf(buff, STREAM_ID_INFO_STR "V:%s A:%s", busV.avgToString(), mA.avgToString(1.0/1000.0));
    DEBUGln(buff);
    sendToGateway(buff, strlen(buff));

    busV.reset();
    mA.reset();
  }
};


class SampleBatteryStatus: public RecurringAction
{
  PowerMonitor& m_pm;

  public:
    SampleBatteryStatus(PowerMonitor& pm)
  : RecurringAction(BATTERY_SAMPLE_INTERVAL_MS)
  , m_pm(pm)
  {}

  virtual void doAction()
  {
    m_pm.sample();
  }
};

// EnableSleepMode triggers every 100ms.
// The system disables sleep mode when the radio transmits.
// This action will reenable sleep mode after 100ms.
// The action timer needs to be reset when sleep mode is disabled.
class EnableSleepMode : public RecurringAction
{
  bool& m_sleepModeFlag;

  public:
  EnableSleepMode(bool& sleepModeFlag)
  : RecurringAction(WAIT_FOR_RECEIVE_TIME_MS)
  , m_sleepModeFlag(sleepModeFlag)
  {
  }

  void reset()
  {
    startTimer(millis());
    m_sleepModeFlag = false;
  }

  virtual void doAction()
  {
    m_sleepModeFlag = true;
  }
};

class RecurringActionManager
{
  public:

  enum Constants
  {
    MAX_ACTION_COUNT = 5
  };

  int m_count;

  RecurringAction* m_actions[MAX_ACTION_COUNT] = {0};

  void doAll()
  {
    unsigned long now = millis();
    for (int i = 0; i < m_count; i++)
    {
      RecurringAction* a = m_actions[i];
      
      if (0 == a)
        continue;

      a->doActionIfTime(now); 
    }
  }
  void add(RecurringAction* a)
  {
    if (m_count < MAX_ACTION_COUNT)
    {
       m_actions[m_count++] = a;
    }
  }
//  unsigned long getTimeNow()
//  {
//    return m_now;
//  }
    
  RecurringActionManager()
  {
    m_count = 0;

  }
};


PowerMonitor m_battery;

bool g_sleep = false;

EnableSleepMode sleepMode(g_sleep);
XmitBatteryStatus batteryXmit(m_battery);
SampleBatteryStatus sampleBattery(m_battery);

SerialInputBuffer serialInputBuffer;
RecurringActionManager timeTriggeredEvents;

void setup() {
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
  DEBUGln(buff);

  if (flash.initialize())
  {
      flash.sleep(); // if Moteino has FLASH-MEM, make sure it sleeps
  }
//  {
//
//    DEBUG("SPI Flash Init OK. Unique MAC = [");
// 
//    flash.readUniqueId();
//    for (byte i=0;i<8;i++)
//    {
//      DEBUG2(flash.UNIQUEID[i], HEX);
//      if (i!=8) DEBUG(':');
//    }
//    DEBUGln(']');
    
    //alternative way to read it:
    //byte* MAC = flash.readUniqueId();
    //for (byte i=0;i<8;i++)
    //{
    //  DEBUG(MAC[i], HEX);
    //  DEBUG(' ');
    //}
  //}
  //else
  //  DEBUGln("SPI Flash MEM not found (is chip soldered?)...");
    
#ifdef ENABLE_ATC
  DEBUGln("RFM69_ATC Enabled (Auto Transmission Control)");
#endif

  m_battery.setup();

  timeTriggeredEvents.add(&batteryXmit);
  timeTriggeredEvents.add(&sampleBattery);
  timeTriggeredEvents.add(&sleepMode);
}

void sleep()
{

  DEBUGln( "sleeping\n" );
  DEBUGFlush();
//time = time + 2000 + millis() - now;

// IMPORTANT to sleep radio before going idle.  If the radio receives something during idle, then receiveDone hangs after wakeup. 
  radio.sleep();  
  LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_ON, TWI_OFF);
  
//  set_sleep_mode( SLEEP_MODE_IDLE );
//  power_all_disable();
//  power_usart0_enable();
//  sleep_mode();
//  power_all_enable();
  DEBUGln( "awake" );
}
void loop() {
  
  mySerialEvent();

  //bool rd = radio.receiveDone()
  // Data received from the radio gets delivered to Serial as 
  // hexidecimal 2 chars per byte no spaces, followed by EOL.
  if (radio.receiveDone())
  {
	// Copy the data from the radio before transmitting ACK.
    uint8_t radioDataBuffer[70];
    int radioDataLen = radio.DATALEN;
    memcpy(radioDataBuffer,(void*)(&radio.DATA[0]),radioDataLen);

	// Return ack as quickly as possible.
    if (radio.ACKRequested())
    {
      radio.sendACK();
    }	
	
	// *AFTER* returning ack, process the received data.
    DEBUG("datalen:");DEBUGln(radioDataLen);
    printDataAsHex((uint8_t*)radioDataBuffer, radioDataLen);
    Serial.print('\n');
  }

  // Process any available serial input before doing other things that might take some time.
  mySerialEvent();


  timeTriggeredEvents.doAll();

  if (g_sleep)
  {
    sleep();
  }
}

void printDataAsHex(uint8_t* dataBuffer, int len)
{
    for(int i = 0; i < len; ++i)
    {
      char twoChar[3];

      sprintf(twoChar, "%02x", dataBuffer[i]);
      Serial.print(twoChar);
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
const int RETURN_STATUS = 14;
const int NOTIFY_SLEEP = 16;

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
     DEBUG("z m_dataDigitCount="); DEBUGln(serialInputBuffer.m_dataDigitCount);
DEBUG("z m_dataByteCount="); DEBUGln(serialInputBuffer.m_dataByteCount);
DEBUG("z m_dataBuffer[0]="); DEBUGln(serialInputBuffer.m_dataBuffer[0]);
     // If the message is serial_sync, then respond on the serial port only and don't send over the radio.
     if (serialInputBuffer.m_dataBuffer[0] == REQUEST_SERIAL_SYNC && serialInputBuffer.m_dataByteCount == 1)
     {
       // keep a count for debugging.  Serial sync request should be infrequent.
       DEBUGln("sync");
       g_serialSyncCount++;
     }
 //    else if (serialInputBuffer.m_dataBuffer[0] == NOTIFY_SLEEP && serialInputBuffer.m_dataByteCount == 2 )
 //    {
 //       if (serialInputBuffer.m_dataBuffer[1])
 //       {
 //         DEBUGln("sleep on");
 //         g_sleep = true;
//        }
 //       else
 //       {
 //         DEBUGln("sleep off");
 //         g_sleep = false;
 //       }
 //    }
    else 
    {
        // If the message is a status message, then send the battery status also.
        if (serialInputBuffer.m_dataBuffer[0] == RETURN_STATUS)
        {
         sampleBattery.doAction();
         batteryXmit.doAction();
        }
        // Send message over the radio. The serial number in the last byte was already removed from the data byte count.
        // Choosing to send over the radio *before* giving the sender the ok to send more data.  
        // This is because, if we spend too long inside radio.send, then the internal serial input buffer can overflow.
		// After this the gateway might quickly send a new message.
        sendToGateway(serialInputBuffer.m_dataBuffer, serialInputBuffer.m_dataByteCount);
     }
     // Tell the sender that we are done and therefore ok to send more.
     // Serial sync response is always sent.
     SendSerialSyncResponse(serialInputBuffer.m_lastDataByte);
   }

  digitalWrite(LED,LOW);
}

void sendToGateway(void* buffer, int byteCount)
{
  radio.sendWithRetry(GATEWAYID, buffer, byteCount);
  sleepMode.reset();
}

