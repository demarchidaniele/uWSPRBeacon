


/*
  uWSPR Beacon
  Ethernet Beacon Controller
  (C) IU3AGC Daniele De Marchi

  WSPR Beacon With AD9851 DDS and Ethernet Controller
  Generates WSPR coordinated frequency hopping transmissions
  on 6 thru 160 meters synchronized by GPS and/or NTP time data.

  Acknowlegements
  The on-chip generation of the WSPR message algorithm is the work of
  Andy Talbot, G4JNT.

  Copyright (C) 2016, IU3AGC Daniele De Marchi

  Permission is granted to use, copy, modify, and distribute this software
  and documentation as described in GNU GPL V3.

    _________________________________________________________________________

  Nano Digital Pin Allocation
  D0
  D1
  D2  
  D3
  D4  
  D5  Serial Rx - GPS-Tx (Needed by the serial lib but not used)
  D6  DDS9851 Data
  D7  DDS9851 Load
  D8  DDS9851 Clock
  D9  
  D10 
  D11 Serial Rx - GPS-Tx
  D12 Serial Tx - GPS-Rx
  D13 TX (LED)
  D14 LCD D7
  D15 LCD D6
  D16 LCD D5
  D17 LCD D4
  D18 LCD enable
  D19 LCD RS
  ------------------------------------------------------------
*/
// include the library code:
//__________________________________________________________________________________________________
// ENTER WSPR DATA:
char call[7] = "IU3AGC";
char locator[5] = "JN55"; // Use 4 character locator e.g. "EM64"


#include <MsTimer2.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#define EE_MAGIC    0x00

#define EE_DHCP     0x10
#define EE_MAC     (EE_DHCP    + 0x01)
#define EE_IP      (EE_MAC     + 0x06)
#define EE_GW      (EE_IP      + 0x04)
#define EE_NETMASK (EE_GW      + 0x04)
#define EE_DNS     (EE_NETMASK + 0x04)
#define EE_NTP     (EE_DNS     + 0x04)

#define EE_CALL    (EE_NTP     + 0x04)
#define EE_LOCATOR (EE_CALL    + 0x06)

#define EE_FR_CALIB (EE_LOCATOR  + 0x04)
#define EE_FREQ     (EE_FR_CALIB + 0x04)
#define EE_TX_FLAG  (EE_FREQ     + 0x04 * 10)
#define EE_POWER    (EE_TX_FLAG  + 0x01 * 10)
#define EE_NEXT     (EE_POWER    + 0x01 * 10)

#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))

//_ ETHERCARD _______________________________________________________________________________________
#include <enc28j60.h>
#include <EtherCard.h>
#include <net.h>
// IP and netmask allocated by DHCP
//static uint8_t myip[4] = { 0, 0, 0, 0 };
//static uint8_t mynetmask[4] = { 0, 0, 0, 0 };
//static uint8_t gwip[4] = { 0, 0, 0, 0 };
//static uint8_t dnsip[4] = { 0, 0, 0, 0 };
//static uint8_t dhcpsvrip[4] = { 0, 0, 0, 0 };
static uint8_t ntp_server[4];
//static uint8_t mymac[6];
// Packet buffer, must be big enough to packet and payload
#define BUFFER_SIZE 200
byte Ethernet::buffer[BUFFER_SIZE];
// NTP Variables

bool ntp_querySent;
unsigned char ntp_query_timeout;
#define ntp_tcpPort 0
uint32_t ntp_timeStamp;

//_ LCD __________________________________________________________________________________________
#include <LiquidCrystal.h>
LiquidCrystal lcd(19, 18, 17, 16, 15, 14);

//_ GPS __________________________________________________________________________________________
#include <SoftwareSerial.h>
#include <MicroNMEA.h>
#define GPS_RXPin 9
#define GPS_TXPin 5
SoftwareSerial GPSSerial(GPS_RXPin, GPS_TXPin);   // Rx, Tx (not used)
char gps_buffer[85];
MicroNMEA GPS(gps_buffer, sizeof(gps_buffer));

//___________________________________________________________________________________________________
struct ulong_octets {
  union {
    unsigned long value;
    struct {
      unsigned char byte[4];
    }
    __attribute__((packed));
  }
  __attribute__((packed));
}
__attribute__((packed));

struct ulong_octets TX_Freq_Calib;
struct ulong_octets TX_Freq[10];
byte TX_dBm[10];            // Power for each band - Min = 0 dBm, Max = 43 dBm, steps 0,3,7,10,13,17,20,23,27,30,33,37,40,43
byte TX_Flag[10];
PROGMEM const unsigned long fCLK = 180000000;  // Enter clock frequency

//_ WSPR Stuff ________________________________________________________________________________________
PROGMEM const uint8_t SyncVec[162] = {
  1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
  1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0,
  0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0
};

// configure variables
int txPin  = 13;                    // TX (HIGH on transmit)
char sz[3];                         // Number of satellites

int seconds = 0, minute = 0, hour = 0;  // Internal clock
#define TIME_SYNC_PERIOD 5 * 60         // Seconds
unsigned int delay_time_sync = TIME_SYNC_PERIOD; // Number of seconds from last time sync
int sats;

int mSecTimer2;
unsigned int OffsetFreq[4];
#define dds_data_pin 6             //DDS DATA PIN
#define dds_clock_pin 8            //DDS CLOCK PIN
#define dds_load_pin 7             //DDS LOAD PIN

byte count = 0;
unsigned long FreqWord, TempWord, TempFreq;
unsigned long TenMHz = 10000000;

char buf[10];
volatile byte bb, i, j, ii, timeslot;
byte symbol[162];
byte c[11];                         // encoded message
byte sym[170];                      // symbol table 162
byte symt[170];                     // symbol table temp
byte RXflag = 1;                    // GPS/DCF receiver control 0 = disable, 1 = enable
byte RMCflag = 0;                   // RMC actual data flag
int  MsgLength;
unsigned long n1;                   // encoded callsign
unsigned long m1;                   // encodes locator

/******************************************************************
   T I M E R 1  I R Q   S E C O N D  C O U N T E R

    Clock - interrupt routine used as master timekeeper
    Timer1 Overflow Interrupt Vector, called every second
 ******************************************************************/
ISR(TIMER1_COMPA_vect) {
  //Clock
  seconds++ ;
  if (seconds >= 60) {
    minute++ ;
    seconds = 0 ;
  }
  if (minute >= 60) {
    hour++;
    minute = 0 ;
  }
  if (hour >= 24) {
    hour = 0 ;
  }
  
  //Start transmission
  if ((minute % 2 == 0) && (seconds == 0))
  {
    setfreq();
    transmit_start();
  }

  //Encode everything 2 seconds before to start
  if (((minute + 1) % 2 == 0) && (seconds == 58))
  {
    timeslot = ((minute + 1) % 20) / 2;
    // Begin WSPR message calculation
    encode_call();
    encode_locator();
    encode_conv();
    interleave_sync();
  } 
  
  //Display time in LCD
  displaytime();
  
  //Manage last sync period of clock
  if (delay_time_sync < TIME_SYNC_PERIOD) delay_time_sync++;
  if (ntp_query_timeout < 255) ntp_query_timeout++;
}


/******************************************************************
   T I M E R 2   I R Q   m S E C O N D   C O U N T E R

    Timer2 Overflow Interrupt Vector, called every mSec to increment
    the mSecTimer2 used for WSPR transmit timing.
 ******************************************************************/
ISR(TIMER2_COMPA_vect) {
  mSecTimer2++;
  if (mSecTimer2 > 681) {
    mSecTimer2 = 0;
    if (bb < 3) {                   // Begin 2 second delay - actually 0.682mSec * 3
      TempWord = FreqWord;
      TransmitSymbol();
      bb++;
    }
    else
    {
      if (count < 162)                // Begin 162 WSPR symbol transmission
      {
        TempWord = FreqWord + OffsetFreq[sym[count]];
        TransmitSymbol();
        count++;                    // Increments the interrupt counter
      }
      else
      {
        transmit_stop();
      }
    }
  }
}


/******************************************************************
   S E T U P
 ******************************************************************/
void setup()
{
  //Set up Timer2 to fire every mSec (WSPR timer)
  TIMSK2 = 0;        //Disable timer during setup
  TCCR2A = 2;        //CTC mode
  TCCR2B = 4;        //Timer Prescaler set to 64
  OCR2A = 247;       // Timer set for 1 mSec with correction factor

  //Set up Timer1A to fire every second (master clock)
  TCCR1B = 0;        //Disable timer during setup
  TIMSK1 = 2;        //Timer1 Interrupt enable
  TCCR1A = 0;        //Normal port operation, Wave Gen Mode normal
  TCCR1B = 12;       //Timer prescaler to 256 - CTC mode
  OCR1A = 62377;     //Timer set for 1000 mSec using correction factor
  // 62500 is nominal for 1000 mSec. Decrease variable to increase clock speed

  //Load offset values
  OffsetFreq[0] = 0;
  OffsetFreq[1] = 1.43 * pow(2, 32) / fCLK;
  OffsetFreq[2] = 2.93 * pow(2, 32) / fCLK;
  OffsetFreq[3] = 4.39 * pow(2, 32) / fCLK;

  // Set up TX pin to output
  pinMode(txPin, OUTPUT);

  // Set Serial to 19200 baud
  Serial.begin(115200);
  Serial.println(F("uBeacon"));
  Serial.println(F("(C) 2016 IU3AGC - Daniele De Marchi"));

  // Set GPS Serial to 9600 baud
  pinMode(GPS_RXPin, INPUT);
  pinMode(GPS_TXPin, OUTPUT);
  GPSSerial.begin(9600);

  // Set up DDS
  pinMode (dds_data_pin, OUTPUT);   // sets pin as OUPUT
  pinMode (dds_clock_pin, OUTPUT);  // sets pin as OUTPUT
  pinMode (dds_load_pin, OUTPUT);   // sets pin as OUTPUT

  // set up the LCD for 16 columns and 2 rows
  lcd.begin(16, 2);
  // Turn on LCD
  lcd.display();

  // turn off transmitter
  TempWord = 0;
  TransmitSymbol();
  setfreq();


  // Display "IDLE" on LCD
  lcd.setCursor(9, 1);
  lcd.print(F("IDLE   "));

  //Set some default values at first boot
  if (EEPROM[EE_MAGIC] != 0x86)
  {
    randomSeed(analogRead(0));
    EEPROM[EE_MAC + 0] = random(255); EEPROM[EE_MAC + 1] = random(255); EEPROM[EE_MAC + 2] = random(255);
    EEPROM[EE_MAC + 3] = random(255); EEPROM[EE_MAC + 4] = random(255); EEPROM[EE_MAC + 5] = random(255);
    EEPROM[EE_DHCP] = 0;//Set Static IP default 192.168.0.250
    EEPROM[EE_IP] = 192; EEPROM[EE_IP + 1] = 168; EEPROM[EE_IP + 2] = 0; EEPROM[EE_IP + 3] = 186;
    EEPROM[EE_NETMASK] = 255; EEPROM[EE_NETMASK + 1] = 255; EEPROM[EE_NETMASK + 2] = 255; EEPROM[EE_NETMASK + 3] = 0;
    EEPROM[EE_GW] = 192; EEPROM[EE_GW + 1] = 168; EEPROM[EE_GW + 2] = 0; EEPROM[EE_GW + 3] = 1;
    EEPROM[EE_DNS] = 192; EEPROM[EE_DNS + 1] = 168; EEPROM[EE_DNS + 2] = 0;  EEPROM[EE_DNS + 3] = 1;
    EEPROM[EE_NTP] = 94;  EEPROM[EE_NTP + 1] = 177; EEPROM[EE_NTP + 2] = 172;EEPROM[EE_NTP + 3] = 69;

    TX_Freq[0].value = 7040100;
    for(int i = 0; i < 10; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        EEPROM.update(EE_FREQ + (i * 4 + j), TX_Freq[0].byte[j]);
      }
      EEPROM.update(EE_POWER + i, 0);
      EEPROM.update(EE_TX_FLAG + i, 0);
    }

    EEPROM.update(EE_MAGIC, 0x86);
  }

  // Load Data from EEPROM
  //Load Call
  memset(call, 0, 7);
  for (int i = 0; i < 6; i++) call[i] = EEPROM[EE_CALL + i];
  Serial.print( F("Call: ") ); Serial.println(call);

  //Load Locator
  memset(locator, 0, 5);
  for (int i = 0; i < 4; i++) locator[i] = EEPROM[EE_LOCATOR + i];
  Serial.print( F("Locator:") ); Serial.println(locator);

  //Load CalFactor
  for (int i = 0; i < 4; i++)
  {
    TX_Freq_Calib.byte[i] = EEPROM[EE_FR_CALIB + i];
  }
  Serial.print( F("TX_Freq_Calib: ") ); Serial.println(TX_Freq_Calib.value);

  //Load TX_Freq, TX_dBm and TX_Flag
  for (int i = 0; i < 10; i++)
  {
    //Load Frequencies
    for (int j = 0; j < 4; j++)
    {
      TX_Freq[i].byte[j] = EEPROM[EE_FREQ + (i * 4 + j)];
    }

    TX_dBm[i]  = EEPROM[EE_POWER   + i];
    TX_Flag[i] = EEPROM[EE_TX_FLAG + i];

    Serial.print(i); Serial.print(F(" TX_Freq: ")); Serial.print( TX_Freq[i].value);
    Serial.print(F(" Hz - ")); Serial.print(TX_dBm[i]); Serial.print(F(" dBm - ")); Serial.println(TX_Flag[i]);
  }

  //Setup Ethernet card
  static byte myMAC[6];

  //Load MAC Address from EEPROM
  for (int i = 0; i < 5; i++)
  myMAC[i] = EEPROM.read(EE_MAC + i);
  
  //Ethernet Bootup
  uint8_t rev = ether.begin(sizeof Ethernet::buffer, myMAC);
  Serial.print( F("\nENC28J60 Revision ") );
  Serial.println( rev, DEC );
  if ( rev == 0)
    Serial.println( F( "Failed to access Ethernet controller" ) );
          
  if (EEPROM.read(EE_DHCP) == 1)
  {
    Serial.println( F( "Setting up DHCP" ));
    if (!ether.dhcpSetup())
    {
      Serial.println( F( "DHCP failed" ));
    }
  } else {
    Serial.println( F( "Setting up IP Address" ));
    static byte myIP[4];
    static byte myGW[4];
    static byte myDNS[4];
    static byte mySubNet[4];

    //Load four octets from EEPROM
    for (int i = 0; i < 4; i++)
    {
      myIP[i]     = EEPROM.read(EE_IP      + i);
      mySubNet[i] = EEPROM.read(EE_NETMASK + i);
      myDNS[i]    = EEPROM.read(EE_DNS     + i);
      myGW[i]     = EEPROM.read(EE_GW      + i);
      ntp_server[i] = EEPROM.read(EE_NTP + i);
    }
    ether.staticSetup(myIP, myGW, myDNS, mySubNet);
  }

  for (int i = 0; i < 4; i++) ntp_server[i] = EEPROM.read(EE_NTP + i);

  Serial.print(F("My MAC: "));
  char str_mac[22];
  sprintf(str_mac, "%02x:%02x:%02x:%02x:%02x:%02x\n", myMAC[0], myMAC[1], myMAC[2], myMAC[3], myMAC[4], myMAC[5]);
  Serial.print(str_mac);
  ether.printIp(F("My IP: "), ether.myip);
  ether.printIp(F("Netmask: "), ether.netmask);
  ether.printIp(F("GW IP: "), ether.gwip);
  ether.printIp(F("DNS IP: "), ether.dnsip);
  ether.printIp(F("NTP IP: "), ntp_server);

  lcd.setCursor(0, 0);
  //lcd.print(F("IP:"));
  sprintf(str_mac, "%03d.%03d.%03d.%03d", ether.myip[0], ether.myip[1], ether.myip[2], ether.myip[3]);
  lcd.print(str_mac);
  
  ntp_querySent = false;
  delay_time_sync = TIME_SYNC_PERIOD - 5;
  RXflag = 1;
}

/******************************************************************
      L O O P
 ******************************************************************/
void loop()
{

  ether.packetLoop(ether.packetReceive());

  if ((delay_time_sync >= TIME_SYNC_PERIOD) && (ntp_querySent == false)) {
    ether.ntpRequest(ntp_server, ntp_tcpPort);
    ntp_query_timeout = 0;
    ntp_querySent = true;
    Serial.write("NTP Send Query\n");
  }

  if ((ntp_query_timeout > 10) && (ntp_querySent == true)) {
    ntp_querySent = false;
    Serial.write("NTP Receive Timeout\n");
  }

  if (ntp_querySent && ether.ntpProcessAnswer(&ntp_timeStamp, ntp_tcpPort)) {
    Serial.write("NTP Receive Response\n");
    ntp_querySent = false;
    seconds = ntp_timeStamp % 60;
    ntp_timeStamp -= seconds;
    ntp_timeStamp /= 60;
    minute = ntp_timeStamp % 60;
    ntp_timeStamp -= minute;
    ntp_timeStamp /= 60;

    hour = ntp_timeStamp % 24;
    delay_time_sync = 0;
  }

  if (RXflag == 1) {

    while (GPSSerial.available() > 0)
    {
      char c;
      c = GPSSerial.read();
      //Serial.write(c);
      GPS.process(c);
    }

    if (GPS.isValid())
    {
      seconds = GPS.getSecond();
      minute = GPS.getMinute();
      hour = GPS.getHour();
      sats = GPS.getNumSatellites();
      delay_time_sync = 0;
    }
  }
  http_handler();
}

/******************************************************************
     WEB SERVER
 ******************************************************************/
#define PAYLOAD_SIZE 50
const char PROGMEM html_header_0[] = "HTTP/1.0 200 OK\r\ncontent-type: text/html; charset=utf-8\r\ncontent-length: ";
const char PROGMEM html_header_1[] = "\r\ncontent-encoding: gzip\r\nPragma: no-cache\r\n\r\n";
PROGMEM const char html_page[] = {

};

void http_handler()
{
  uint16_t len = ether.packetReceive();
  uint16_t pos = ether.packetLoop(len);

  if (pos) {

    Serial.println(F("---------- NEW PACKET ----------"));
    Serial.println((char *)Ethernet::buffer + pos);
    Serial.println(F("--------------------------------"));
    Serial.println();

    //Call Loc e TX Calibration configuration
    bool refershPage = 0;
    if (strncmp("GET /set", (char *)Ethernet::buffer + pos, 8) == 0) {
      static char *delimiters = PSTR("?=& ");
      char *token = token = strtok_P((char *)Ethernet::buffer + pos, delimiters);
      while (token != NULL)
      {
        if (strcmp_P(token, PSTR("call")) == 0)
        {
          //Serial.print("Save Call:");
          token = strtok_P(NULL, delimiters);
          memset(call, 0x00, 7);
          memcpy(call, token, MIN(strlen(token), 6));
          for (int i = 0; i < 6; i++) EEPROM.update(EE_CALL + i, call[i]);
          refershPage = 1;
          //Serial.print("Len:");Serial.println(MIN(strlen(token), 6));
          //Serial.println(token);
        }

        if (strcmp_P(token, PSTR("locator")) == 0)
        {
          //Serial.print("Save Locator: ");
          token = strtok_P(NULL, delimiters);
          memset(locator, 0x00, 5);
          memcpy(locator, token, MIN(strlen(token), 4));
          for (int i = 0; i < 4; i++) EEPROM.update(EE_LOCATOR + i, locator[i]);
          refershPage = 1;
          //Serial.print("Len:");Serial.println(MIN(strlen(token), 4));
          //Serial.println(token);
        }

        if (strcmp_P(token, PSTR("cal")) == 0)
        {
          //Serial.print("Save TX_Freq_Calib: ");
          token = strtok_P(NULL, delimiters);

          String inString = "";
          while (isDigit(*token)) {
            // convert the incoming byte to a char
            // and add it to the string:
            inString += (char) * token;
            token++;
          }

          TX_Freq_Calib.value = inString.toInt();
          for (int i = 0; i < 4; i++) EEPROM.update(EE_FR_CALIB + i, TX_Freq_Calib.byte[i]);
          refershPage = 1;
          //Serial.println(TX_Freq_Calib.value);
        }

        token = strtok_P(NULL, PSTR("?=& "));
      }
    }

    if (strncmp("GET /freq", (char *)Ethernet::buffer + pos, 8) == 0) {
      static char *delimiters = PSTR("?=& ");
      char *token = token = strtok_P((char *)Ethernet::buffer + pos, delimiters);

      bool enable = false;
      unsigned char freq_ptr = 0x00;
      ulong_octets freq_value;
      unsigned char dBm = 0x00;
      freq_value.value = 0;
      while (token != NULL)
      {
        if (strcmp_P(token, PSTR("ptr")) == 0)
        {
          //Serial.print("Save TX_Freq_Calib: ");
          token = strtok_P(NULL, delimiters);
          freq_ptr = atoi(token);
        }
        
        if (strcmp_P(token, PSTR("en")) == 0)
        {
          //Serial.print("Save TX_Freq_Calib: ");
          token = strtok_P(NULL, delimiters);
          enable = true;
        }
        
        if (strcmp_P(token, PSTR("fr")) == 0)
        {
          //Serial.print("Save TX_Freq_Calib: ");
          token = strtok_P(NULL, delimiters);
          freq_value.value = atol(token);                    
        }
        
        if (strcmp_P(token, PSTR("dB")) == 0)
        {
          //Serial.print("Save TX_Freq_Calib: ");
          token = strtok_P(NULL, delimiters);
          dBm = atoi(token);
        }
        
        token = strtok_P(NULL, PSTR("?=& "));
        if (token==NULL)
        {
          //Save values values in EEPROM
          for (int j = 0; j < 4; j++)
          {
            EEPROM.update(EE_FREQ + (freq_ptr * 4 + j), freq_value.byte[j]);
          }
          EEPROM.update(EE_POWER + freq_ptr, dBm);
          EEPROM.update(EE_TX_FLAG + freq_ptr, enable);

          if(freq_ptr == timeslot) transmit_stop();
          
          TX_Freq[freq_ptr].value = freq_value.value;
          TX_dBm[freq_ptr] = dBm;
          TX_Flag[freq_ptr] = enable;

          refershPage = 1;
        }
      }
    }    

    //Network configuration
    if (strncmp("GET /net", (char *)Ethernet::buffer + pos, 8) == 0) {
      char *token = token = strtok_P((char *)Ethernet::buffer + pos, PSTR("?=& "));
      unsigned char dhcp_flag = false;
      refershPage = 1;
      while (token != NULL)
      {
        static char *delimiters = PSTR("?=&. ");

        if (strcmp_P(token, PSTR("dhcp")) == 0)
        {
          //Serial.print("Save DHCP flag:");
          token = strtok_P(NULL, delimiters);
          //Serial.print(token);
          //Serial.println("");
          dhcp_flag = true;
        }

        if (strcmp_P(token, PSTR("ip")) == 0)
        {
          //Serial.print("Save IP:");
          token = strtok_P(NULL, delimiters); //octect= AAA
          EEPROM.update(EE_IP + 0, atoi(token));
          token = strtok_P(NULL, delimiters); //octect= BBB
          EEPROM.update(EE_IP + 1, atoi(token));
          token = strtok_P(NULL, delimiters); //octect= CCC
          EEPROM.update(EE_IP + 2, atoi(token));
          token = strtok_P(NULL, delimiters); //octect= DDD
          EEPROM.update(EE_IP + 3, atoi(token));
          //Serial.print(EEPROM[EE_IP + 0]); Serial.print("."); Serial.print(EEPROM[EE_IP + 1]); Serial.print("."); Serial.print(EEPROM[EE_IP + 2]); Serial.print("."); Serial.println(EEPROM[EE_IP + 3]);

        }

        if (strcmp_P(token, PSTR("mask")) == 0)
        {
          //Serial.print("Save NetMask:");
          token = strtok_P(NULL, delimiters); //token= AAA
          EEPROM.update(EE_NETMASK + 0, atoi(token));
          token = strtok_P(NULL, delimiters); //token= BBB
          EEPROM.update(EE_NETMASK + 1, atoi(token));
          token = strtok_P(NULL, delimiters); //token= CCC
          EEPROM.update(EE_NETMASK + 2, atoi(token));
          token = strtok_P(NULL, delimiters); //token= DDD
          EEPROM.update(EE_NETMASK + 3, atoi(token));
          //Serial.print(EEPROM[EE_NETMASK + 0]); Serial.print("."); Serial.print(EEPROM[EE_NETMASK + 1]); Serial.print("."); Serial.print(EEPROM[EE_NETMASK + 2]); Serial.print("."); Serial.println(EEPROM[EE_NETMASK + 3]);
        }

        if (strcmp_P(token, PSTR("gw")) == 0)
        {
          //Serial.print("Save Default Gateway:");
          token = strtok_P(NULL, delimiters); //token= AAA
          EEPROM.update(EE_GW + 0, atoi(token));
          token = strtok_P(NULL, delimiters); //token= BBB
          EEPROM.update(EE_GW + 1, atoi(token));
          token = strtok_P(NULL, delimiters); //token= CCC
          EEPROM.update(EE_GW + 2, atoi(token));
          token = strtok_P(NULL, delimiters); //token= DDD
          EEPROM.update(EE_GW + 3, atoi(token));
          //Serial.print(EEPROM[EE_GW + 0]); Serial.print("."); Serial.print(EEPROM[EE_GW + 1]); Serial.print("."); Serial.print(EEPROM[EE_GW + 2]); Serial.print("."); Serial.println(EEPROM[EE_GW + 3]);
        }

        if (strcmp_P(token, PSTR("dns")) == 0)
        {
          //Serial.print("Save Default Gateway:");
          token = strtok_P(NULL, delimiters); //token= AAA
          EEPROM.update(EE_DNS + 0, atoi(token));
          token = strtok_P(NULL, delimiters); //token= BBB
          EEPROM.update(EE_DNS + 1, atoi(token));
          token = strtok_P(NULL, delimiters); //token= CCC
          EEPROM.update(EE_DNS + 2, atoi(token));
          token = strtok_P(NULL, delimiters); //token= DDD
          EEPROM.update(EE_DNS + 3, atoi(token));
          //Serial.print(EEPROM[EE_DNS + 0]); Serial.print("."); Serial.print(EEPROM[EE_DNS + 1]); Serial.print("."); Serial.print(EEPROM[EE_DNS + 2]); Serial.print("."); Serial.println(EEPROM[EE_DNS + 3]);
        }

        if (strcmp_P(token, PSTR("ntp")) == 0)
        {
          //Serial.print("Save NTP Server:");
          token = strtok_P(NULL, delimiters); //token= AAA
          EEPROM.update(EE_NTP + 0, atoi(token));
          token = strtok_P(NULL, delimiters); //token= BBB
          EEPROM.update(EE_NTP + 1, atoi(token));
          token = strtok_P(NULL, delimiters); //token= CCC
          EEPROM.update(EE_NTP + 2, atoi(token));
          token = strtok_P(NULL, delimiters); //token= DDD
          EEPROM.update(EE_NTP + 3, atoi(token));
          //Serial.print(EEPROM[EE_NTP + 0]); Serial.print("."); Serial.print(EEPROM[EE_NTP + 1]); Serial.print("."); Serial.print(EEPROM[EE_NTP + 2]); Serial.print("."); Serial.println(EEPROM[EE_NTP + 3]);
        }

        token = strtok_P(NULL, delimiters);

        //Check end of parameters parse
        //and salve the presence of some checkboxes values
        if (token == NULL) {
          if (dhcp_flag == false)
          {
            //Serial.println("Save DHCP: OFF");
            EEPROM.update(EE_DHCP, 0);
          }
          else
          {
            //Serial.println("Save DHCP: ON");
            EEPROM.update(EE_DHCP, 1);
          }
        }
      }
    }

    if ((strncmp("GET / ", (char *)Ethernet::buffer + pos, 6) == 0) ||
        (refershPage == 1)) {
      ether.httpServerReplyAck();
      uint16_t http_buf_size = sizeof(html_page);
      uint16_t http_buf_index = 0;
      uint16_t packet_count = 0;

      //Serial.print("Size of html is"); Serial.println(sizeof(html_page));

      memcpy_P(ether.tcpOffset(), html_header_0, strlen_P(html_header_0));
      ether.httpServerReply_with_flags(strlen_P(html_header_0), TCP_FLAGS_ACK_V);
      //Serial.print("Len"); Serial.println(strlen_P(html_header_0));

      char str_content_lenght[10];
      sprintf(str_content_lenght, "%d", http_buf_size);
      memcpy(ether.tcpOffset(), str_content_lenght, strlen(str_content_lenght));
      ether.httpServerReply_with_flags(strlen(str_content_lenght), TCP_FLAGS_ACK_V);
      //Serial.print("Len"); Serial.println(strlen(str_content_lenght));

      memcpy_P(ether.tcpOffset(), html_header_1, strlen_P(html_header_1));
      ether.httpServerReply_with_flags(strlen_P(html_header_1), TCP_FLAGS_ACK_V);

      while (1) {
        // is the last packet?
        if (http_buf_size - http_buf_index < PAYLOAD_SIZE) {
          uint16_t packet_size = http_buf_size - http_buf_index;
          memcpy_P(ether.tcpOffset(), html_page + http_buf_index, packet_size);
          ether.httpServerReply_with_flags(packet_size, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V);
          break;
        } else {
          memcpy_P(ether.tcpOffset(), html_page + http_buf_index, PAYLOAD_SIZE);
          ether.httpServerReply_with_flags(PAYLOAD_SIZE, TCP_FLAGS_ACK_V);
          http_buf_index += PAYLOAD_SIZE;
          packet_count += 1;
        }
      }
    }

    if (strncmp("GET /json", (char *)Ethernet::buffer + pos, 9) == 0) {
      Serial.print("Send JSON");
      ether.httpServerReplyAck();
      char helpbuf[16];
      http_send_flash(F("{"));

      http_send_flash(F("\n\"tx\":\""));
      sprintf_P(helpbuf, PSTR("%d"), (RXflag==1)?0:1);
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"act_sat\":\""));
      sprintf_P(helpbuf, PSTR("%0d"), sats);
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"act_freq\":\""));
      sprintf(helpbuf, "%lu", TX_Freq[timeslot]);
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"act_dB\":\""));
      sprintf(helpbuf, "%02d", TX_dBm[timeslot]);
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"act_time\":\""));
      sprintf_P(helpbuf, PSTR("%02d:%02d:%02d"), hour, minute, seconds);
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"call\":\""));
      http_send_strn(call, 6);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"locator\":\""));
      http_send_strn(locator, 4);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"calfactor\":\""));
      sprintf_P(helpbuf, PSTR("%04d"), TX_Freq_Calib);
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      uint8_t i;
      http_send_flash(F(",\n\"band\":["));
      for (i = 0; i < 10; i++) {
        if (i != 0) http_send_flash(F(","));

        http_send_flash(F("\n {\"freq\":\""));
        sprintf_P(helpbuf, PSTR("%lu"), TX_Freq[i]);
        http_send_str(helpbuf);
        http_send_flash(F("\""));
        
        http_send_flash(F(", \"dB\":\""));
        sprintf_P(helpbuf, PSTR("%d"), TX_dBm[i]);
        http_send_str(helpbuf);
        http_send_flash(F("\""));
        
        http_send_flash(F(", \"tx_flag\":"));
        if (TX_Flag[i])
          http_send_flash(F("true"));
        else
          http_send_flash(F("false"));
        http_send_flash(F("}"));
      }
      http_send_flash(F("\n]"));

      http_send_flash(F(",\n\"dhcp\":"));
      sprintf_P(helpbuf, PSTR("%d"), EEPROM.read(EE_DHCP));
      http_send_str(helpbuf);
      //http_send_flash(F("\""));

      http_send_flash(F(",\n\"ip\":\""));
      sprintf_P(helpbuf, PSTR("%03d.%03d.%03d.%03d"), EEPROM.read(EE_IP + 0), EEPROM.read(EE_IP + 1), EEPROM.read(EE_IP + 2), EEPROM.read(EE_IP + 3));
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"netmask\":\""));
      sprintf_P(helpbuf, PSTR("%03d.%03d.%03d.%03d"), EEPROM.read(EE_NETMASK + 0), EEPROM.read(EE_NETMASK + 1), EEPROM.read(EE_NETMASK + 2), EEPROM.read(EE_NETMASK + 3));
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"gw\":\""));
      sprintf_P(helpbuf, PSTR("%03d.%03d.%03d.%03d"), EEPROM.read(EE_GW + 0), EEPROM.read(EE_GW + 1), EEPROM.read(EE_GW + 2), EEPROM.read(EE_GW + 3));
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"dns\":\""));
      sprintf_P(helpbuf, PSTR("%03d.%03d.%03d.%03d"), EEPROM.read(EE_DNS + 0), EEPROM.read(EE_DNS + 1), EEPROM.read(EE_DNS + 2), EEPROM.read(EE_DNS + 3));
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F(",\n\"ntp\":\""));
      sprintf_P(helpbuf, PSTR("%03d.%03d.%03d.%03d"), EEPROM.read(EE_NTP + 0), EEPROM.read(EE_NTP + 1), EEPROM.read(EE_NTP + 2), EEPROM.read(EE_NTP + 3));
      http_send_str(helpbuf);
      http_send_flash(F("\""));

      http_send_flash(F("\n}"));
      memcpy_P(ether.tcpOffset(), PSTR(" "), 1);
      ether.httpServerReply_with_flags(1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V);
    }

    //Send null buffer if some other request
    if (strncmp("GET ", (char *)Ethernet::buffer + pos, 4) == 0) {
            ether.httpServerReplyAck();
            memcpy_P(ether.tcpOffset(), PSTR("HTTP/1.1 404 Not Found\r\n"), 24);
            ether.httpServerReply_with_flags(22, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V);
    }
  }
}

void http_send_flash(const __FlashStringHelper* str)
{
  volatile int len = strlen_P((const char*) str);
  memcpy_P(ether.tcpOffset(), str, len);
  ether.httpServerReply_with_flags(len, TCP_FLAGS_ACK_V);
}

void http_send_str(char* str)
{
  volatile int len = strlen((const char*) str);
  memcpy(ether.tcpOffset(), str, len);
  ether.httpServerReply_with_flags(len, TCP_FLAGS_ACK_V);
}

void http_send_strn(char* str, unsigned char len)
{
  memcpy(ether.tcpOffset(), str, len);
  ether.httpServerReply_with_flags(len, TCP_FLAGS_ACK_V);
}


/******************************************************************
     C O D E R I N G
 ******************************************************************/
void encode()
{
  encode_call();
  encode_locator();
  encode_conv();
  interleave_sync();
};

//******************************************************************
// normalize characters 0..9 A..Z Space in order 0..36
char chr_normf(char bc )
{
  char cc = 36;
  if (bc >= '0' && bc <= '9') cc = bc - '0';
  if (bc >= 'A' && bc <= 'Z') cc = bc - 'A' + 10;
  if (bc >= 'a' && bc <= 'z') cc = bc - 'a' + 10;
  if (bc == ' ' ) cc = 36;

  return (cc);
}

//******************************************************************
void encode_call()
{
  unsigned long t1;

  // coding of callsign
  if (chr_normf(call[2]) > 9)
  {
    call[5] = call[4];
    call[4] = call[3];
    call[3] = call[2];
    call[2] = call[1];
    call[1] = call[0];
    call[0] = ' ';
  }

  n1 = chr_normf(call[0]);
  n1 = n1 * 36 + chr_normf(call[1]);
  n1 = n1 * 10 + chr_normf(call[2]);
  n1 = n1 * 27 + chr_normf(call[3]) - 10;
  n1 = n1 * 27 + chr_normf(call[4]) - 10;
  n1 = n1 * 27 + chr_normf(call[5]) - 10;

  // merge coded callsign into message array c[]
  t1 = n1;
  c[0] = t1 >> 20;
  t1 = n1;
  c[1] = t1 >> 12;
  t1 = n1;
  c[2] = t1 >> 4;
  t1 = n1;
  c[3] = t1 << 4;
}

//******************************************************************
void encode_locator()
{
  unsigned long t1;
  // coding of locator
  m1 = 179 - 10 * (chr_normf(locator[0]) - 10) - chr_normf(locator[2]);
  m1 = m1 * 180 + 10 * (chr_normf(locator[1]) - 10) + chr_normf(locator[3]);
  m1 = m1 * 128 + TX_dBm[timeslot] + 64;

  // merge coded locator and power into message array c[]
  t1 = m1;
  c[3] = c[3] + ( 0x0f & t1 >> 18);
  t1 = m1;
  c[4] = t1 >> 10;
  t1 = m1;
  c[5] = t1 >> 2;
  t1 = m1;
  c[6] = t1 << 6;
}

//******************************************************************
// convolutional encoding of message array c[] into a 162 bit stream
void encode_conv()
{
  int bc = 0;
  int cnt = 0;
  int cc;
  unsigned long sh1 = 0;

  cc = c[0];

  for (int i = 0; i < 81; i++) {
    if (i % 8 == 0 ) {
      cc = c[bc];
      bc++;
    }
    if (cc & 0x80) sh1 = sh1 | 1;

    symt[cnt++] = parity(sh1 & 0xF2D05351);
    symt[cnt++] = parity(sh1 & 0xE4613C47);

    cc = cc << 1;
    sh1 = sh1 << 1;
  }
}

//******************************************************************
byte parity(unsigned long li)
{
  byte po = 0;
  while (li != 0)
  {
    po++;
    li &= (li - 1);
  }
  return (po & 1);
}

//******************************************************************
// interleave reorder the 162 data bits and and merge table with the sync vector
void interleave_sync()
{
  int ii, ij, b2, bis, ip;
  ip = 0;

  for (ii = 0; ii <= 255; ii++) {

    bis = 1;
    ij = 0;

    for ( b2 = 0; b2 < 8 ; b2++) {
      if (ii & bis) ij = ij | (0x80 >> b2);
      bis = bis << 1;
    }

    if (ij < 162 ) {
      sym[ij] = pgm_read_byte_near(SyncVec + ij) + 2 * symt[ip];
      ip++;
    }

  }
}

/******************************************************************
    S E T F R E Q
    Determine time slot and load band frequency data. Display
    frequency and calculate frequency word for DDS
 ******************************************************************/
void setfreq()
{
  // Print frequency to the LCD
  lcd.setCursor(0, 0);

  ltoa(TX_Freq[timeslot].value, buf, 10);

  if (buf[7] == 0) {
    lcd.print(buf[0]);
    lcd.print(F(","));
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(F("."));
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(F(" KHz  "));
  }
  else {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(F(","));
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(F("."));
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(F(" KHz  "));
  }
  FreqWord = (TX_Freq[timeslot].value + (TX_Freq_Calib.value * (TX_Freq[timeslot].value / pow(10, 7)))) * pow(2, 32) / fCLK;
}


/******************************************************************
    T R A N S M I T
    Determine if it is time to transmit. If so, determine if it is
    time to transmit the WSPR message. If not turn to IDLE
 ******************************************************************/
void transmit_start()
{
  //if(delay_time_sync >= 255) return;

  if (TX_Flag[timeslot] == 1)
  { // Start WSPR transmit process
    RXflag = 0;                 // Disable GPS/DCF receiver
    lcd.setCursor(9, 1);
    lcd.print(F("WSPR TX"));
    digitalWrite(txPin, HIGH);  // External transmit control ON
    bb = 0;
    count = 0;                  // Start WSPR symbol transmit process
    TIMSK2 = 2;                 // Enable timer2 interrupt
  }
  else
  {
    void transmit_stop();
  }
}

void transmit_stop()
{
  // Turn off transmitter and idle
  TIMSK2 = 0;                 // Disable WSPR timer
  digitalWrite(txPin, LOW);   // External transmit control OFF
  TempWord = 0;               // Turn off transmitter
  TransmitSymbol();
  RXflag = 1;                 // Turn GPS/DCF receiver back on
  lcd.setCursor(9, 1);
  lcd.print(F("IDLE   "));
}

/******************************************************************
     T U N E    D D S
 ******************************************************************/
void TransmitSymbol()
{
  digitalWrite (dds_load_pin, LOW);       // take load pin low

  for (char i = 0; i < 32; i++)
  {
    if ((TempWord & 1) == 1)
      dds_writeOne();
    else
      dds_writeZero();
    TempWord = TempWord >> 1;
  }
  dds_write_byte(0x09);

  digitalWrite (dds_load_pin, HIGH);      // Take load pin high again
}

void dds_write_byte(unsigned char value)
{
  unsigned char i;

  for (i = 0; i < 8; i++)
  {
    if ((value & 1) == 1)
      dds_writeOne();
    else
      dds_writeZero();
    value = value >> 1;
  }
}

void dds_writeOne()
{
  digitalWrite(dds_clock_pin, LOW);
  digitalWrite(dds_data_pin, HIGH);
  digitalWrite(dds_clock_pin, HIGH);
  digitalWrite(dds_data_pin, LOW);
}

void dds_writeZero()
{
  digitalWrite(dds_clock_pin, LOW);
  digitalWrite(dds_data_pin, LOW);
  digitalWrite(dds_clock_pin, HIGH);
}

/******************************************************************
     D I S P L A Y   T I M E

     Displays Time and number of active satellites during Tx IDLE
 ******************************************************************/
void displaytime()
{
  lcd.setCursor(0, 1);
  lcd.display();
  if (hour < 10) lcd.print (F("0"));
  lcd.print (hour);
  lcd.print (F(":"));
  if (minute < 10) lcd.print (F("0"));
  lcd.print (minute);
  lcd.print (F(":"));
  if (seconds < 10) lcd.print (F("0"));
  lcd.print (seconds);
  lcd.print (F(" "));

  if (RXflag == 0) return;

  if (sats >= 0 && sats < 20) {
    lcd.setCursor(9, 1);
    lcd.print (F(" SAT:"));
    lcd.setCursor(14, 1);
    lcd.print (sats);
  }
}

/******************************************************************
     D I S P L A Y   F R E Q
 ******************************************************************/
void DisplayFreq()
{
  // Print frequency to the LCD
  lcd.setCursor(0, 0);

  ltoa(TempFreq, buf, 10);

  if (buf[7] == 0) {
    lcd.print(buf[0]);
    lcd.print(F(","));
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(F("."));
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(F(" KHz "));
  }
  else {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(F(","));
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(F("."));
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(F(" KHz "));
  }
}
