/* brew_control.ino */
#include "RCSwitch/RCSwitch.h"
#include "Adafruit_DHT.h"
#include "LiquidCrystal.h"
#include "application.h"

SYSTEM_MODE(MANUAL);
/* 
    uncomment to enable
*/
//#define SERIALDEBUG

/*
   Pins
*/
#define DHTPIN D6     // what pin we're connected to
#define CHANNEL1 A0
#define CHANNEL2 A1
#define TRANSMITPIN D7
#define RS D0
#define ENABLE D1 
#define DATA7 D5
#define DATA6 D4
#define DATA5 D3
#define DATA4 D2

/*
    switch codes
*/
#define SWITCH_ONE_ON     "000000010100010001010101"
#define SWITCH_ONE_OFF    "000000010100010001010100"
#define SWITCH_TWO_ON     "000000010101000001010101"
#define SWITCH_TWO_OFF    "000000010101000001010100"
#define SWITCH_THREE_ON   "000000010001010001010101"
#define SWITCH_THREE_OFF  "000000010001010001010100"
#define SWITCH_FOUR_ON    "000000010101010001010101"
#define SWITCH_FOUR_OFF   "000000010101010001010100"

/*
  Uncomment whatever type of DHT you're using!
*/
#define DHTTYPE DHT11		// DHT 11 
//#define DHTTYPE DHT22		// DHT 22 (AM2302)
//#define DHTTYPE DHT21		// DHT 21 (AM2301)
 
/* 
    DHT connections
        Connect pin 1 (on the left) of the sensor to +5V
        Connect pin 2 of the sensor to whatever your DHTPIN is
        Connect pin 4 (on the right) of the sensor to GROUND
        Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
*/

/*
    eeprom
*/
typedef struct
{
    double amb_humid;
    double amb_temp;
    double temp_ch1;
    double temp_ch2;
    double ch1_hl;
    double ch1_hh;
    double ch2_hl;
    double ch2_hh;
    double ch1_cl;
    double ch1_ch;
    double ch2_cl;
    double ch2_ch;
} data_t;


union eeprom_t
{
    data_t data;
    uint8_t bytes[sizeof(data_t)];
} ;

static char checksum = 0;

#define CHECKSUM_ADDRESS ((uint8_t)0u)
#define CACHE_ADDRESS    ((uint8_t)1u)

/*
   cloud vaiables
       readings
       hysteresis bands
*/
static double amb_humid = 0.0;
static double amb_temp = 0.0;
static double temp_ch1 = 0.0;
static double temp_ch2 = 0.0;
static double ch1_hl = 17 + 6;
static double ch1_hh = 19 + 6;
static double ch2_hl = 17 + 6;
static double ch2_hh = 19 + 6;
static double ch1_cl = 18 + 6;
static double ch1_ch = 20 + 6;
static double ch2_cl = 18 + 6;
static double ch2_ch = 20 + 6;
static eeprom_t cache;

/*
    objects
*/
TCPClient  client;
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(RS, ENABLE, DATA7, DATA6, DATA5, DATA4); // RW to GND
RCSwitch mySwitch;

/*
   assign a MAC address for grovestreams.
*/
const byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0xED};
static char mac_string[20];        //Don't Change. Set below from the above mac variable. The readable Mac is used by GS to determine which component the
                       // feeds are uploading into. It must match an existing GroveStreams component's ID
/*
    GroveStreams Settings
*/
const char gsApiKey[] = "80c18974-1ec3-3ab5-badf-3f344b2978fd";   //My Feed PUT API key with auto registration rights
const char gsComponentName[] = "BrewCupboardController"; 
const char gsDomain[] = "grovestreams.com";   //Don't change. The GroveStreams domain.
const char gsComponentTemplateId[] = "temp";  //Don't change. Tells GS what template to use when the feed initially arrives and a new component needs to be created.
                                        // The blueprint is expecting "temp".
/*
    GroveStreams Stream IDs. Stream IDs tell GroveStreams which component streams the values will be assigned to.
    Don't change these unless you edit your GroveStreams component definition and change the stream IDs to match these.
*/
static char gsStreamId1[] = "t1";   /**< Don't change. Temperature 1       */
static char gsStreamId2[] = "t2";   /**< Don't change. Temperature 2       */
static char gsStreamId3[] = "ta";   /**< Don't change. Ambient Temperature */
static char gsStreamId4[] = "ha";   /**< Don't change. Ambient Humidity    */
static char gsStreamId5[] = "s1";   /**< Don't change. Switch 4 status     */
static char gsStreamId6[] = "s2";   /**< Don't change. Switch 4 status     */
static char gsStreamId7[] = "s3";   /**< Don't change. Switch 4 status     */
static char gsStreamId8[] = "s4";   /**< Don't change. Switch 4 status     */
static char data[55];

/*
    Grovestreams update timings
*/
const uint32_t updateFrequency = ( 5 * 60000ul ); /**< Update frequency in milliseconds (86400000 milliseconds = 24 hours).3600000 = 1hour  */
static uint32_t lastSuccessfulUploadTime = 0u;           /**< Used to determine if samples need to be uploaded.                                    */
static bool first_time_conneceted = true;

/*
    Control timings
*/
const uint32_t controlFrequency = ( 1 * 60000ul );
static uint32_t lastcontroltime = 0u;

/*
    LCD update timings
*/
const uint32_t lcdFrequency = ( 10000ul );
static uint32_t lastlcdtime = 0u;

/*
    Sensor timings
*/
const uint32_t sensorFrequency = ( 2000ul );
static uint32_t lastsensortime = 0u;

/*
    switch data
*/
static char switch1_string[1] = {0};
static char switch2_string[1] = {0};
static char switch3_string[1] = {0};
static char switch4_string[1] = {0};
static bool switch1_on = false;
static bool switch2_on = false;
static bool switch3_on = false;
static bool switch4_on = false;

/*
    function prototypes
 */
void dht11( double* humid, double* temperature );
float TMP36_read( int channel );
int getLength( int someValue );
void sendData( int thisData );
void updateGroveStreams(char* data);
int set_ch1_hl( String value );
int set_ch1_hh( String value );
int set_ch2_hl( String value );
int set_ch2_hh( String value );
int set_ch1_cl( String value );
int set_ch1_ch( String value );
int set_ch2_cl( String value );
int set_ch2_ch( String value );
void eeprom_read( void );
void eeprom_write( void );
bool check_checksum ( void );
char calc_checksum ( eeprom_t memory );
void client_run( void );
void lcd_run( void );
void sensors_run( void );

/*********************************************************************************
    Main
*********************************************************************************/
/*
    setup() - initialise objects
 */
void setup()
{
#ifdef SERIALDEBUG
    Serial.begin(9600);
#endif
    mySwitch.enableTransmit(TRANSMITPIN);
	dht.begin();
    lcd.begin(16,2);
    sprintf(mac_string, "%02x:%02x:%02x:%02x:%02x:%02x\0", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    WiFi.on();
    WiFi.connect();
    Particle.connect();
    eeprom_read();
}

/* main loop run everything
 */
void loop()
{
    sensors_run();
    control_run();
    client_run();
    lcd_run();
    process_run();
}

/*********************************************************************************
    Sensors
*********************************************************************************/
/** Sensor scheduling
*/
void sensors_run( void )
{
    if ( time_to_update( lastsensortime, sensorFrequency ) )    
    {
        temp_ch1 = TMP36_read(CHANNEL1);
        temp_ch2 = TMP36_read(CHANNEL2);
        dht11( &amb_humid, &amb_temp );
        lastsensortime = millis();
    }
}

/** read TMP36 and convert to a temperature in degrees Celcius
 */
float TMP36_read( int channel )
{
    float result;
    int analogue = analogRead(channel);

    /* converting that reading to voltage, for 3.3v adc ref */
    float voltage = (float)analogue * 3.3/4095.0;

    /* 
        converting from 10 mv per degree with 500 mV offset
        to degrees ((voltage - 500mV) times 100)
    */
    result = (voltage - 0.5) * 100;  

#ifdef SERIALDEBUG
    /* print out the voltage */
    Serial.print(voltage); 
    Serial.println(" volts");
    /* now print out the temperature */
    Serial.print(result); 
    Serial.println(" degrees C");
#endif 
    
    return result; 
}

/** function to handle the dht sensor with debug output if enabled
 */ 
void dht11( double* humid, double* temperature)
{
    /*
        Reading temperature or humidity takes about 250 milliseconds!
        Sensor readings may also be up to 2 seconds 'old' (its a 
        very slow sensor)
    */
	*humid = dht.getHumidity();
    /* Read temperature as Celsius */
	*temperature = dht.getTempCelcius();
#ifdef SERIALDEBUG
    /* Read temperature as Farenheit */
	float f = dht.getTempFarenheit();
    /* Compute heat index */
	float hi = dht.getHeatIndex();
	float dp = dht.getDewPoint();
	float k = dht.getTempKelvin();

	Serial.print("Humid: "); 
	Serial.print(*humid);
	Serial.print("% - ");
	Serial.print("Temp: "); 
	Serial.print(*temperature);
	Serial.print("*C ");
	Serial.print(f);
	Serial.print("*F ");
	Serial.print(k);
	Serial.print("*K - ");
	Serial.print("DewP: ");
	Serial.print(dp);
	Serial.print("*C - ");
	Serial.print("HeatI: ");
	Serial.print(hi);
	Serial.println("*C");
	Serial.println(Time.timeStr());
#endif
}


/*********************************************************************************
    Control
*********************************************************************************/
void control_run( void )
{
    if ( time_to_update( lastcontroltime, controlFrequency ) )    
    {
        /* 
            Heating
        */
        if ( ( temp_ch1 * 100 )  < ( ch1_hl * 100 ) )
        {
            mySwitch.send(SWITCH_ONE_ON);
            sprintf(switch1_string, "%d", 1);
            switch1_on = true;
        }
        else if ( ( temp_ch1 * 100 ) > ( ch1_hh * 100 ))
        {
            mySwitch.send(SWITCH_ONE_OFF);
            sprintf(switch1_string, "%d", 0);
            switch1_on = false;
        }

        if ( ( temp_ch2 * 100 ) < ( ch2_hl * 100 ) )
        {
            mySwitch.send(SWITCH_TWO_ON);
            sprintf(switch2_string, "%d", 1);
            switch2_on = true;
        }
        else if ( ( temp_ch2 * 100 ) > ( ch2_hh * 100 ) )
        {
            mySwitch.send( SWITCH_TWO_OFF );
            sprintf(switch2_string, "%d", 0);
            switch2_on = false;
        }
        /*
            Cooling?
        */
        if ( ( temp_ch1 * 100 ) <  ( ch1_cl * 100 )  )
        {
            mySwitch.send(SWITCH_THREE_OFF) ;
            sprintf(switch3_string, "%d", 0);
            switch3_on = false;
        }
        else if ( ( temp_ch1 * 100 ) > ( ch1_ch * 100 ) )
        {
            mySwitch.send(SWITCH_THREE_ON);
            sprintf(switch3_string, "%d", 1);
            switch3_on = true;
        }

        if ( ( temp_ch2 * 100 ) < ( ch2_cl * 100 ) )
        {
            mySwitch.send(SWITCH_FOUR_OFF);
            sprintf(switch4_string, "%d", 0);
            switch4_on = false;
        }
        else if ( ( temp_ch2 * 100 ) > ( ch2_ch * 100 ) )
        {
            mySwitch.send(SWITCH_FOUR_ON);
            sprintf(switch4_string, "%d", 1);
            switch4_on = true;
        }
        lastcontroltime = millis();
    }
}


/*********************************************************************************
    Display
*********************************************************************************/
/** LCD scheduling
 */
void lcd_run( void )
{
   /*
       update LCD
    */
    static uint8_t count = 0;
    
    if ( time_to_update(lastlcdtime, lcdFrequency))
    {
        lcd.clear();
        switch (count)
        {
            case 0:
            {
                lcd.setCursor(0, 0);
                lcd.printf( "Temp 1: ");
                lcd.printf( "%2.2f", temp_ch1 );
                lcd.print((char)223); /* degrees character */
                lcd.printf( "C");
    
                lcd.setCursor(0, 1);
                lcd.printf( "Temp 2: ");
                lcd.printf( "%2.2f", temp_ch2 );
                lcd.print((char)223); /* degrees character */
                lcd.printf( "C");
                break;
            }
            case 1:
            {
                lcd.setCursor(0, 0);
                lcd.printf( "Amb Temp: ");
                lcd.printf( "%2.0f", amb_temp  );
                lcd.print((char)223); /* degrees character */
                lcd.printf( "C");
    
                lcd.setCursor(0, 1);
                lcd.printf( "Humidity: ");
                lcd.printf( "%2.0f", amb_humid );
                lcd.printf( "%%");
                break;
            }
            case 2:
            {
                lcd.setCursor(0, 0);
                lcd.printf( "Heating:");

                lcd.setCursor(0, 1);
                lcd.printf( "Ch1 ");
                if ( switch1_on )
                {
                    lcd.printf( "On  " );
                }
                else
                {
                    lcd.printf( "Off " );
                }
                lcd.printf( "Ch2 ");
                if ( switch2_on )
                {
                    lcd.printf( "On" );
                }
                else
                {
                    lcd.printf( "Off" );
                }
                break;
            }
            case 3:
            {
                lcd.setCursor(0, 0);
                lcd.printf( "Cooling:");

                lcd.setCursor(0, 1);
                lcd.printf( "Ch1 ");
                if ( switch3_on )
                {
                    lcd.printf( "On  " );
                }
                else
                {
                    lcd.printf( "Off " );
                }
                lcd.printf( "Ch2 ");
                if ( switch4_on )
                {
                    lcd.printf( "On" );
                }
                else
                {
                    lcd.printf( "Off" );
                }
                break;
            }
            default:
            {
                /* do nothing */
            }
        }
        lastlcdtime = millis();
        count++;
        if (count > 3)
        {
            count = 0;
        }
    }
}

/*********************************************************************************
    Internet
*********************************************************************************/
/** runs the connection to the internet.
 */
void client_run(void)
{
    if ( WiFi.ready() )
    {
        // if you're not connected, and ten seconds have passed since
        // your last connection, then connect again and send data:
        if( !client.connected() && time_to_update( lastSuccessfulUploadTime, updateFrequency ) ) 
        {
            /*
                Assemble the URL parameters which are seperated with the "&" character
                Example: &s1=25.68&s2=78.23
            */
            char ambient_humidity_string[10] = {0}; 
            char ambient_temperature_string[10] = {0};
            char temperature_channel1_string[10] = {0};
            char temperature_channel2_string[10] = {0};
    
            sprintf(ambient_humidity_string, "%4.0f", amb_humid);
            sprintf(ambient_temperature_string, "%4.0f", amb_temp);
            sprintf(temperature_channel1_string, "%4.2f", temp_ch1);
            sprintf(temperature_channel2_string, "%4.2f", temp_ch2);
    
            sprintf(data, "&%s=%s&%s=%s&%s=%s&%s=%s&%s=%s&%s=%s&%s=%s&%s=%s", 
                                          gsStreamId1, trim(temperature_channel1_string),
                                          gsStreamId2, trim(temperature_channel2_string),
                                          gsStreamId3, trim(ambient_temperature_string),
                                          gsStreamId4, trim(ambient_humidity_string),
                                          gsStreamId5, trim(switch1_string),
                                          gsStreamId6, trim(switch2_string),
                                          gsStreamId7, trim(switch3_string),
                                          gsStreamId8, trim(switch4_string)
                    );
            updateGroveStreams(data);
        }
    }
}

/** Starts the Connection and attempts to send data
 */
void updateGroveStreams(char* data)
{
    uint32_t connectAttemptTime = millis();
    
    if (client.connect(gsDomain, 80))
    {
        /*
            You may need to increase the size of urlBuf if any other char array sizes have increased
        */
        char urlBuf[200];
        sprintf(urlBuf, "PUT /api/feed?compTmplId=%s&compId=%s&compName=%s&api_key=%s%s HTTP/1.1",
               gsComponentTemplateId, mac_string, gsComponentName, gsApiKey, data);
        /*
            Uncomment the next three lines for debugging purposes
        */
#ifdef SERIALDEBUG
        Serial.println(urlBuf);    
        Serial.print(F("urlBuf length = "));
        Serial.println(strlen(urlBuf));
#endif
        /*
            Sens the PUT request.
        */
        client.println(urlBuf);  /* Send the url with temp readings in one println(..) to decrease the chance of dropped packets */
        client.print(F("Host: "));
        client.println();
        client.println(F("Connection: close"));
        client.println(F("Content-Type: application/json"));
        client.println();

        if (client.connected())
        {
            /*
                Wait for the response, consume and display
            */
            while(!client.available())
            {
                delay(1);
            }
 
            while(client.available())
            {
                char c = client.read();
#ifdef SERIALDEBUG
                Serial.print(c);
#endif
            }
            /*
                Client is now disconnected, stop it to cleannup.
            */
            client.stop();
            lastSuccessfulUploadTime = connectAttemptTime;
        }
 
    }
 
}

/** set lower threshold for channel 1 heating
 *  only called from particle 
 */ 
int set_ch1_hl( String value )
{
    int result = 0; 
    double setpoint = atof( value );
    if ( setpoint > ch1_hh )
    {
        result = -1;
        // set error?
    }
    else if ( setpoint > ch1_cl )
    {
        result = -2;
        //set error?
    }
    /* change it anyway */
    ch1_hl = setpoint;
    
    cache.data.ch1_hl = ch1_hl;
    eeprom_write();
    return result;
}

/** set upper threshold for channel 1 heating
 *  only called from particle 
 */ 
int set_ch1_hh( String value )
{
    int result = 0; 
    double setpoint = atof( value );
    if ( setpoint < ch1_hl )
    {
        result = -1;
        // set error?
    }
    else if ( setpoint < ch1_ch )
    {
        result = -2;
        //set error?
    }
    /* change it anyway */
    ch1_hh = setpoint;
    
    cache.data.ch1_hh = ch1_hh;
    eeprom_write();
    return result;
}

/** set lower threshold for channel 2 heating
 *  only called from particle 
 */ 
int set_ch2_hl( String value )
{
    int result = 0; 
    double setpoint = atof( value );
    if ( setpoint > ch2_hh )
    {
        result = -1;
        // set error?
    }
    else if ( setpoint > ch2_cl )
    {
        result = -2;
        //set error?
    }
    /* change it anyway */
    ch2_hl = setpoint;
    
    cache.data.ch2_hl = ch2_hl;
    eeprom_write();
    return result;
}

/** set upper threshold for channel 2 heating
 *  only called from particle 
 */ 
int set_ch2_hh( String value )
{
    int result = 0; 
    double setpoint = atof( value );
    if ( setpoint < ch2_hl )
    {
        result = -1;
        // set error?
    }
    else if ( setpoint < ch2_cl )
    {
        result = -2;
        //set error?
    }
    /* change it anyway */
    ch2_hh = setpoint;
    
    cache.data.ch2_hh = ch2_hh;
    eeprom_write();
    return result;
}

/** set lower threshold for channel 1 cooling
 *  only called from particle 
 */ 
int set_ch1_cl( String value )
{
    int result = 0; 
    double setpoint = atof( value );
    if ( setpoint > ch1_ch )
    {
        result = -1;
        // set error?
    }
    else if ( setpoint > ch1_hl )
    {
        result = -2;
        //set error?
    }
    /* change it anyway */
    ch1_cl = setpoint;
    
    cache.data.ch1_cl = ch1_cl;
    eeprom_write();
    return result;
}

/** set upper threshold for channel 1 cooling
 *  only called from particle 
 */ 
int set_ch1_ch( String value )
{
    int result = 0; 
    double setpoint = atof( value );
    if ( setpoint < ch1_cl )
    {
        result = -1;
        // set error?
    }
    else if ( setpoint < ch1_hh )
    {
        result = -2;
        //set error?
    }
    /* change it anyway */
    ch1_ch = setpoint;
    
    cache.data.ch1_ch = ch1_ch;
    eeprom_write();
    return result;
}

/** set lower threshold for channel 2 cooling
 *  only called from particle 
 */ 
int set_ch2_cl( String value )
{
    int result = 0; 
    double setpoint = atof( value );
    if ( setpoint > ch2_ch )
    {
        result = -1;
        // set error?
    }
    else if ( setpoint > ch2_hl )
    {
        result = -2;
        //set error?
    }
    /* change it anyway */
    ch2_cl = setpoint;
    
    cache.data.ch2_cl = ch2_cl;
    eeprom_write();
    return result;
}

/** set upper threshold for channel 2 cooling
 *  only called from particle 
 */ 
int set_ch2_ch( String value )
{
    int result = 0; 
    double setpoint = atof( value );
    if ( setpoint < ch2_cl )
    {
        result = -1;
        // set error?
    }
    else if ( setpoint < ch2_hh )
    {
        result = -2;
        //set error?
    }
    /* change it anyway */
    ch2_ch = setpoint;
     
    cache.data.ch2_ch = ch2_ch;
    eeprom_write();
    return result;
}

/*********************************************************************************
    Utilities
*********************************************************************************/
/**  housekeeping for the cloud
 */
void process_run( void )
{
    /*
        Update the internet
    */
    if(Particle.connected() == false)
    {
         Particle.connect();
    }
    else
    {
        Particle.process();
        if (first_time_conneceted)
        {
            Particle.function( "fch1_hl", set_ch1_hl );
            Particle.function( "fch1_hh", set_ch1_hh );
            Particle.function( "fch1_cl", set_ch1_cl );
            Particle.function( "fch1_ch", set_ch1_ch );
            Particle.function( "fch2_hl", set_ch2_hl );
            Particle.function( "fch2_hh", set_ch2_hh );
            Particle.function( "fch2_cl", set_ch2_cl );
            Particle.function( "fch2_ch", set_ch2_ch );
    
            Particle.variable("amb_humid", amb_humid );
            Particle.variable("amb_temp",  amb_temp  );
            Particle.variable("temp_ch1",  temp_ch1  );
            Particle.variable("temp_ch2",  temp_ch2  );

            Particle.variable("ch1_hl", ch1_hl );
            Particle.variable("ch1_hh", ch1_hh );
            Particle.variable("ch2_hl", ch2_hl );
            Particle.variable("ch2_hh", ch2_hh );
            Particle.variable("ch1_cl", ch1_cl );
            Particle.variable("ch1_ch", ch1_ch );
            Particle.variable("ch2_cl", ch2_cl );
            Particle.variable("ch2_ch", ch2_ch );
            first_time_conneceted = false;
        }
    }
}

/* check to see if it's time to update
 * handles wrap on millis() since this will always be on.
 */
bool time_to_update( uint32_t lasttime, uint32_t frequency )
{
    uint32_t timenow = millis();
    bool result = false;
    if ( lasttime > timenow )    
    {
        result = ((0xFFFFFFFFu - lasttime) + timenow) > frequency;
    }
    else
    {
        result = ( timenow - lasttime > frequency );    
    }
    return result;
}

/** trim
 *  function to trim leading and ending spaces
 */
char* trim(char* input)                                        
{
    uint16_t i,j;
    char *output=input;
    for (i = 0, j = 0; i<strlen(input); i++,j++)          
    {
        if (input[i]!=' ')                          
        {
            output[j]=input[i];
        }
        else
        {
            j--;
        }
    }
    output[j]=0;
    return output;
}

/** Read structure and checksum from eeprom
 */
void eeprom_read( void )
{
    //for (int i=0; i<sizeof(data_t); i++) {
    //    cache.bytes[i] = EEPROM.read(i);
    //}   
    EEPROM.get(CHECKSUM_ADDRESS, checksum);
    EEPROM.get(CACHE_ADDRESS, cache);
    if ( check_checksum() )
    {
        amb_humid = cache.data.amb_humid;
        amb_temp = cache.data.amb_temp;
        temp_ch1 = cache.data.temp_ch1;
        temp_ch2 = cache.data.temp_ch2;
        ch1_hl = cache.data.ch1_hl;
        ch1_hh = cache.data.ch1_hh;
        ch2_hl = cache.data.ch2_hl;  
        ch2_hh = cache.data.ch2_hh;
        ch1_cl = cache.data.ch1_cl;
        ch1_ch = cache.data.ch1_ch;
        ch2_cl = cache.data.ch2_cl;
        ch2_ch = cache.data.ch2_ch;
    }
    else
    {
        cache.data.amb_humid = amb_humid;
        cache.data.amb_temp = amb_temp;
        cache.data.temp_ch1 = temp_ch1;
        cache.data.temp_ch2 = temp_ch2;
        cache.data.ch1_hl = ch1_hl;
        cache.data.ch1_hh = ch1_hh;
        cache.data.ch2_hl = ch2_hl;
        cache.data.ch2_hh = ch2_hh;
        cache.data.ch1_cl = ch1_cl;
        cache.data.ch1_ch = ch1_ch;
        cache.data.ch2_cl = ch2_cl;
        cache.data.ch2_ch = ch2_ch;
        eeprom_write();
    }
}

/** write eeprom structure and checksum
 */
void eeprom_write ( void )
{
    checksum = calc_checksum( cache );
    //for (int i=0; i<sizeof(data_t); i++)
    //{
    //    EEPROM.write(i, cache.bytes[i]);
    //}
    EEPROM.put(CACHE_ADDRESS, cache);
    EEPROM.put(CHECKSUM_ADDRESS, checksum);
}

/** calculate eeprom checksum
 *     this needs updating to automatically handle eeprom size changes
 */
char calc_checksum ( eeprom_t memory )
{
    char result = 0;
    double temp = cache.data.amb_humid + cache.data.amb_temp + cache.data.temp_ch1 + cache.data.temp_ch2 + cache.data.ch1_hl + cache.data.ch1_hh + cache.data.ch2_hl + cache.data.ch2_hh + cache.data.ch1_cl + cache.data.ch1_ch + cache.data.ch2_cl + cache.data.ch2_ch;
    result = (char) temp;
    return result;
}

/** check the checksum stored matches the calculated
 */
bool check_checksum ( void )
{
    bool result = false;
    char calculated_checksum = calc_checksum( cache );
    if (checksum == calculated_checksum )
    {
        result = true;
    }
    return result;
}

