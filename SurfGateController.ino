#define BLUETOOTH_NAME "SurfGate"
#define BLUETOOTH_SPEED 9600
#define BLUETOOTH_ENABLED true
#define GATE_MINIMUM_KNOTS 6
#define GATE_MAXIMUM_KNOTS 12
#define GPS_VERBOSE false
#define GPS_REQUIRED false

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>


class GateController
{
  bool firstUpdate = true;
  int relayOpenPin;
  int relayClosePin;
  int opening;
  int closing;
  long previousMillis;
  long actualMillis;
  long targetOpenMillis;
  long targetCloseMillis;

  public:
  GateController(int openPin, int closePin) {
    relayOpenPin = openPin;
    relayClosePin = closePin;
    targetOpenMillis = 0;
    targetCloseMillis = 0;
    previousMillis = 0;
    actualMillis = 0;
    opening = 0;
    closing = 0;
  }

  long Update(int on, unsigned long openDuration) {
    long currentMillis = millis();
    if (firstUpdate) {
      previousMillis = currentMillis;
      firstUpdate = false;
    }

    if (!on) {
      //Serial.println("NOT ON");
      on = 1;
      openDuration = 0;
    }
    else {
      //Serial.println("ON!!!");
    }

    // delta for de-bouncing
    long delta = abs(actualMillis - openDuration);
    //if (delta < 100) {Serial.print(" - DELTA : " ); Serial.print(delta); Serial.print(" target: "); Serial.println(openDuration);}

    if (on) {
      if ((actualMillis < openDuration) && (delta > 50)) {
        opening = 1;
        closing = 0;
        actualMillis = actualMillis + (currentMillis - previousMillis);
        //Serial.println("OPENING!"); 
        //Serial.print(" ActualMillis: "); Serial.print(actualMillis);
        //Serial.print(" openDuration: "); Serial.print(openDuration);
        //Serial.print(" delta: "); Serial.print(delta);
        Open();
      }
      else if ((actualMillis > openDuration) && (delta > 50)) {       
        opening = 0;
        closing = 1;
        //Serial.println("CLOSING! actualMillis:" + (String)actualMillis + " currentMillis:" + (String)currentMillis + " previousMillis:" + (String)previousMillis + " open duration:" + (String)openDuration);
        actualMillis = actualMillis - (currentMillis - previousMillis);
        if (actualMillis < 0) {
          actualMillis = 0;
        }
        //Serial.println("CLOSING! actualMillis:" + (String)actualMillis + " currentMillis:" + (String)currentMillis + " previousMillis:" + (String)previousMillis + " open duration:" + (String)openDuration);
        //Serial.println("---------------");
        Close();
      }
      else {       
        opening = 0;
        closing = 0;
        //Serial.println("STOPING!");
        actualMillis = openDuration; // for vanity so we don't include the small delta.
        Stop();
      }     
    }
    previousMillis = currentMillis;
    if (actualMillis < 0) actualMillis = 0;
    return actualMillis;
  }

  void Open() {
    digitalWrite(relayOpenPin, HIGH);
    digitalWrite(relayClosePin, LOW);   
  }

  void Stop() {
    digitalWrite(relayOpenPin, LOW);
    digitalWrite(relayClosePin, LOW);
  }

  void Close() {
    digitalWrite(relayOpenPin, LOW);  
    digitalWrite(relayClosePin, HIGH);   
  }

};


/*
 * Note that Arduino Uno only allows one tx and one rx.  
 * We'll initiate the Bluetooth first using both RX and TX, 
 * then we'll use Bluetooth TX only, and GPS RX only.
 * 
 * (Mega would allow TX and RX on both)
 */
SoftwareSerial btSerial(10, 11);  // BT TX, BT RX
SoftwareSerial gpsSerial(3,2);   // GPS TX, GPS RX


Adafruit_GPS GPS(&gpsSerial);
uint32_t timer = millis();

/*
 * setup pins for actuator relays
 */
int relayPin1 = 4;                 // IN1 connected to digital pin 6
int relayPin2 = 5;                 // IN2 connected to digital pin 7
int relayPin3 = 6;                 // IN3 connected to digital pin 8
int relayPin4 = 7;                 // IN4 connected to digital pin 9

int portPin = 12;                  // The port gate switch
int starPin = 13;                  // The starboard gate switch

int potPin = 2;                    // The Potentiometer for Gate Adjustment (Analog)

/*
 * Gate state variables
 */
int enabled = 0;
int portExtendedDuration = 0;
int starExtendedDuration = 0;
int portDurationTarget = 0;
int starDurationTarget = 0;
int portExtending = 0;
int starExtending = 0;
float gatePotOldValue = 0;

int maxExtenstiopenDurationMS = 5000;

GateController portGateController(relayPin1,relayPin2);
GateController starGateController(relayPin3,relayPin4);


// define reset function for software-enable Arduino resets
void(* resetFunc) (void) = 0;

void setup()  
{   
  // Serial Setup
  Serial.begin(115200);
  delay(1000);
  Serial.println("SurfGateController Booting...");

  // Relay board setup
  setupRelay();

  // BT Setup
  setupBluetooth();

  // GPS Setup
  setupGPS();

  //
  // Setup the Interrupt used for GPS reading
  //
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function above
  Serial.println("Setting GPS Interrupt.");
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  // analogRead(potPin);
  gatePotOldValue = analogRead(potPin);

  // Signal ready status
  Serial.println("All Systems Ready.");
}

int getGateExtensionOpenDuration() {
  float value = analogRead(potPin);
  int delta = abs(value - gatePotOldValue);
  if (delta < 50) {
    value = gatePotOldValue;
  }
  else {
    gatePotOldValue = value;    
  }

  float adjust = value / 1024;
  int duration = int(maxExtenstiopenDurationMS * adjust);
  duration = ((int)duration/100 + ((int)duration%100>2)) * 100;
  return duration;
}

void setupBluetooth() {
  Serial.print("Initializing Bluetooth at "); Serial.print(BLUETOOTH_SPEED); Serial.println(" BAUD...");
  btSerial.begin(BLUETOOTH_SPEED);
  delay(1000);

  // Should respond with OK
  btSerial.print("AT");
  parseBluetoothSetupResponse();

  // Should respond with its version
  btSerial.print("AT+VERSION");
  parseBluetoothSetupResponse();

  // Set pin to 0000
  btSerial.print("AT+PIN0000");
  parseBluetoothSetupResponse();

  // Set the name to BLUETOOTH_NAME
  btSerial.print("AT+NAME");
  btSerial.print(BLUETOOTH_NAME);
  delay(2000);
  parseBluetoothSetupResponse();

  // Set baudrate to 9600 is 4, 57600 is 7, 115200 is AT+BAUD8
  btSerial.print("AT+BAUD4");
  parseBluetoothSetupResponse();

  Serial.println("Bluetooth setup done.");   
}


void setupGPS() {
  Serial.println("Initializing GPS at 9600 BAUD...");
  delay(10000);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE); // show GPS firmware version
  Serial.print("GPS Firmware Version: "); Serial.println(PMTK_Q_RELEASE);  
  Serial.println("GPS setup done.");
}


void setupGateSwitch() {
  pinMode(starPin, INPUT);
  pinMode(portPin, INPUT);
  digitalWrite(starPin, LOW);
  digitalWrite(portPin, LOW);
}

void setupRelay()
{
  pinMode(relayPin1, OUTPUT);      // sets the digital pin as output
  pinMode(relayPin2, OUTPUT);      // sets the digital pin as output
  pinMode(relayPin3, OUTPUT);      // sets the digital pin as output
  pinMode(relayPin4, OUTPUT);      // sets the digital pin as output
  digitalWrite(relayPin1, LOW);    // Prevents relays from starting up engaged
  digitalWrite(relayPin2, LOW);    // Prevents relays from starting up engaged
  digitalWrite(relayPin3, LOW);    // Prevents relays from starting up engaged
  digitalWrite(relayPin4, LOW);    // Prevents relays from starting up engaged
}

void testRelay()
{
  Serial.println("Relay Test Beginning.");
  digitalWrite(relayPin1, HIGH);   // energizes the relay and lights the LED
  delay(500);
  digitalWrite(relayPin2, HIGH);   // energizes the relay and lights the LED
  delay(500);
  digitalWrite(relayPin3, HIGH);   // energizes the relay and lights the LED
  delay(500);
  digitalWrite(relayPin4, HIGH);   // energizes the relay and lights the LED
  delay(2000);
  digitalWrite(relayPin1, LOW);    // de-energizes the relay and LED is off
  delay(500);
  digitalWrite(relayPin2, LOW);    // de-energizes the relay and LED is off
  delay(500);
  digitalWrite(relayPin3, LOW);    // de-energizes the relay and LED is off
  delay(500);
  digitalWrite(relayPin4, LOW);    // de-energizes the relay and LED is off
  Serial.println("Relay Test Complete.");
}

void parseBluetoothSetupResponse() {
    delay(1000);    
    while (btSerial.available()) {
      Serial.write(btSerial.read());
    }
    Serial.write("\n");
}

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  GPS.read();
  if (btSerial.available()) {
      char t = (char)btSerial.read();
      Serial.write(t);
  }
}


void loop()
{ 
  /*
   * GPS and Communication Engine
   */
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  /*
   * Read Sensors
   */
  int gateDuration = getGateExtensionOpenDuration();  
  int portEnabled = digitalRead(portPin);
  int starEnabled = digitalRead(starPin);
  
  String dir = String();
  if(portEnabled) dir = "STARBOARD";
  else if (starEnabled) dir = "PORT";
  else dir = "CENTER";

  // override the enabled bit if we have GPS and we're outside of acceptable speed limits
  if (GPS_REQUIRED) {
    if (GPS.fix) {
      if (GPS.speed < 7 || GPS.speed > 14) {
        //Serial.println("Speed out-of-bounds. Disabling gates.");
        enabled = 0;
      }
      else {
        enabled = 1;
      }
    } 
  }
  else {
    enabled = 1;
  }

  if (!enabled) {
    portEnabled = 0;
    starEnabled = 0;    
  }

  /*
   * Gate Engine
   */
  long portPosition = portGateController.Update(portEnabled, gateDuration);
  long starPosition = starGateController.Update(starEnabled, gateDuration);


  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  {
    timer = millis();
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 500) { 
    timer = millis(); // reset the timer
    
    if (GPS.fix && GPS_VERBOSE) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.println();
    }

    if (GPS.speed < 2) {
      GPS.speed = 0;
    }

    // output statistics...
    // format: name:value,name1:value1,name2:value2,...
    String fix = GPS.fix ? "1" : "0";
    String sats = GPS.satellites ? (String)GPS.satellites : "0";
    int portPositionPercentage = ((float)portPosition / (float)gateDuration) * 100;
    int starPositionPercentage = ((float)starPosition / (float)gateDuration) * 100;
    
    String stats = String();
    stats += "enabled:" + (String)enabled + ",";
    stats += "gps_fix:" + fix + ",";
    stats += "gps_fix_quality:" + (String)GPS.fixquality + ",";    
    stats += "satelites:" + sats + ",";
    stats += "speed:" + (String)GPS.speed + ",";
    stats += "direction:" + dir + ",";
    stats += "port_position_percent:" + (String)portPositionPercentage + ",";
    stats += "starboard_position_percent:" + (String)starPositionPercentage + ",";
    stats += "gate_duration:" + (String)gateDuration + ",";
    stats += "free_ram:" + (String)freeRam();
    Serial.println(stats);

    btSerial.println(stats);
  }

}

