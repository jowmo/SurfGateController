#define BLUETOOTH_NAME "SurfGate"
#define BLUETOOTH_SPEED 9600
#define GATE_MINIMUM_KNOTS 6
#define GATE_MAXIMUM_KNOTS 12

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>


/*
 * Note that Arduino Uno only allows one tx and one rx.  
 * We'll initiate the Bluetooth first using both RX and TX, 
 * then we'll use Bluetooth TX only, and GPS RX only.
 * 
 * (Mega would allow TX and RX on both)
 */
SoftwareSerial gpsSerial(3, 2);   // GPS TX, GPS RX
SoftwareSerial btSerial(10, 11);  // BT TX, BT RX

Adafruit_GPS GPS(&gpsSerial);

/*
 * setup pins for actuator relays
 */
int relayPin1 = 6;                 // IN1 connected to digital pin 6
int relayPin2 = 7;                 // IN2 connected to digital pin 7
int relayPin3 = 8;                 // IN3 connected to digital pin 8
int relayPin4 = 9;                 // IN4 connected to digital pin 9

int portPin = 12;                  // The port gate switch
int starPin = 13;                  // The starboard gate switch

int audioPin = 4;                  // The piezo device

// define reset function for software-enable Arduino resets
void(* resetFunc) (void) = 0;

void setup()  
{   
  // piezo output pin
  pinMode(audioPin, OUTPUT);
  
  // Serial Setup
  Serial.begin(115200);
  Serial.println("SurfGateController Booting...");

  // Play startup tone
  playStartup();

  // Relay board setup
  setupRelay();

  // BT Setup
  setupBluetooth();

  // GPS Setup
  setupGPS();

  // Signal ready status
  playReady();
  Serial.println("All Systems Ready.");

  //
  // Setup the Interrupt used for GPS reading
  //
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function above
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void playStartup() {
  int freq = 500;
  for (int i=0; i<3; i++) {
    tone (audioPin, freq, 200);
    freq += 1000;
    delay(200);
  }
}


void playReady() {
  tone (audioPin, 2000, 1000);
  return; 
  delay(300);
  for (int i=0; i<4; i++) {
    tone (audioPin, 2500, 1000);
    delay(300);
  }
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
  digitalWrite(relayPin1, HIGH);        // Prevents relays from starting up engaged
  digitalWrite(relayPin2, HIGH);        // Prevents relays from starting up engaged
  digitalWrite(relayPin3, HIGH);        // Prevents relays from starting up engaged
  digitalWrite(relayPin4, HIGH);        // Prevents relays from starting up engaged
}

void testRelay()
{
  digitalWrite(relayPin1, LOW);   // energizes the relay and lights the LED
  delay(500);
  digitalWrite(relayPin2, LOW);   // energizes the relay and lights the LED
  delay(500);
  digitalWrite(relayPin3, LOW);   // energizes the relay and lights the LED
  delay(500);
  digitalWrite(relayPin4, LOW);   // energizes the relay and lights the LED
  delay(2000);
  digitalWrite(relayPin1, HIGH);    // de-energizes the relay and LED is off
  delay(500);
  digitalWrite(relayPin2, HIGH);    // de-energizes the relay and LED is off
  delay(500);
  digitalWrite(relayPin3, HIGH);    // de-energizes the relay and LED is off
  delay(500);
  digitalWrite(relayPin4, HIGH);    // de-energizes the relay and LED is off
}

void parseBluetoothSetupResponse() {
    delay(1000);    
    while (btSerial.available()) {
      Serial.write(btSerial.read());
    }
    Serial.write("\n");
}


// process incoming application commands
void commandProcessor() {
  
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();

  if (btSerial.available()) {
      char t = (char)btSerial.read();
      Serial.write(t);
  }
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


void engagePortGate() {
  digitalWrite(relayPin1, LOW); 
  delay(1000);
  digitalWrite(relayPin1, HIGH); 
}
void engageStarGate() {
  digitalWrite(relayPin3, LOW); 
  delay(1000);
  digitalWrite(relayPin3, HIGH);  
}
void retractGates() {
  //digitalWrite(relayPin2, LOW); 
  //digitalWrite(relayPin4, LOW);
  //delay(1000);
}

uint32_t timer = millis();
void loop()
{
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    

    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
    if (GPS.fix) {
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
      Serial.print("Free RAM: "); Serial.print(freeRam()); Serial.println("B");
      Serial.println();
    }

    if (GPS.speed < 1) {
      GPS.speed = 0;
    }

    // format: fix, satellies, knots, direction, in_motion, percent_deployed
    btSerial.print((int)GPS.fix);
    btSerial.print(",");
    btSerial.print((int)GPS.satellites);
    btSerial.print(",");
    btSerial.print(GPS.speed);
    btSerial.print(",");
    btSerial.print("STARBOARD"); // PORT, CENTER, STARBOARD
    btSerial.print(",");
    btSerial.print("0");
    btSerial.print(",");
    btSerial.print("80");   
    btSerial.println();

  
    int portEnabled = digitalRead(portPin);
    int starEnabled = digitalRead(starPin);
  
    Serial.print("Port enabled: "); Serial.println(portEnabled);
    Serial.print("Starboard enabled: "); Serial.println(starEnabled);
  
    if (digitalRead(portPin)) {
      //engagePortGate();
    }
    else if (digitalRead(starPin)) {
      //engageStarGate();
    }
    else {
      //retractGates();
    }
  
    
  }

}
