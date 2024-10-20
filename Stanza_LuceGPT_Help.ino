#include <IRremote.hpp>

#define PCTIME 1715848200 //change from here the PCTIME to change time in https://www.epochconverter.com/batch#results //


unsigned long lastBluOffTime = 0; // Variabile per memorizzare l'ultimo spegnimento del LED blu
unsigned long cooldownPeriod = 2000; // Periodo di inattività (2 secondi) prima di riattivare i PIR

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 1800000;  



#define SMARTPHONE_SPEGNIMENTO 0xF8F84D2
#define SMARTPHONE_SBARRE 0x1A8CAE
#define SPEGNIMENTO 0xBA45FF00
#define AZIONE_B 0xEA15FF00
#define AZIONE_N 0xBF40FF00


#define trigPin 12 // define TrigPin
#define echoPin 11 // define EchoPin
#define MAX_DISTANCE 200 // Maximum sensor distance is rated at 400-500cm.
// define the timeOut according to the maximum range. timeOut= 2*MAX_DISTANCE /100 /340 *1000000 = MAX_DISTANCE*58.8
#define MIN_DISTANZA_LUCE 2
#define MAX_DISTANZA_LUCE 10

float timeOut = MAX_DISTANCE * 60;
int soundVelocity = 340; // define sound speed=340m/s



/*
 T1357041600  
 *
 * A Processing example sketch to automatically send the messages is included in the download
 * On Linux, you can use "date +T%s\n > /dev/ttyACM0" (UTC time zone)
 */ 
 
#include <TimeLib.h>

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define IR_RECEIVE_PIN 9


#include <Wire.h>
#include <LiquidCrystal_I2C.h>




//the digital pin connected to the PIR sensor's output
int pirPin2 = 2;
int pirPin3 = 4;
const int ledBlu = 8; //Led blue modalità notte
const int ledLuce = 7; //Led della luce



//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 30;        
//the time when the sensor outputs a low impulse
long unsigned int lowIn;         



boolean lockLow = true;
boolean takeLowTime;  

unsigned long lastMotionTime = 0; // variabile per memorizzare il tempo dell'ultimo movimento


// Inizializza il display LCD con indirizzo I2C 0x27 e dimensioni 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long previousMillis = 0;
unsigned long lcdUpdateInterval = 1000;


bool eseguito = false;
bool available = false;
bool controlloBlu = false;
bool controlloLuce = false;

void setup() {
  Serial.begin(9600);
  
  // Inizializza il display LCD
  lcd.init();
  lcd.backlight();


  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver

  pinMode(ledBlu, OUTPUT);
  pinMode(ledLuce, OUTPUT);



  pinMode(pirPin2, INPUT);
  pinMode(pirPin3, INPUT);



  pinMode(trigPin,OUTPUT);// set trigPin to output mode
  pinMode(echoPin,INPUT); // set echoPin to input mode


  //give the sensor some time to calibrate
  Serial.println("calibrating sensor...");

  for(int i = 0; i < calibrationTime; i++){
    delay(1000);
  }

  Serial.println("Calibrarion done");

/*
  while (!Serial) ; // Needed for Leonardo only
  setSyncProvider( requestSync);  //set function to call when sync required
  Serial.println("Waiting for sync message");
 
  while(!available){
    if (Serial.available()) {
      processSyncMessage();
      available = true;
    }
  }
  */
  processSyncMessage();



}

void loop() {

//Serial.println("Fase tempo...");
unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= lcdUpdateInterval) {
    previousMillis = currentMillis;
    tempoI2C();
  }

  if (controlloBlu == false) {


    //Serial.println("Fase movimento...");
    sensoreMovimento();
    

            

  } else if(controlloBlu == true){

    //Serial.println("Fase notturna...");


  }
    
  infrarossi();
  accendiSpegni();

}


void tempoI2C(){
  //Serial.println("Dentro tempo");

    lcd.setCursor(0, 0);
    lcd.print("Giorno: ");
    lcd.print(day());
    lcd.print("/");
    lcd.print(month());

    lcd.setCursor(0, 1);
    lcd.print("ORA: ");
    lcd.print(hour());
    lcd.print(":");
    lcd.print(minute());
    
    

}


void accendiSpegni(){
  int i = 0;
  int j = 0;
  if(((getSonar()) > MIN_DISTANZA_LUCE) && ((getSonar() < MAX_DISTANZA_LUCE))){
    while(i < 4){
      if(((getSonar()) > MIN_DISTANZA_LUCE) && ((getSonar() < MAX_DISTANZA_LUCE))){
        j++;
        delay(300);
      }

      i++;

    }

    if(j > 3){
      // aggiorna il tempo dell'ultimo movimento
        lastMotionTime = millis();
        
        if(controlloLuce == true){
        
        digitalWrite(ledLuce, LOW); // Spegni luce
        controlloLuce = false;


        }
        else{
          digitalWrite(ledBlu, LOW);
          controlloBlu = false;

          digitalWrite(ledLuce, HIGH);
          controlloLuce = true;
        
        
        }


      delay(2000);

    }

   


  }

}



float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(trigPin, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,

  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pingTime = pulseIn(echoPin, HIGH, timeOut); // Wait HC-SR04 returning to the high level and measure out this waitting time


  distance = (float)pingTime * soundVelocity / 2 / 10000; // calculate the distance according to the time

  Serial.println(distance);
  return distance; // return the distance value
}




void sensoreMovimento(){

    if ((digitalRead(pirPin2) == HIGH)) {
        digitalWrite(ledBlu, LOW);
        controlloBlu = false;
        digitalWrite(ledLuce, HIGH); // accendi il led
        controlloLuce = true;

        // aggiorna il tempo dell'ultimo movimento
        lastMotionTime = millis();


    
        /* TESTING SENSORS
        if(digitalRead(pirPin2) == HIGH){
          Serial.println("LETTO");

        }

        if(digitalRead(pirPin3) == HIGH){
          Serial.println("SCRIVANIA");
        }
        */
    }

    // controlla se è trascorso il tempo di inattività
    if (millis() - lastMotionTime > pause) {
        digitalWrite(ledLuce, LOW); // spegni il led se è trascorso il tempo di inattività
        controlloLuce = false;
        
    }
}


void infrarossi(){
  if (IrReceiver.decode()) {

       if (IrReceiver.decodedIRData.decodedRawData == AZIONE_B || IrReceiver.decodedIRData.decodedRawData == AZIONE_N || IrReceiver.decodedIRData.decodedRawData == SMARTPHONE_SBARRE) {
          if(!controlloBlu){
            digitalWrite(ledBlu, HIGH);
            digitalWrite(ledLuce, LOW);
            controlloBlu = true;
            controlloLuce = false;

          }
          else{
            digitalWrite(ledBlu, LOW);
            controlloBlu = false;


            lastBluOffTime = millis(); // Registra l'ora di spegnimento

            delay(cooldownPeriod); //aspetta DELAY LUCE millisecondi

          }

          delay(2000);
        }

      if(IrReceiver.decodedIRData.decodedRawData == SMARTPHONE_SPEGNIMENTO || IrReceiver.decodedIRData.decodedRawData == SPEGNIMENTO){
         if(!controlloBlu){
            digitalWrite(ledBlu, HIGH);
            digitalWrite(ledLuce, LOW);
            controlloBlu = true;
            controlloLuce = false;

          }
          else{
            digitalWrite(ledBlu, LOW);
            controlloBlu = false;

            digitalWrite(ledLuce, HIGH);
            controlloLuce = true;

          }
        }

        delay(2000);
      }
   
    

    IrReceiver.resume(); // Enable receiving of the next value

}


/*
DA QUI IN POI LE FUNZIONE PER L'ORA E PER IL DISPLAY
*/
void digitalClockDisplay(){
  // digital clock display of the time
  
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void processSyncMessage(){
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
/*
  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     pctime = 1714435200;
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
*/
  pctime = PCTIME;
  if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
      setTime(pctime); // Sync Arduino clock to the time received on the serial port
    }
}

time_t requestSync(){
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}