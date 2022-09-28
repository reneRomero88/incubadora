#include "DHT.h"
#include <PID_v1.h>
#include <JELdimmer2.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define DHTPINN A2 
#define DHTTYPEE DHT11 //#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN A3  
#define DHTTYPE DHT22 
#define Termistor A6
//const int DHTPIN = 6;

byte Heart[] = {
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000
};
byte Speaker[] = {
  B10001,
  B11011,
  B01110,
  B10101,
  B01110,
  B11011,
  B10001,
  B00000
};

String mens1 = " FAMILIA ROMERO E " ;
String mens2 = "ALARMA TEMPERATURA" ;
String mens3 = "ALARMA TEMPERATURA" ;
int alarm1,x1;
unsigned long timeBuffer = 0; 
LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHTPINN, DHTTYPEE);
DHT dht22(DHTPIN, DHTTYPE);
ACdimmer dimmer1(3);
ACdimmer dimmer2(5); //initialase port for dimmer: name(PinNumber);
ACdimmer dimmer3(6);
ACdimmer dimmer4(9);
Servo myservo;
int outVal = 0, Ventilador=0, tempReading, resistencia=0;
///const int PBvolteo =  A7; 
const int Buttonplus = A1;     // the number of the pushbutton pin
const int Buttonless =  A0;
//int PBvolteo = 0;
int PBstate, M, k, pb;
double Setpoint, Input, PIDD;
double consKp = 400, consKi =800 , consKd = 0;
float tempC, tempA, tempB, tempMin = 0, tempMax = 0, tempErr = -0.6; 

PID myPID(&Input, &PIDD, &Setpoint, consKp, consKi, consKd, DIRECT);
//PID myPID(&Input, &PIDD2, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
//void(* resetSoftware)(void) = 0;
void controlMotor();
void cicloMotor(int inicio, int fin, int type);
void controlEventos();
void readValues(int i);
void controlDimmer();

void setup() {
    Serial.begin(19200);
    myservo.attach(10);
    lcd.init();
    lcd.backlight();
    lcd.createChar(1, Heart);
    lcd.createChar(2, Speaker);
    dht.begin();
    dht22.begin();
 
    dimmer1.begin(ON);
    dimmer2.begin(ON);
    dimmer3.begin(ON);
    dimmer4.begin(ON);
   
    pinMode(Buttonplus, INPUT_PULLUP);
    pinMode(Buttonless, INPUT_PULLUP);
    //pinMode(PBvolteo, INPUT_PULLUP);

    Setpoint = 37.6 + tempErr; /// este es el set point de temperatura que se necesite
    myPID.SetMode(AUTOMATIC);   //turn the PID on
    tempMin = tempMax = Setpoint;

}

void controlMotor() {
    unsigned long time = 0;
    const int posAtras = 55;
    const int posArriba = 100;
    const int posFrente = 125;
    const int creciente = 1;
    const int decreciente = 0;
    const long timeCiclo = 600000; //los 14 millones es para cada tiempo de volteo 14,000,000 = 4 horas
    
    PBstate = analogRead(Termistor);
	PBstate = 0;
    time = millis();
    
    if (time - timeBuffer >= timeCiclo) {
        timeBuffer = time;
        if(PBstate == 0) {// para el boton cuando se quiere voltear y cuando no
            pb = pb + 1;
            cicloMotor(posArriba, posFrente, creciente);
            cicloMotor(posFrente, posAtras, decreciente);
            cicloMotor(posAtras, posArriba, creciente);
        }
    }
}

void cicloMotor(int inicio, int fin, int type) {
    const int grados = 3; //avance de 3°
    const int milles = 1000; //tiempo de espera
    const int timeAvance = 350; //cada 350 milis se meueve X°
    const int timeEspera = 4500; //tiempo de espera

    if (type == 0) {
        for (int pos = inicio; pos >= fin; pos -= grados) {
            myservo.write(pos);
            delay(timeAvance);
        }
    } else  {
        for (int pos = inicio ; pos <= fin ; pos += grados) {
            myservo.write(pos);
            delay(timeAvance);
        }
    }
  
    delay(timeEspera); /// tiempo de espera en 55°
}

void controlEventos() { 
    int buttonStateplus = digitalRead(Buttonplus);
    int buttonStateless = digitalRead(Buttonless); 

    if (buttonStateplus == LOW) {
        Setpoint = Setpoint + 0.1;
    }

    if (buttonStateless == LOW) {
        Setpoint = Setpoint - 0.1;
    }
}

void readValues(int i) {
    float humA = dht.readHumidity(); /// lee el sensor azul
    float tempA = dht.readTemperature();// lee el sensor azul
    float humB = dht22.readHumidity(); // lee el sensor blanco
    float tempB = dht22.readTemperature();// lee el sensor blanco

    tempC = tempA;

    if (tempMax <= tempC) {
        tempMax = tempC;
    }

    if (tempMin >= tempC) {
        tempMin = tempC;
    }

    unsigned long runMillis = millis();
    unsigned long allSeconds = millis() / 1000;
    int runHours= allSeconds / 3600;
    int secsRemaining = allSeconds % 3600;
    int runMinutes = secsRemaining / 60;
    int runSeconds = secsRemaining % 60;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(runHours);
    lcd.print(":");
    lcd.print(runMinutes);
    lcd.print(":");
    lcd.print(runSeconds);

    lcd.setCursor(3, 1);
    lcd.print("K:");
    lcd.print(Setpoint);
    lcd.setCursor(3, 2);
    lcd.print("A:");
    lcd.print(tempA);
    lcd.setCursor(3, 3);
    lcd.print("B:");
    lcd.print(tempB);

    lcd.setCursor(11, 0);
    lcd.print("Hum:");
    lcd.print(humA);

    lcd.setCursor(11, 1);
    //lcd.print("Humedad:");
    //lcd.setCursor(11, 1);
    lcd.print("Min:");
    lcd.print(tempMin);
    //lcd.print("%");
    lcd.setCursor(11, 2);
    lcd.print("Max:");
    lcd.print(tempMax);
    //lcd.print("%");

    //lcd.setCursor(11, 0);
    //lcd.print("Humedad:");
    //lcd.setCursor(11, 1);
    //lcd.print("A:");
    //lcd.print(humA);
    //lcd.print("%");
    //lcd.setCursor(11, 2);
    //lcd.print("B:");
    //lcd.print(humB);
    //lcd.print("%");

    lcd.setCursor(11, 3); 
    lcd.print("L:");
    lcd.print(i);
    lcd.print("-");
    lcd.print(pb);
}

void controlDimmer() {
    dimmer1.begin(ON);
    dimmer2.begin(ON);
    dimmer3.begin(ON);
    dimmer4.begin(ON);
    outVal = map(PIDD, 0, 255, 0, 75); // hace un mapeo de 0 a 75 % de capacidad a la resistencia (arriba de 85% se satura)// analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
    Ventilador = map(PIDD, 0, 255,40, 85); // hace un mapeo de 50 a 85 % de capacidad al ventilador (arriba de 85% se satura)
    resistencia=map(PIDD, 0, 255, 0, 70);// hace un mapeo de 0 a 80 % de capacidad a la resistencia (arriba de 85% se satura) calienta mas esta que la primera
    dimmer1.setPower(outVal); // dimmer 1 con mapeo de outval
    dimmer2.setPower(Ventilador);// dimmer 3 con mapeo de ventilador
    dimmer3.setPower(resistencia); // dimmer 2 con mapeo de resistencia
    dimmer4.setPower(Ventilador); //dimer 4 con mapeo de ventilador

    if (tempC >= Setpoint+0.1) {
        dimmer1.begin(OFF);
        dimmer3.begin(OFF);
    } else {
        dimmer1.begin(ON);
        dimmer3.begin(ON);

    }
}

void loop() {
    const int tempAlertMin = 10;
    const int tempAlertMax = 40;

    readValues(0);
    if (tempAlertMin <= tempC <= tempAlertMax) {
    
        controlMotor();
        
        //for (int i = 0; i <= 20; i++) {
            controlEventos();
        //    readValues(i);
            controlDimmer();
            Input = tempC;
            x1  = Setpoint  - tempC;
            alarm1  = abs(x1);
            myPID.SetTunings(consKp, consKi, consKd);
            myPID.Compute();
            
            //lcd.setCursor(0, 0);  
            //lcd.print("T:");
            //lcd.print(temperature);
            //lcd.print("    H:");
            //lcd.print(humidity);
            
            //lcd.setCursor(2, 1);
            //lcd.print(t);
            //lcd.print("      ");
            //lcd.print(h);
        
            //if(PBstate==0) {
              
                //lcd.setCursor(0, 0);  
                //lcd.print("VV:");
                //lcd.print(pb);
                //lcd.print("    H:");
                //lcd.print(humidity);
                
                //lcd.setCursor(2, 1);
                //lcd.print(t);
                //lcd.print("      ");
                //lcd.print(h);
                
                //lcd.setCursor(2, 2);
                //lcd.print(tempC);
                //lcd.print("  V   *");
                //lcd.print(  Setpoint);
            //} else {
                //lcd.setCursor(0, 0);  
                //lcd.print("T:");
                //lcd.print(temperature);
                //lcd.print("    H:");
                //lcd.print(humidity);
                
                
                 //lcd.setCursor(2, 1);
                //lcd.print(t);
                //lcd.print("      ");
                //lcd.print(h);
                 
                 //lcd.setCursor(2, 2);
                //lcd.print(tempC);
                //lcd.print("     *");
                //lcd.print(  Setpoint);
            //}    
            lcd.display();
            delay(1000);
        
            //if(outVal>>0 && tempC>=Setpoint+0.9) {/// para fallas o errores reales que hace el sistema 
            //    M=1;
            //    outVal=0;
            //    Ventilador=0;
            //    PIDD=0;
                //--tempC=Setpoint;
                //lcd.setCursor(0, 3);
                //lcd.write(2);
                //lcd.print(mens1);
                //lcd.write(2);
                //lcd.display(); 
                //resetSoftware();
            //    outVal = map(PIDD, 0, 255, 0, 70); // analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
            //    Ventilador = map(PIDD, 0, 255, 50, 85);
            //    dimmer1.setPower(outVal); // dimmer 1
            //    dimmer2.setPower(Ventilador);// dimmer 3
            //    dimmer3.setPower(outVal); // dimmer 2
            //    dimmer4.setPower(Ventilador); //dimer 4
            //    dimmer1.begin(OFF);
            //    dimmer2.begin(OFF);
            //    dimmer3.begin(OFF);
            //    dimmer4.begin(OFF);
            //    myPID.SetTunings(consKp, consKi, consKd);
            //    myPID.Compute();
            //    M=0;
            //    delay(3500);
            //}
        
            //if(k==0 && tempC==Setpoint) {/// para fallas o errores reales que hace el sistema
            //    M=1;
            //    k=1;
            //}
        
            //if (k==1 && tempC<=Setpoint-1.5) {
            //    outVal=0;
                //--tempC=Setpoint;
            //    Ventilador=0;
            //    PIDD=0;
                //lcd.setCursor(0, 3);
                //lcd.write(2);
                //lcd.print(mens1);
                //lcd.write(2);
                //lcd.display(); 
                //resetSoftware();
            //    outVal = map(PIDD, 0, 255, 30, 70); // analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
            //    Ventilador = map(PIDD, 0, 255, 40, 85);
            //    dimmer1.setPower(outVal); // dimmer 1
            //    dimmer2.setPower(Ventilador);// dimmer 3
            //    dimmer3.setPower(outVal); // dimmer 2
            //    dimmer4.setPower(Ventilador); //dimer 4
            //    dimmer1.begin(OFF);
            //    dimmer2.begin(OFF);
            //    dimmer3.begin(OFF);
            //    dimmer4.begin(OFF);
            //    myPID.SetTunings(consKp, consKi, consKd);
            //    myPID.Compute();
            //    M=0;
            //    delay(3500);
            //}
        
            //if (alarm1<=.3){
                //lcd.setCursor(0, 3);
                //lcd.write(1);
                //lcd.print(mens1.substring(0, i));
                //lcd.write(1);
                //lcd.display();
            //} else {
                //lcd.setCursor(0, 3);
                //lcd.write(2);
                //lcd.print(mens2);
                //lcd.write(2);
                //lcd.display();     
            //}
            delay(500);  
//        }
    }// else { /// para fallas o errores reales que hace el sistema
     //   outVal=0;
        //--tempC=Setpoint;
     //   Ventilador=0;
     //   PIDD=0;
        //lcd.setCursor(0, 3);
        //lcd.write(2);
        //lcd.print(mens1);
        //lcd.write(2);
        //lcd.display(); 
        //resetSoftware();
     //   outVal = map(PIDD, 0, 255, 0, 70); // analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
     //   Ventilador = map(PIDD, 0, 255, 40, 85);
     //   dimmer1.setPower(outVal); // dimmer 1
     //   dimmer2.setPower(Ventilador);// dimmer 3
     //   dimmer3.setPower(outVal); // dimmer 2
     //   dimmer4.setPower(Ventilador); //dimer 4
     //   dimmer1.begin(OFF);
     //   dimmer2.begin(OFF);
     //   dimmer3.begin(OFF);
     //   dimmer4.begin(OFF);
     //   myPID.SetTunings(consKp, consKi, consKd);
     //   myPID.Compute();
     //   M=0;
     //   delay(3500);
    //}
}
