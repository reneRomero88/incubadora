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
int alarm1, alarm2,x1, x2, Setpoint2;
unsigned long tiempo; 
unsigned long tiempo2=0; 
unsigned long tiempo3=0;
LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHTPINN, DHTTYPEE);
DHT dht22(DHTPIN, DHTTYPE);
ACdimmer dimmer1(3);
ACdimmer dimmer2(5); //initialase port for dimmer: name(PinNumber);
ACdimmer dimmer3(6);
ACdimmer dimmer4(9);
Servo myservo;
int outVal = 0, Ventilador=0, pos, tempReading, resistencia=0;
///const int PBvolteo =  A7; 
const int Buttonplus = A1;     // the number of the pushbutton pin
const int Buttonless =  A0; 
int buttonStateplus = 0;
int buttonStateless = 0;
//int PBvolteo = 0;
int PBstate, M, k, pb;
double Setpoint, Input, PIDD;
//double Setpoint, Input, PIDD2;
//double aggKp = 4, aggKi = 0.2, aggKd = 0;
double consKp = 400, consKi =800 , consKd = 0;
float tempC; 

PID myPID(&Input, &PIDD, &Setpoint, consKp, consKi, consKd, DIRECT);
//PID myPID(&Input, &PIDD2, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
//void(* resetSoftware)(void) = 0;

void setup() {
    Serial.begin(19200);
    myservo.attach(10);
    pos = 100;
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

    Setpoint = 37.6; /// este es el set point de temperatura que se necesite
    Setpoint2 =30;
    myPID.SetMode(AUTOMATIC);   //turn the PID on

}

void loop() {
    PBstate= analogRead(Termistor);
    if(10<=tempC<=40) {
        dimmer1.begin(ON);
        dimmer2.begin(ON);
        dimmer3.begin(ON);
        dimmer4.begin(ON);
   
        tiempo=millis();
        if (tiempo-tiempo2 >= 14003600) {  //// los 14 millones es para cada tiempo de volteo 14,400,000= 4 horas
            tiempo2=tiempo;
            pb=pb+1;
            if(PBstate==0) {// para el boton cuando se quiere voltear y cuando no
                
                for (pos == 100 ; pos <=125 ; pos +=3) { // de 100° se mueve a 125° 
                    myservo.write(pos);
                    delay(350); // cada 350 milis se meueve 3°
                }
                delay(4500); /// tiempo de espera en 125°
                
                for (pos == 125 ; pos >=55 ; pos -=3) { // de 125° se mueve a 55°
                    myservo.write(pos);
                    delay(350); // cada 350 milis se mueve 3°
                }
                delay(4500); /// tiempo de espera en 55°
                
                for (pos == 55 ; pos <=100 ; pos +=3) { // de 55° se mueve a posición original 100°
                    myservo.write(pos);
                    delay(350);  /// cada 350 milis se mueve 3°
                }
            } else { /// cuando no solo mantener la posicion en 100°
                pos==100;
            }
        }
 
        for (int i = 0; i <= 20; i++) {
            buttonStateplus = digitalRead(Buttonplus);
            buttonStateless = digitalRead(Buttonless);
            outVal = map(PIDD, 0, 255, 0, 75); // hace un mapeo de 0 a 75 % de capacidad a la resistencia (arriba de 85% se satura)// analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
            Ventilador = map(PIDD, 0, 255,50, 85); // hace un mapeo de 50 a 85 % de capacidad al ventilador (arriba de 85% se satura)
            resistencia=map(PIDD, 0, 255, 0, 80);// hace un mapeo de 0 a 80 % de capacidad a la resistencia (arriba de 85% se satura) calienta mas esta que la primera
            dimmer1.setPower(outVal); // dimmer 1 con mapeo de outval
            dimmer2.setPower(Ventilador);// dimmer 3 con mapeo de ventilador
            dimmer3.setPower(resistencia); // dimmer 2 con mapeo de resistencia
            dimmer4.setPower(Ventilador); //dimer 4 con mapeo de ventilador
        
            float humidity = dht.readHumidity(); /// lee el sensor azul
            float temperature = dht.readTemperature();// lee el sensor azul
            float h = dht22.readHumidity(); // lee el sensor blanco
            float t = dht22.readTemperature();// lee el sensor blanco
            
            if(buttonStateplus==LOW) {// && Setpoint<=38.2)// 37.7 60%  ultimos 3 dias 36.5, 70 y 75%
                Setpoint=Setpoint+0.1;
            }
            
            if(buttonStateless==LOW) {  // && Setpoint>=36.8){
                Setpoint=Setpoint-0.1;
            }
        
            if(tempC>=Setpoint+0.1) {
                dimmer1.begin(OFF);
                dimmer3.begin(OFF);
            } else {
                dimmer1.begin(ON);
                dimmer2.begin(ON);
            
            }
          
            tempC = temperature;//t;//t+temperature)/2;  
            Input = tempC;
            x1  = Setpoint  - tempC;
            x2  = Setpoint2  - humidity;
            alarm1  = abs(x1);
            alarm2  = abs(x2);
            float gap = abs(Setpoint - Input); //distance away from setpoint
            myPID.SetTunings(consKp, consKi, consKd);
            myPID.Compute();
            ////Serial.print((int)tiempo);Serial.print("  ");
            ////Serial.print((int)tiempo2);Serial.print("  ");
            Serial.print((float)outVal);Serial.print("  ");
            Serial.print((float)Ventilador);Serial.print("  ");
            //Serial.print((float)temperatuzre); Serial.print(" *C, ");
            //Serial.print((float)t); Serial.print(" PID, ");
            //Serial.print((float)gap); Serial.print(" termistor, ");
            Serial.print((float)Setpoint); Serial.print(" a0, ");
            //  Serial.print((int)humidity); Serial.print(" %, ");
              Serial.print((int)tempC); Serial.println(" CC, ");
            //Serial.print((int)Input); Serial.println(" *C, ");
            
            lcd.setCursor(0, 0);  
            lcd.print("T:");
            lcd.print(temperature);
            lcd.print("    H:");
            lcd.print(humidity);
            
            lcd.setCursor(2, 1);
            lcd.print(t);
            lcd.print("      ");
            lcd.print(h);
        
            if(PBstate==0) {
              
                lcd.setCursor(0, 0);  
                lcd.print("VV:");
                lcd.print(pb);
                lcd.print("    H:");
                lcd.print(humidity);
        	    
                lcd.setCursor(2, 1);
                lcd.print(t);
                lcd.print("      ");
                lcd.print(h);
                
                lcd.setCursor(2, 2);
                lcd.print(tempC);
                lcd.print("  V   *");
                lcd.print(  Setpoint);
            } else {
                lcd.setCursor(0, 0);  
                lcd.print("T:");
                lcd.print(temperature);
                lcd.print("    H:");
                lcd.print(humidity);
        	    
                
                 lcd.setCursor(2, 1);
                lcd.print(t);
                lcd.print("      ");
                lcd.print(h);
                 
                 lcd.setCursor(2, 2);
                lcd.print(tempC);
                lcd.print("     *");
                lcd.print(  Setpoint);
            }    
            lcd.display();
            delay(1000);
        
            if(outVal>>0 && tempC>=Setpoint+0.9) {/// para fallas o errores reales que hace el sistema 
                M=1;
                outVal=0;
                Ventilador=0;
                PIDD=0;
                //tempC=Setpoint;
                lcd.setCursor(0, 3);
                lcd.write(2);
                lcd.print(mens1);
                lcd.write(2);
                lcd.display(); 
                //resetSoftware();
                outVal = map(PIDD, 0, 255, 0, 70); // analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
                Ventilador = map(PIDD, 0, 255, 50, 85);
                dimmer1.setPower(outVal); // dimmer 1
                dimmer2.setPower(Ventilador);// dimmer 3
                dimmer3.setPower(outVal); // dimmer 2
                dimmer4.setPower(Ventilador); //dimer 4
                dimmer1.begin(OFF);
                dimmer2.begin(OFF);
                dimmer3.begin(OFF);
                dimmer4.begin(OFF);
                float gap = abs(Setpoint - Input); //distance away from setpoint
                myPID.SetTunings(consKp, consKi, consKd);
                myPID.Compute();
                M=0;
                delay(3500);
            }
        
            if(k==0 && tempC==Setpoint) {/// para fallas o errores reales que hace el sistema
                M=1;
                k=1;
            }
        
            if (k==1 && tempC<=Setpoint-1.5) {
                outVal=0;
                tempC=Setpoint;
                Ventilador=0;
                PIDD=0;
                lcd.setCursor(0, 3);
                lcd.write(2);
                lcd.print(mens1);
                lcd.write(2);
                lcd.display(); 
                //resetSoftware();
                outVal = map(PIDD, 0, 255, 30, 70); // analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
                Ventilador = map(PIDD, 0, 255, 40, 85);
                dimmer1.setPower(outVal); // dimmer 1
                dimmer2.setPower(Ventilador);// dimmer 3
                dimmer3.setPower(outVal); // dimmer 2
                dimmer4.setPower(Ventilador); //dimer 4
                dimmer1.begin(OFF);
                dimmer2.begin(OFF);
                dimmer3.begin(OFF);
                dimmer4.begin(OFF);
                float gap = abs(Setpoint - Input); //distance away from setpoint
                myPID.SetTunings(consKp, consKi, consKd);
                myPID.Compute();
                M=0;
                delay(3500);
            }
        
            if (alarm1<=.3){
                lcd.setCursor(0, 3);
                lcd.write(1);
                lcd.print(mens1.substring(0, i));
                lcd.write(1);
                lcd.display();
            } else {
                lcd.setCursor(0, 3);
                lcd.write(2);
                lcd.print(mens2);
                lcd.write(2);
                lcd.display();     
            }
            delay(500);  
        }
    } else { /// para fallas o errores reales que hace el sistema
        outVal=0;
        tempC=Setpoint;
        Ventilador=0;
        PIDD=0;
        lcd.setCursor(0, 3);
        lcd.write(2);
        lcd.print(mens1);
        lcd.write(2);
        lcd.display(); 
        //resetSoftware();
        outVal = map(PIDD, 0, 255, 0, 70); // analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
        Ventilador = map(PIDD, 0, 255, 40, 85);
        dimmer1.setPower(outVal); // dimmer 1
        dimmer2.setPower(Ventilador);// dimmer 3
        dimmer3.setPower(outVal); // dimmer 2
        dimmer4.setPower(Ventilador); //dimer 4
        dimmer1.begin(OFF);
        dimmer2.begin(OFF);
        dimmer3.begin(OFF);
        dimmer4.begin(OFF);
        float gap = abs(Setpoint - Input); //distance away from setpoint
        myPID.SetTunings(consKp, consKi, consKd);
        myPID.Compute();
        M=0;
        delay(3500);
    }
}
