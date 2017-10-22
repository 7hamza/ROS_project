#include <Wire.h>

//Librairie de l'ecran lcd
#include "rgb_lcd.h"

//Definition des PIN
#define PIR_MOTION_SENSOR 2
#define LED 4

//declaration variable ecran
rgb_lcd lcd;

void setup()
{
    //Setup du lcd 16x2 background bleu
    lcd.begin(16, 2);
    lcd.setRGB(0, 0, 255);

    
    //Setup en INPUt le detecteur de mouvement
    pinMode(PIR_MOTION_SENSOR, INPUT);
    
    //Setup de la LED en OUTPUT
    pinMode(LED,OUTPUT);
    
    //Setup du baud rate e sortie a 9600
    Serial.begin(9600);

}

void loop()
{   
    
    
    //Si qlqu'un est detecte
    if(isPeopleDetected()){
        
        //Led allume
        digitalWrite(LED, HIGH);
        
        //Ecriture sur port serie qu'on recupere sur Node-RED
        Serial.write("1");
              
        //Message de presence sur l ecran LCD Background rouge
        lcd.setRGB(255, 0, 0);
        lcd.setCursor(0, 0);
        lcd.print("Who's there o_O");
         delay(1000);
       
        
    }
      //Personne n'est detecte
    
    //Led eteinte
    digitalWrite(LED, LOW);
    //Bckground bleu     
    lcd.setRGB(0, 0, 255);
    //Message su l'ecran 
    lcd.setCursor(0, 0);
    lcd.print("All clear (^-^)");     
 
        
        
        
}


/***************************************************************/
/*Function: Detect whether anyone moves in it's detecting range*/
/*Return:-boolean, true is someone detected.*/
boolean isPeopleDetected()
{
    int sensorValue = digitalRead(PIR_MOTION_SENSOR);
    if(sensorValue == HIGH)//if the sensor value is HIGH?
    {
        return true;//yes,return true
    }
    else
    {
        return false;//no,return false
    }
}
