/* Author : Xylerk
 * Modified by PtitKev
 * Description : Envoi d'une trame contenant une température et une humidité générées aléatoirement
 * Licence : CC-BY-SA
 */

#include <TimerOne.h>
#include "Ydle.h"

#define RX_PIN 12
#define TX_PIN 10
#define BT_PIN 3

// On définit les constantes des index
// pour plus de lisibilité dans le code
#define DATA_DEGREEC 0
#define DATA_HUMIDITY 1

// Déclaration de la variable Ydle
ydle y;

void setup()
{
  // Définit une configuration de base si nécessaire
  // m_Config.IdMaster = 1;
  // m_Config.IdNode = 2;
  // eeprom_write_block((void*)&m_Config, 0, sizeof(m_Config));

  // Init du serial
  Serial.begin(115200);

  // Init de la node,
  ydle y(RX_PIN, TX_PIN, BT_PIN);
  y.init_timer();
}

void loop()
{
  y.receive();
  if(y.initialized()){
    // Génération température alétoire et affichage moniteur série
    float temperature=(random(100)+.01*random(100))*(-1)*pow(2,random(2));
    Serial.print("Température : ");
    Serial.println(temperature);

    // Génération humidité alétoire et affichage moniteur série
    float humidity=random(100)+.01*random(100);
    Serial.print("Humidité : ");
    Serial.println(humidity);

    // Construction trame
    Frame_t frame;

    //Déclaration type de trame
    y.initFrame(&frame, YDLE_TYPE_STATE);

    //Ajout température
    y.addData(&frame, DATA_DEGREEC, temperature);

    //Ajout humidité
    y.addData(&frame, DATA_HUMIDITY, humidity);

    //envoi de la trame
    y.send(&frame);

    // Pause de 5 secondes
    delay(5000);
  }
}