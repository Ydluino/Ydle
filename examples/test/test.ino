/* Author : Fabrice Scheider AKA Denia
 * Modified by PtitKev
 * Description : Sktech de test de la librairie
 * Licence : CC-BY-SA
 */

#include <TimerOne.h>
#include "Ydle.h"

#define RX_PIN 12
#define TX_PIN 10
#define BT_PIN 3

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
    
  }
}

// User callback to handle the new order
void dummy_callback(Frame_t *frame){
	Serial.println("Hey, i'm the callback dude !");
	Serial.print("Paquet recu de :");
	Serial.println(frame->sender);
	Serial.print("Type de trame:");
	Serial.println(frame->type);
}