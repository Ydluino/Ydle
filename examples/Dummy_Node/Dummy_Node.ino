/* Author : Fabrice Scheider AKA Denia
 * Modified by Xylerk PtitKev
 * Description : Template avec les déclarations minimales pour le fonctionnement de la bibliothèque ydle 
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
		Frame_t frame;
		// Choisir le type de trame à envoyer
		y.initFrame(&frame, YDLE_TYPE_STATE);
		// Ajouter les données dans la trame
		y.addData(&frame, /* index de donnée */, /* donnée */);
		y.addData(&frame, /* index de donnée */, /* donnée */);
		// Envoyer la trame
		y.send();
	}
}
