// Ydle.h
//
// Ydle implementation for Arduino
// See the README file in this directory for documentation
//
// Authors:
// Fabrice Scheider AKA Denia,
// Manuel Esteban AKA Yaug
// Matthieu Desgardin AKA Zescientist
// Yargol AKA Yargol
// Xylerk
// PtitKev
//
// WebPage: http://www.ydle.fr/index.php
// Contact: http://forum.ydle.fr/index.php
// Licence: CC by sa (http://creativecommons.org/licenses/by-sa/3.0/fr/)
//
// To use the Ydle library, you must have:
// #include <Ydle.h>
// At the top of your sketch.

#ifndef YDLE_H
#define YDLE_H

#include <stdlib.h>
#include <Arduino.h>

#if defined(ARDUINO)
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <wiring.h>
#endif
#else // error
#error Platform not defined
#endif

#define _YDLE_DEBUG

#define YDLE_MAX_FRAME 2

#define YDLE_MAX_SIZE_FRAME 64
#define YDLE_ACK_TIMEOUT 1000

#define YDLE_TYPE_STATE			1 // Node send data
#define YDLE_TYPE_CMD				2 // ON/OFF sortie etc...
#define YDLE_TYPE_ACK				3 // Acquit last command
#define YDLE_TYPE_STATE_ACK		4 // Node send data and want ACK

#define YDLE_DATA_BOOL				0 // true / false (1 bit / 8 bits data)
#define YDLE_DATA_UINT8			1 // (8 bits / 16 bits data)
#define YDLE_DATA_UINT16			2 // (16 bits / 24 bits data)
#define YDLE_DATA_UINT24			3 // (24 bits / 32 bits data)

#define YDLE_CMD_LINK				0 // Link a node to the master
#define YDLE_CMD_ON					1 // Send a ON command to node data = N° output
#define YDLE_CMD_OFF				2 // Send a OFF command to node data = N° output
#define YDLE_CMD_RESET				3 // Ask a node to reset is configuration
#define YDLE_CMD_SET				4 // Set value
#define YDLE_CMD_GET				5 // Get value
#define YDLE_CMD_PING				6 // PING

// Défini un type de structure Frame_t
struct Frame_t
{
	uint8_t receptor;		// 8 bytes
	uint8_t sender;		// 8 bytes
	uint8_t type;			// 2 bytes
	uint8_t taille;			// 3 bytes data len + crc in BYTES
	uint8_t data[30];		//
	uint8_t crc;			// 8 bytes
} ;

// Défini un type de structure Config_t
struct Config_t
{
	uint8_t IdMaster;		// Id du master
	uint8_t IdNode;		// Id de la node
} ;

extern "C" {
	// callback function
	typedef void (*ydleCallbackFunction)(Frame_t *frame) ;
}

volatile static Config_t m_Config;
volatile static bool m_bLnkSignalReceived;
volatile static bool m_initializedState; // Indique si le node est initialisé

class ydle
{
	private:
		ydleCallbackFunction callback;
		bool _callback_set;

	public:
		static ydle * ydleObj;
		// Le constructeur qui lance une instance avec les numéros des pins de l'émetteur, du récepteur et du boutton
		ydle(int rx, int tx, int button) ;

		// Le constructeur qui lance une instance avec les numéros des pins de l'émetteur, du récepteur et du boutton
		// Par défaut, le récepteur est en 12, l'émetteur en 10 et le boutton en 3
		ydle() ;

		static void timerInterrupt() ;
		void pll() ;

		// Used to read the configuration
		void ReadConfig() ;

		// Envoie des verrous et des bits formant une trame
		uint8_t send(Frame_t *frame) ;

		// New function need to be called by the main function in order to handle the new received frame
		uint8_t receive() ;

		// Ecoute le récepteur pour l'arrivée d'un signal
		void listenSignal() ;

		// Crée une trame avec les infos données en paramètre
		void initFrame(Frame_t *frame, unsigned long destination, unsigned long sender, unsigned long type) ;

		// Crée une trame avec le type
		void initFrame(Frame_t *frame, unsigned long type) ;

		// extract any type of data from receivedsignal
		int extractData(Frame_t *frame, int index, int &itype, long &ivalue) ;

		// add TYPE_CMD data
		void addCmd(Frame_t *frame, int type, int data) ;

		// Retourne l'état de la Node
		bool initialized() ;

		// Retourne l'état du bouton de reset
		bool resetButton() ;

		// Launch the timer for the receive function
		void init_timer() ;

		// Function to attach a user defined function for handle received frame
		void attach(ydleCallbackFunction function) ;

		// Remove EEProm values & change initialized state
		static void resetNode() ;

		// Ajout d'une donnée
		void addData(Frame_t *frame, int type, bool data) ;
		void addData(Frame_t *frame, int type, int data) ;
		void addData(Frame_t *frame, int type, long int data) ;
		void addData(Frame_t *frame, int type, float data) ;

		// CRC calculation
		unsigned char computeCrc(Frame_t *frame) ;

	private:
		// Fonctions de débogage
		void log(String msg) ;
		void log(String msg, int i) ;

		// Affiche le contenue d'une trame
		void printFrame(Frame_t *trame) ;

		// Do something with a received Command
		void onCommandReceived(Frame_t *frame) ;

		// Compare le signal reçu au signal de référence
		bool checkSignal(Frame_t *frame) ;

		// EEprom handling, to serialize the configuraiton
		void writeEEProm() ;
		void readEEProm() ;

		uint8_t crc8(const uint8_t* buf, uint8_t length) ;
} ;

#endif