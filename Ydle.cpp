// Ydle.cpp
//
// Ydle implementation for Arduino
// See the README file in this directory for documentation.
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
// Pll function inspired on VirtualWire library

#include <TimerOne.h>
#include <avr/eeprom.h>

#include "Float.h"
#include "Crc.h"
#include "Ydle.h"

const PROGMEM char _atm_crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

// On déclare les structures
static Frame_t g_SendFrame; // send frame
static Frame_t g_ReceivedFrame; // received frame
static Frame_t g_FrameBuffer[YDLE_MAX_FRAME];
static Frame_t g_SendFrameBuffer; // Une seule pour le moment

static uint8_t m_data[YDLE_MAX_SIZE_FRAME]; // data + crc

static uint8_t pinRx = 12; // Le numéro de la broche IO utilisée pour le module récepteur
static uint8_t pinTx = 10; // Le numéro de la broche IO utilisée pour le module émetteur
static uint8_t pinLed = 13; // Le numéro de la broche IO utilisée pour la Led de statut
static uint8_t pinCop = 8; // Permet la recopie du signal sur une sortie pour vérification à l'oscilloscope
static uint8_t pinButton = 3; // Le numéro de la broche IO utilisée pour l'installation du boutton de resettage

static uint16_t start_bit = 0x6559; // Octet de start
static uint8_t start_bit2 = 0b01000010; // Octet de start après décodage Manchester

volatile uint8_t sample_value = 0; // Disponibilité d'un sample
volatile uint8_t sample_count = 1; // Nombre de samples sur la période en cours
volatile uint8_t last_sample_value = 0; // La valeur du dernier sample reçu
// La somme des valeurs des samples. si inférieur à 5 "1" samples dans le cycle de PLL
// le bit est déclaré comme 0, sinon à 1
static uint8_t sample_sum = 0;

// La rampe PLL, varie entre 0 et 159 sur les 8 samples de chaque période de bit
// Quand la PLL est synchronisée, la transition de bit arrive quand la rampe vaut 0
static uint8_t pll_ramp = 0;

static uint16_t rx_bits = 0; // Les 16 derniers bits reçus, pour repérer l'octet de start
static uint8_t rx_active = 0; // Flag pour indiquer la bonne réception du message de start

#define YDLE_SPEED 1000 // Le débit de transfert en bits/secondes
#define YDLE_TPER 1000000/YDLE_SPEED // La période d'un bit en microseconds
#define YDLE_FBIT YDLE_TPER/8 // La fréquence de prise de samples

static uint8_t bit_value = 0; // La valeur du dernier bit récupéré
static uint8_t bit_count = 0; // Le nombre de bits récupérés

static uint8_t sender = 0; // Id sender reçue
static uint8_t receptor = 0; // Id receptor reçue
static uint8_t type = 0; // Info type reçue
static uint8_t taille = 0; // Info taille reçue

static uint8_t rx_bytes_count = 0; // Nombre d'octets reçus
static uint8_t length_ok = 0; // Disponibilité de la taille de trame

volatile uint8_t wait_ack = 0;
volatile uint8_t last_check = 0;
volatile uint8_t retry = 0;

static uint8_t rx_done = 0; // Si le message est complet
static uint8_t frameReadyToBeRead = false;
static uint8_t pFrame = 0;
volatile uint8_t transmission_on = false;

static int tx_sample = 0;
static int tx_bit = 7;
static int tx_index = 0;
static bool bit_test = false;
volatile uint8_t frameToSend[40];
volatile uint8_t frameToSendLength = 0;

ydle * ydle::ydleObj;

// Initialisation des IO avec des valeurs entrées par l'utilisateur

ydle::ydle(int rx, int tx, int button) {
    m_bLnkSignalReceived = false;
    m_initializedState = false;
    _callback_set = false;
    readEEProm();

    pinRx = rx;
    pinTx = tx;
    pinButton = button;

    pinMode(pinRx, INPUT);
    pinMode(pinTx, OUTPUT);
    pinMode(pinButton, INPUT);
    pinMode(pinLed, OUTPUT);
    pinMode(pinCop, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(pinButton), resetNode, RISING);

    ydleObj = this;
}

// Initialisation des IO avec les valeurs par défaut

ydle::ydle() {
    m_bLnkSignalReceived = false;
    m_initializedState = false;
    _callback_set = false;
    readEEProm();

    pinMode(pinRx, INPUT);
    pinMode(pinTx, OUTPUT);
    pinMode(pinButton, INPUT);
    pinMode(pinLed, OUTPUT);
    pinMode(pinCop, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(pinButton), resetNode, RISING);

    ydleObj = this;
}

// Launch the timer for the receive function

void ydle::init_timer() {
    Timer1.initialize(YDLE_FBIT); // set a timer of length YDLE_FBIT microseconds
    Timer1.attachInterrupt(ydle::timerInterrupt); // attach the service routine here
}

void ydle::timerInterrupt() {
    if (!transmission_on) {
        if (rx_done) {
            rx_done = false;
        }
        sample_value = digitalRead(pinRx);
        ydleObj->pll();
    }
    if (transmission_on) {
        if (tx_sample == 0) {
            if (bit_test == 0) {
                digitalWrite(pinTx, frameToSend[tx_index] & 1 << tx_bit);
                bit_test = true;
            } else {
                digitalWrite(pinTx, !(frameToSend[tx_index] & 1 << tx_bit));
                bit_test = false;
                tx_bit--;
            }
        }
        if (tx_bit < 0) {
            tx_index++;
            tx_bit = 7;
        }
        tx_sample++;
        if (tx_sample > 7) {
            tx_sample = 0;
        }
        if (tx_index > frameToSendLength && tx_sample == 0) {
            transmission_on = false;
            digitalWrite(pinTx, LOW);
            digitalWrite(pinLed, LOW);
            digitalWrite(pinCop, HIGH);
        }
    }
}

void ydle::pll() {
    sample_count++;
    // On additionne chaque sample et on incrémente le nombre du prochain sample
    if (sample_value) {
        sample_sum++;
    }
    // On vérifie s'il y a eu une transition de bit
    if (sample_value != last_sample_value) {
        // Transition, en avance si la rampe > 40, en retard si < 40
        if (pll_ramp < 80) {
            pll_ramp += 11;
        } else {
            pll_ramp += 29;
        }
        last_sample_value = sample_value;
    } else {
        // Si pas de transition, on avance la rampe de 20 (= 80/4 samples)
        pll_ramp += 20;
    }

    // On vérifie si la rampe à atteint son maximum de 80
    if (pll_ramp >= 160) {
        // On ajoute aux 16 derniers bits reçus rx_bits, MSB first
        // On stock les 16 derniers bits
        rx_bits <<= 1;
        // On vérifie la somme des samples sur la période pour savoir combien était à l'état haut
        // S'ils étaient < 2, on déclare un 0, sinon un 1;
        if (sample_sum >= 5) {
            rx_bits |= 1;
            bit_value = 1;
        } else {
            rx_bits |= 0;
            bit_value = 0;
        }
        pll_ramp -= 160; // On soustrait la taille maximale de la rampe à sa valeur actuelle
        sample_sum = 0; // On remet la somme des samples à 0 pour le prochain cycle
        sample_count = 1; // On ré-initialise le nombre de sample

        // Si l'on est dans le message, c'est ici qu'on traite les données
        if (rx_active) {
            bit_count++;
            // On récupére les bits et on les places dans des variables
            // 1 bit sur 2 avec Manchester
            if (bit_count % 2 == 1) // Les 8 premiers bits de données
            {
                if (bit_count < 16) {
                    receptor <<= 1;
                    receptor |= bit_value;
                } else if (bit_count < 32) // Les 8 bits suivants
                {
                    sender <<= 1;
                    sender |= bit_value;
                } else if (bit_count < 38) // Les 3 bits de type
                {
                    type <<= 1;
                    type |= bit_value;
                } else if (bit_count < 48) // Les 5 bits de longueur de trame
                {
                    rx_bytes_count <<= 1;
                    rx_bytes_count |= bit_value;
                } else if ((bit_count - 48) < (rx_bytes_count * 16)) // les données
                {
                    length_ok = 1;
                    m_data[(bit_count - 48) / 16] <<= 1;
                    m_data[(bit_count - 48) / 16] |= bit_value;
                }
            }

            // Quand on a reçu les 24 premiers bits, on connait la longueur de la trame
            // On vérifie alors que la longueur semble logique
            if (bit_count >= 48) {
                // Les bits 19 à 24 informent de la taille de la trame
                // On les vérifie car leur valeur ne peuvent être < à 1 et > à 30 + 1 pour le CRC
                if (rx_bytes_count < 1 || rx_bytes_count > 30) {
#ifdef _YDLE_DEBUG
                    log("error!");
#endif
                    // Mauvaise taille de message, on ré-initialise la lecture
                    rx_active = false;
                    sample_count = 1;
                    bit_count = 0;
                    length_ok = 0;
                    sender = 0;
                    receptor = 0;
                    type = 0;
                    taille = 0;
                    memset(m_data, 0, sizeof (m_data));
                    return;
                }
            }

            // On vérifie si l'on a reçu tout le message
            if ((bit_count - 48) >= (rx_bytes_count * 16) && (length_ok == 1)) {
#ifdef _YDLE_DEBUG
                log("complete");
#endif

                rx_active = false;
                g_ReceivedFrame.sender = sender;
                g_ReceivedFrame.receptor = receptor;
                g_ReceivedFrame.type = type;
                g_ReceivedFrame.taille = rx_bytes_count; // data + crc
                memcpy(g_ReceivedFrame.data, m_data, rx_bytes_count - 1); // copy data len - crc
                g_ReceivedFrame.crc = m_data[rx_bytes_count - 1];

                // May be an array ?
                if (pFrame == YDLE_MAX_FRAME) {
                    pFrame = 0;
                }
                memcpy(&g_FrameBuffer[pFrame], &g_ReceivedFrame, sizeof (Frame_t));
                pFrame++;
                frameReadyToBeRead = true;

                length_ok = 0;
                sender = 0;
                receptor = 0;
                type = 0;
                taille = 0;
                memset(m_data, 0, sizeof (m_data));
            }
        } else if (rx_bits == start_bit) // Pas dans le message, on recherche l'octet de start
        {
#ifdef _YDLE_DEBUG
            log("start");
#endif
            // Octet de start, on commence à collecter les données
            rx_active = true;
            bit_count = 0;
            rx_bytes_count = 0;
        }
    }
}

/***************************************************************************************/

// Remove EEProm values & change initialized state

void ydle::resetNode() {
    memset((void*) &m_Config, 0, sizeof (Config_t));
    ydleObj->writeEEProm();
    m_initializedState = false;
    m_bLnkSignalReceived = false;
}

// lire en mémoire EEProm du signal de référence

void ydle::readEEProm() {
    memset((void*) &m_Config, 0, sizeof (m_Config));
    eeprom_read_block((void*) &m_Config, 0, sizeof (Config_t));
    if ((m_Config.IdMaster != 0) && (m_Config.IdNode != 0)) {
#ifdef _YDLE_DEBUG
        log("Config find in eeprom");
        log("Config.IdMaster : ", m_Config.IdMaster);
        log("Config.IdNode : ", m_Config.IdNode);
#endif
        m_initializedState = true;
    } else {
#ifdef _YDLE_DEBUG
        log("No config in EEprom");
#endif
        memset((void*) &m_Config, 0, sizeof (m_Config));
        m_initializedState = false;
    }
}

// Ecriture en mémoire EEProm du signal de référence

void ydle::writeEEProm() {
    eeprom_write_block((void*) &m_Config, 0, sizeof (m_Config));
#ifdef _YDLE_DEBUG
    log("Enregistrement du signal reçu comme signal de référence");
#endif
}

// Function to attach a user defined function for handle received frame

void ydle::attach(ydleCallbackFunction function) {
    callback = function;
    _callback_set = true;
}

/***************************************************************************************/

// Synchronise l'AGC, envoie l'octet de start puis transmet la trame

uint8_t ydle::send(Frame_t *frame) {
    int i = 0, j = 0;

    digitalWrite(pinLed, HIGH); // on allume la Led pour indiquer une émission

    if (frame->type == YDLE_TYPE_STATE_ACK) {
        if (wait_ack != 1) {
            memcpy(&g_SendFrameBuffer, frame, sizeof (Frame_t));
            wait_ack = 1;
            last_check = millis();
        }
    }

    // From now, we are ready to transmit the frame
    while (rx_active) {
        // Wait that the current transmission finish
        delay(2 * YDLE_TPER);
#ifdef _YDLE_DEBUG
        log("The ligne is occuped");
#endif
    }
    memset((void*) &frameToSend, 0x0, 40);
    if (frame->crc == 0) {
        // add crc BYTE
        frame->taille++;
        // calcul crc
        frame->crc = computeCrc(frame);
    }
#ifdef _YDLE_DEBUG
    printFrame(frame);
#endif

    uint8_t index = 0;
    frameToSendLength = 7 + frame->taille;
    frameToSend[index++] = 0xFF;
    frameToSend[index++] = 0xFF;
    frameToSend[index++] = 0xFF;
    frameToSend[index++] = 0xFF;
    frameToSend[index++] = start_bit2;
    frameToSend[index++] = frame->receptor;
    frameToSend[index++] = frame->sender;
    frameToSend[index++] = (frame->type << 5) + frame->taille;
    for (int j = 0; j < frame->taille - 1; j++) {
        frameToSend[index++] = frame->data[j];
    }
    frameToSend[index++] = frame->crc;

    tx_index = 0;
    tx_bit = 7;

    digitalWrite(pinLed, LOW);
    transmission_on = true;
}

// New function need to be called by the main function in order to handle the new received frame

uint8_t ydle::receive() {
    unsigned char crc_p;

    if (frameReadyToBeRead) {
        for (int i = 0; i < (int) pFrame; i++) {
            crc_p = computeCrc(&g_FrameBuffer[i]);
            if (crc_p != g_FrameBuffer[i].crc) {
#ifdef _YDLE_DEBUG
                log("crc error!!!!!!!!!");
                printFrame(&g_FrameBuffer[i]);
                log("crc error!!!!!!!!!");
#endif // _YDLE_DEBUG
            } else {
#ifdef _YDLE_DEBUG
                log("Frame ready to be handled");
#endif // _YDLE_DEBUG
                // We receive a CMD so trait it
                if (g_FrameBuffer[i].type == YDLE_TYPE_CMD) {
#ifdef _YDLE_DEBUG
                    printFrame(&g_FrameBuffer[i]);
#endif // _YDLE_DEBUG
                    onCommandReceived(&g_FrameBuffer[i]);
                } else if (g_FrameBuffer[i].type == YDLE_TYPE_ACK) {
                    if (g_FrameBuffer[i].sender == g_SendFrameBuffer.receptor
                            && g_FrameBuffer[i].receptor == g_SendFrameBuffer.sender) {
#ifdef _YDLE_DEBUG
                        log("ACK received");
#endif // _YDLE_DEBUG
                        memcpy(&g_SendFrameBuffer, 0, sizeof (Frame_t));
                        wait_ack = 0;
                        retry = 0;
                        last_check = 0;
                    }
                } else {
#ifdef _YDLE_DEBUG
                    printFrame(&g_FrameBuffer[i]);
#endif
                    // Send the frame to the callback function
                    if (_callback_set)
                        callback(&g_FrameBuffer[i]);
                }
                // Frame handled
            }
        }
        // Peut poser un problème si une interruption se produit et
        // que la pll termine de traiter un paquet et la pose sur la pile exactement à ce moment là
        pFrame = 0;
        frameReadyToBeRead = false;
    }
    if (wait_ack == 1) {
        if (retry <= 3) {
            uint8_t curt = millis();
            if (curt - last_check >= YDLE_ACK_TIMEOUT) {
                send(&g_SendFrameBuffer);
#ifdef _YDLE_DEBUG
                log("Timeout, resending frame");
#endif
                last_check = curt;
            }
            retry++;
        } else {
            // Lost packet... sorry dude !
            wait_ack = 0;
            retry = 0;
            last_check = 0;
#ifdef _YDLE_DEBUG
            log("Lost packet... sorry dude !");
#endif
            return 0;
        }
    }

}

// Do something with a received Command

void ydle::onCommandReceived(Frame_t *frame) {
    int cmd;
    long value;
    int n = 0;

    // TODO: Need a better method to get the data
    while (extractData(frame, n, cmd, value) == 1) {
#ifdef _YDLE_DEBUG
        log("CMD Received : ", cmd);
#endif
        //A node ask to link us & we are not already linked
        if (cmd == YDLE_CMD_LINK && !m_initializedState) {
#ifdef _YDLE_DEBUG
            log("Link received");
#endif
            m_Config.IdNode = frame->receptor;
            m_Config.IdMaster = frame->sender;
            m_initializedState = true;
            writeEEProm();
        } else if (cmd == YDLE_CMD_ON && checkSignal(frame)) {
#ifdef _YDLE_DEBUG
            log("ON");
#endif
        } else if (cmd == YDLE_CMD_OFF && checkSignal(frame)) {
#ifdef _YDLE_DEBUG
            log("OFF");
#endif
        } else if (cmd == YDLE_CMD_RESET && checkSignal(frame)) {
#ifdef _YDLE_DEBUG
            log("Reset received");
#endif
            resetNode();
        } else if (cmd == YDLE_CMD_SET && checkSignal(frame)) {
#ifdef _YDLE_DEBUG
            log("SET");
#endif
        } else if (cmd == YDLE_CMD_GET && checkSignal(frame)) {
#ifdef _YDLE_DEBUG
            log("GET");
#endif
        } else if (cmd == YDLE_CMD_PING && checkSignal(frame)) {
#ifdef _YDLE_DEBUG
            log("PING");
#endif
        } else if (_callback_set) {
            callback(frame);
        }

        // send ACK if frame is for us.
        if (checkSignal(frame)) {
#ifdef _YDLE_DEBUG
            log("************** Send ACK ********************");
#endif
            memset(&g_SendFrame, 0x0, sizeof (Frame_t));
            initFrame(&g_SendFrame, YDLE_TYPE_ACK); // Create a new ACK Frame
            send(&g_SendFrame);
#ifdef _YDLE_DEBUG
            log("************** End ACK *********************");
#endif
        }
        n++;
    }
}

/***************************************************************************************/

// Comparaison du signal reçu et du signal de référence

bool ydle::checkSignal(Frame_t *frame) {
    if (frame->sender == m_Config.IdMaster && frame->receptor == m_Config.IdNode)
        return true;
    else
        return false;
}

// Fonction qui crée une trame avec les infos fournies

void ydle::initFrame(Frame_t *frame, unsigned long destination, unsigned long sender, unsigned long type) {
    frame->sender = sender;
    frame->receptor = destination;
    frame->type = type;
    frame->taille = 0;
    frame->crc = 0;
    memset(frame->data, 0, sizeof (frame->data));
}

// Fonction qui crée une trame avec un type fournie

void ydle::initFrame(Frame_t *frame, unsigned long type) {
    initFrame(frame, m_Config.IdMaster, m_Config.IdNode, type);
}

/***************************************************************************************/

// extract any type of data from receivedsignal

int ydle::extractData(Frame_t *frame, int n, int &itype, long &value) {
    uint8_t *ptr = frame->data;
    int ltype;
    int lsigne;
    int i = 0;
    int iModifType = 0;
    int iNbByteRest;
    bool bifValueisNegativ = false;

    // Min 1 byte of data with the 1 bytes CRC always present, else there is no data
    if (frame->taille < 2) {
        return -1;
    }

    iNbByteRest = frame->taille - 1;
    do {
        itype = *ptr >> 4;
        ltype = *ptr >> 1 & 0x7;
        lsigne = *ptr >> 0 & 0x1;

        bifValueisNegativ = false;

        // Cmd type if always 12 bits signed value
        if (frame->type == YDLE_TYPE_CMD) {
            iModifType = YDLE_DATA_UINT8;
        } else {
            iModifType = itype;
        }

        switch (iModifType) {
                // 1 bit
            case YDLE_DATA_BOOL:
                value = lsigne;
                ptr++;
                iNbByteRest--;
                break;

                // 8 bits
            case YDLE_DATA_UINT8:
                if (lsigne) {
                    if (*ptr & 0x8)
                        bifValueisNegativ = true;
                    value = (*ptr & 0x07) << 8;
                } else {
                    value = (*ptr) << 8;
                }
                ptr++;
                value += *ptr;
                ptr++;
                if (bifValueisNegativ)
                    value = value * (-1);
                iNbByteRest -= 2;
                break;

                // 16 bits
            case YDLE_DATA_UINT16:
                if (lsigne) {
                    if (*ptr & 0x8)
                        bifValueisNegativ = true;
                    value = (*ptr & 0x07) << 16;
                } else {
                    value = (*ptr) << 16;
                }
                ptr++;
                value += (*ptr) << 8;
                ptr++;
                value += *ptr;
                ptr++;
                if (lsigne)
                    value = value * (-1);
                iNbByteRest -= 3;
                break;

                // 24 bits
            case YDLE_DATA_UINT24:
                if (lsigne) {
                    if (*ptr & 0x8)
                        bifValueisNegativ = true;
                    value = (*ptr & 0x07) << 24;
                } else {
                    value = (*ptr) << 24;
                }
                ptr++;
                value += (*ptr) << 16;
                ptr++;
                value += (*ptr) << 8;
                ptr++;
                value += *ptr;
                ptr++;
                if (lsigne)
                    value = value * (-1);
                iNbByteRest -= 4;
                break;
        }
        if (n == i) {
            return 1;
        }
        i++;
    } while (iNbByteRest > 0);
    return 0;
}

/***************************************************************************************/

// add TYPE_CMD data

void ydle::addCmd(Frame_t *frame, int type, int data) {
    if (frame->taille < 28) {
        frame->data[frame->taille] = type << 4;
        frame->data[frame->taille + 1] = data;
        frame->taille += 2;
    }#ifdef _YDLE_DEBUG
    else
        log("data full for a CMD");
#endif // _YDLE_DEBUG
}

// Ajout d'une valeur bool

void ydle::addData(Frame_t *frame, int type, bool data) {
    log("addData bool");
    if (frame->taille < 29) {
        frame->data[frame->taille] = type << 4;
        frame->data[frame->taille] += YDLE_DATA_BOOL << 1;
        frame->data[frame->taille] += data & 0x0F;
        frame->taille++;
    }#ifdef _YDLE_DEBUG
    else
        log("data full for a boolean");
#endif // _YDLE_DEBUG
}

// Ajout d'une valeur int

void ydle::addData(Frame_t *frame, int type, int data) {
    // 8 bits int
    if (data > -256 && data < 256) {
        log("addData int 8");
        if (frame->taille < 28) {
            frame->data[frame->taille] = type << 4;
            frame->data[frame->taille] += YDLE_DATA_UINT8 << 1;
            frame->data[frame->taille] += (data < 0 ? 1 : 0) << 0;
            frame->data[frame->taille + 1] = data;
            frame->taille += 2;
        }#ifdef _YDLE_DEBUG
        else
            log("data full for a 8 bits int");
#endif // _YDLE_DEBUG
    }// 16 bits int
    else if (data > -65536 && data < 65536) {
        log("addData int 16");
        if (frame->taille < 27) {
            frame->data[frame->taille] = type << 4;
            frame->data[frame->taille] += YDLE_DATA_UINT8 << 1;
            frame->data[frame->taille] += (data < 0 ? 1 : 0) << 0;
            frame->data[frame->taille + 1] = (data >> 8) & 0xFF;
            frame->data[frame->taille + 2] = data;
            frame->taille += 3;
        }#ifdef _YDLE_DEBUG
        else
            log("data full for a 16 bits int");
#endif // _YDLE_DEBUG
    }
}

// Ajout d'une valeur long int

void ydle::addData(Frame_t *frame, int type, long int data) {
    // 24 bits int
    if (data > -16777216 && data < 16777216) {
        log("addData long int 24");
        if (frame->taille < 26) {
            frame->data[frame->taille] = type << 4;
            frame->data[frame->taille] += YDLE_DATA_UINT8 << 1;
            frame->data[frame->taille] += (data < 0 ? 1 : 0) << 0;
            frame->data[frame->taille + 1] = (data >> 16);
            frame->data[frame->taille + 2] = (data >> 8);
            frame->data[frame->taille + 3] = data;
            frame->taille += 4;
        }#ifdef _YDLE_DEBUG
        else
            log("data full for a 24 bits int");
#endif // _YDLE_DEBUG
    }// 32 bits int
    else if (data > -4294967296 && data < 4294967296) {
        log("addData long int 32");
        if (frame->taille < 25) {
            frame->data[frame->taille] = type << 4;
            frame->data[frame->taille] += YDLE_DATA_UINT8 << 1;
            frame->data[frame->taille] += (data < 0 ? 1 : 0) << 0;
            frame->data[frame->taille + 1] = (data >> 24) & 0xFF;
            frame->data[frame->taille + 2] = (data >> 16) & 0xFF;
            frame->data[frame->taille + 3] = (data >> 8) & 0xFF;
            frame->data[frame->taille + 4] = data;
            frame->taille += 5;
        }#ifdef _YDLE_DEBUG
        else
            log("data full for a 32 bits int");
#endif // _YDLE_DEBUG
    }
}

// Fonction qui retourne "true" si la Node est initialisée

bool ydle::initialized() {
    return m_initializedState;
}

/***************************************************************************************/

// The inital and final constants as used in the ATM HEC.

uint8_t ydle::crc8(const uint8_t* buf, uint8_t length) {
    const uint8_t initial = 0x00;
    const uint8_t final = 0x55;
    uint8_t crc = initial;
    while (length) {
        crc = pgm_read_byte_near(_atm_crc8_table + (*buf ^ crc));
        buf++;
        length--;
    }
    return crc ^ final;
}

// CRC calculation

unsigned char ydle::computeCrc(Frame_t *frame) {
    unsigned char *buf, crc;
    int a, j;

    buf = (unsigned char*) malloc(frame->taille + 3);
    memset(buf, 0x0, frame->taille + 3);

    buf[0] = frame->sender;
    buf[1] = frame->receptor;
    buf[2] = frame->type;
    buf[2] = buf[2] << 5;
    buf[2] |= frame->taille;

    for (a = 3, j = 0; j < frame->taille - 1; a++, j++) {
        buf[a] = frame->data[j];
    }

    crc = crc8(buf, frame->taille + 2);
    free(buf);
    return crc;
}

/***************************************************************************************/

// Affiche les logs sur la console série

void ydle::log(String msg) {
#if not defined( __AVR_ATtiny85__ ) or defined(_YDLE_DEBUG)
    Serial.println(msg);
#endif
}

// Affiche les logs sur la console série

void ydle::log(String msg, int i) {
#if not defined( __AVR_ATtiny85__ ) or defined(_YDLE_DEBUG)
    Serial.print(msg);
    Serial.println(i);
#endif
}

// Affiche le contenue d'une trame reçue

void ydle::printFrame(Frame_t *trame) {
#if not defined( __AVR_ATtiny85__ ) or defined (_YDLE_DEBUG)
    char sztmp[255];

    log("-----------------------------------------------");
    sprintf(sztmp, "Emetteur : %d", trame->sender);
    log(sztmp);

    sprintf(sztmp, "Recepteur : %d", trame->receptor);
    log(sztmp);

    sprintf(sztmp, "Type : %d", trame->type);
    log(sztmp);

    sprintf(sztmp, "Taille : %d", trame->taille);
    log(sztmp);

    sprintf(sztmp, "CRC : %d", trame->crc);
    log(sztmp);

    sprintf(sztmp, "Data Hex: ");
    for (int a = 0; a < trame->taille - 1; a++)
        sprintf(sztmp, "%s 0x%02X", sztmp, trame->data[a]);
    log(sztmp);

    sprintf(sztmp, "Data Dec: ");
    for (int a = 0; a < trame->taille - 1; a++)
        sprintf(sztmp, "%s %d", sztmp, trame->data[a]);
    log(sztmp);
    log("-----------------------------------------------");
#endif
}