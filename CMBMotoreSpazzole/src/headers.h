#define LOG_BAUD 115200
#define UNDEFINED_VALUE -123456

#define CONFIG_NAMESPACE "config"

// pins
#define I_MOT 34
#define H1 27
#define H2 26
#define ADC_BATT 35
#define IN1 33
#define IN2 32

#define PULSANTE 12
#define POTENZIOMETRO 14

#define BUTTON_LONGPRESS_TIME 2000

// costanti motore
#define PWM_FREQ 25000
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_RESOLUTION 8
#define POLES 21                 //encoder in quadratura:ON misura 4xPPR, quindi 12 poli pagnetici x 2 fronti.

// default parametri impostabili
#define DEFAULT_RAIL_LENGTH_RAD DEFAULT_RAIL_LENGTH_RAD / POLES // rad
#define DEFAULT_RAIL_LENGTH_PULSES 1340                         // pulses
#define DEFAULT_RAIL_START 0.02                                 //% RAIL_LENGTH
#define DEFAULT_RAIL_END 0.98                                   //% RAIL_LENGTH
#define DEFAULT_RAD_OPEN 18                                     // rad/s
#define DEFAULT_RAD_CLOSE -7                                    // rad/s
#define DEFAULT_TIMEOUT 4000                                    // ms
#define DEFAULT_OPEN_TIMEOUT 700                                // ms

// default parametri PID
#define DEFAULT_KP_OPEN 0.5
#define DEFAULT_KI_OPEN 5.0
#define DEFAULT_KD_OPEN 0.0

#define DEFAULT_KP_CLOSE 0.2
#define DEFAULT_KI_CLOSE 2.5
#define DEFAULT_KD_CLOSE 0.0

// stati per macchina a stati
#define STATE_START 0
#define STATE_INACTIVE 1
#define STATE_INIZIOCORSA 2
#define STATE_APERTURA 3
#define STATE_FINECORSA 4
#define STATE_CHIUSURA 5
#define STATE_CONFIGURAZIONE 6
#define STATE_DELAY 7