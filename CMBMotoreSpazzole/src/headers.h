#define LOG_BAUD 115200
#define UNDEFINED_VALUE -123456

// pins
#define I_MOT 34
#define H1 27
#define H2 26
#define ADC_BATT 17
#define IN1 33
#define IN2 32

#define PULSANTE 12
#define POTENZIOMETRO 14

// costanti motore
#define PWM_FREQ 25000
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_RESOLUTION 8
#define POLES 11

// default parametri impostabili
#define DEFAULT_RAIL_LENGTH_RAD 120                               // rad
#define DEFAULT_RAIL_LENGTH_PULSES DEFAULT_RAIL_LENGTH_RAD *POLES // rad
#define DEFAULT_RAIL_START 0.02                                   //% RAIL_LENGTH
#define DEFAULT_RAIL_END 0.98                                     //% RAIL_LENGTH
#define DEFAULT_RAD_OPEN 18                                       // rad/s
#define DEFAULT_RAD_CLOSE -7                                     // rad/s
#define DEFAULT_TIMEOUT 2000                                      // ms
#define DEFAULT_OPEN_TIMEOUT 700                                  // ms

// stati per macchina a stati
#define STATE_START 0
#define STATE_INACTIVE 1
#define STATE_INIZIOCORSA 2
#define STATE_APERTURA 3
#define STATE_FINECORSA 4
#define STATE_CHIUSURA 5