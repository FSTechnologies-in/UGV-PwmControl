// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h
int PWM_MIN 60 // Minimum pwm
int PWMRANGE 120 //maximum pwm
uint16_t frequency_right = 380; //frequency (in Hz)
uint16_t frequency_left = 400; //frequency (in Hz)

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
float mapPwm(float x, float out_min, float out_max);// Mapping pwm function


/*
Setup all GPIOs 
GPIO2------> Right Direction Relay
GPIO4------> Left Direction Relay
GPIO9-------> Right pwm
GPIO3-------> Left pwm
GPIO5 ------> Limit switch
*/
const uint8_t R_PWM = 9;//D3
const uint8_t L_PWM = 3;//D9
const uint8_t right_relay = 2;// D2 RIGHT MOTOR
const uint8_t left_relay = 4;// D4 RIGHT  MOTOR


/* Left pwm and right pwm variable to access global */
uint16_t lPwm;
uint16_t rPwm;

long sig_started=0;
long pwm_interval=50;
/* Industrial pc publish linear and angular value store in below variable to access through out program*/
float linear_velocity_ref;
float angular_velocity_ref;

uint8_t speed_right=25,speed_left=25; // speed increase variable
typedef enum {ZERO,ONE,TWO,THREE,FOUR,FIVE}number;
