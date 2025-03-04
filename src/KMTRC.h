#pragma once

#include "Arduino.h"
#include "Wire.h"
#include <soc/pcnt_struct.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include "esp_log.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PCA9555 defines
//
#define NXP_INPUT 0
#define NXP_OUTPUT 2
#define NXP_INVERT 4
#define NXP_CONFIG 6

#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PCA9685 defines
//
#define BUFFER_LENGTH 32

#define PCA9685_MODE_INVRT (byte)0x10        // Inverts polarity of channel output signal
#define PCA9685_MODE_OUTPUT_ONACK (byte)0x08 // Channel update happens upon ACK (post-set) rather than on STOP (endTransmission)
#define PCA9685_MODE_OUTPUT_TPOLE (byte)0x04 // Use a totem-pole (push-pull) style output, typical for boards using this chipset
#define PCA9685_MODE_OUTNE_HIGHZ (byte)0x02  // For active low output enable, sets channel output to high-impedance state
#define PCA9685_MODE_OUTNE_LOW (byte)0x01    // Similarly, sets channel output to high if in totem-pole mode, otherwise high-impedance state

#define PCA9685_MIN_CHANNEL 0
#define PCA9685_MAX_CHANNEL 15
#define PCA9685_CHANNEL_COUNT 16

typedef enum
{
    PCA9685_PhaseBalancer_None = -1,  // Disables phase balancing, all high phase areas start at begining of cycle
    PCA9685_PhaseBalancer_Linear = 0, // Balances all outputs linearly, 256 steps away from previous output
    PCA9685_PhaseBalancer_Weaved,     // Balances first few outputs better, steps away from previous shorten towards last output

    PCA9685_PhaseBalancer_Count
} PCA9685_PhaseBalancer;

// #define PCA9685_ENABLE_DEBUG_OUTPUT
#define PCA9685_I2C_BASE_ADDRESS (byte)0x40

// Register addresses from data sheet
#define PCA9685_MODE1_REG (byte)0x00
#define PCA9685_MODE2_REG (byte)0x01
#define PCA9685_SUBADR1_REG (byte)0x02
#define PCA9685_SUBADR2_REG (byte)0x03
#define PCA9685_SUBADR3_REG (byte)0x04
#define PCA9685_ALLCALL_REG (byte)0x05
#define PCA9685_LED0_REG (byte)0x06 // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG (byte)0xFE
#define PCA9685_ALLLED_REG (byte)0xFA

// Mode1 register pin layout
#define PCA9685_MODE_RESTART (byte)0x80
#define PCA9685_MODE_EXTCLK (byte)0x40
#define PCA9685_MODE_AUTOINC (byte)0x20
#define PCA9685_MODE_SLEEP (byte)0x10
#define PCA9685_MODE_SUBADR1 (byte)0x08
#define PCA9685_MODE_SUBADR2 (byte)0x04
#define PCA9685_MODE_SUBADR3 (byte)0x02
#define PCA9685_MODE_ALLCALL (byte)0x01

#define PCA9685_SW_RESET (byte)0x06        // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_PWM_FULL (uint16_t)0x01000 // Special value for full on/full off LEDx modes

// To balance the load out in a weaved fashion, we use this offset table to distribute
// the load on the outputs in a more interleaving fashion than just a simple 16 offset
// per channel. We can set the off cycle value to be lower than the on cycle, which will
// put the high edge across the 0-4095 phase cycle range, which is supported by device.
static uint16_t phaseDistTable[16] = {0, 2048, 1024, 3072, 512, 3584, 1536, 2560, 256, 3840, 1280, 2304, 3328, 768, 2816, 1792};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ultrasonic defines
//
/*
 * Values of divisors
 */
#define CM 29
#define INC 71
#define USEHOPIN 25
#define USTRIGPIN 23
#define US2EHOPIN 27
#define US2TRIGPIN 26

#define WIFI
//#define PCA9685_ENABLE_DEBUG_OUTPUT
#define MOTOR_MAX 100
#define MOTOR_MIN -100

const uint8_t FORWARD = 1;
const uint8_t BACKWARD = 2;
const uint8_t BRAKE = 3;
const uint8_t RELEASE = 4;

// definitions of KMTRC pins for use arduino funtions
const int DIO1 = 12;
const int DIO2 = 13;
const int DIO3 = 14;

const int ADC1 = 35;
const int ADC2 = 34;
const int ADC3 = 33;

#define STEPS 200

enum STEP_TYPE
{
    WAVE_DRIVE,
    FULL_STEP,
    HALF_STEP
};

class KMTRC
{
    friend class KMTRC_PCA9555;
    friend class PCA9685;

private:
    uint8_t _addr;
    uint16_t _freq;

    uint8_t PWMpin;
    uint8_t IN1pin;
    uint8_t IN2pin;

    void runMotor(uint8_t num, uint8_t cmd);
    void setMotorSpeed(uint8_t num, uint8_t speed);

    void setPWM(uint8_t pin, uint16_t value);
    void setPin(uint8_t pin, bool value);

public:
    KMTRC(/* args */);
    ~KMTRC();

    void begin(void);

    //------------------------------------------------DC_Motors-------------------------------------------------------------------------------------------------

    /**
     * @brief Функция задания скорости колёс робота
     * @p motor - номер колеса
     * @p v - скорость колёса
     */
    void moveWheel(uint8_t motor, int v);

    void DC_Motor(uint8_t num, uint8_t speed, uint8_t action);

    void mecanumWheelsDrive(uint8_t speed, uint8_t direction);

    void mecanumWheelsDriveTime(uint8_t speed, uint8_t direction, int movetime);
    // отсчёты энкодера с каких колёс??
    void mecanumWheelsDriveEncoderTime(uint8_t speed, uint8_t direction, int movetime);
    // отсчёты энкодера с каких колёс??
    void mecanumWheelsDriveEncoder(uint8_t speed, uint8_t direction, int encnumber);

    void mecanumMoveForward(uint8_t speed);
    void mecanumMoveBackward(uint8_t speed);
    void mecanumrotateRight(uint8_t speed);
    void mecanumrotateLeft(uint8_t speed);
    void mecanumMoveSidewaysRight(uint8_t speed);
    void mecanumMoveSidewaysLeft(uint8_t speed);
    void mecanumMoveRightForward(uint8_t speed);
    void mecanumMoveLeftForward(uint8_t speed);
    void mecanumStopMoving(uint8_t speed);

    //-------------------------------------------------Servo-------------------------------------------------------------------------------------------------

    /**
     * @brief Функция установки угла поворота сервопривода
     * @p num - номер сервопривода
     * @p angle - угол поворота сервопривода
     */
    void setServoAngle(uint8_t num, uint8_t angle);

    //----------------------------------------------Analog Servos-------------------------------------------------------------------------------------------------

    /**
     * @brief Функция установки угла поворота аналогово сервопривода
     * @p num - номер сервопривода
     * @p angle - угол поворота сервопривода
     */
    void setAnalogServoAngle(uint8_t num, uint8_t angle);

    //------------------------------------------------Steppers-------------------------------------------------------------------------------------------------

    // void setStepper(int stepper_num, int stepper_speed, int steps);

    void setStepperSpeed(int stepper_num, int stepper_speed);

    void setStepperStep(int stepper_num, int steps);

    //------------------------------------------------Encoder-------------------------------------------------------------------------------------------------

    void encoderSetup(uint8_t num);
    long encPosition(uint8_t num);
    void clearEncoderData(uint8_t num);

    //-------------------------------------------------Outher-------------------------------------------------------------------------------------------------

    int ADCRead(uint8_t pin);
    int DIORead(uint8_t pin);

    void setLedDIO4(uint8_t freq);

    bool ReadButton(uint8_t num);

    void setBuzzer(uint32_t freq, long duration);

    unsigned int distUltrasonic1(void);
    unsigned int distUltrasonic2(void);

    void readLineSensor(uint8_t port);

    void readAccelGyroSensor(uint8_t port);

    void readBattreyLevel();
};

class KMTRC_Stepper
{
    // friend class KMTRC;

private:
    // void stepMotor(int this_step);
    void stepMotorALT(int thisStep);

    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in us, based on speed
    int number_of_steps;      // total number of steps this motor can take
    int step_number;          // which step the motor is on

    // motor pin numbers:
    int motor_pin_1;
    int motor_pin_2;
    int motor_pin_3;
    int motor_pin_4;

    unsigned long last_step_time; // timestamp in us of when the last step was taken

public:
    KMTRC_Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                           int motor_pin_3, int motor_pin_4);
    ~KMTRC_Stepper();

    //------------------------------------------------Steppers-------------------------------------------------------------------------------------------------

    // speed setter method:
    void setStepperSpeed(long whatSpeed);

    // mover method:
    void stepperStep(int number_of_steps);
};

class KMTRC_PCA9555
{
    // объявление дружественного класса
    friend class KMTRC;

public:
    KMTRC_PCA9555(uint8_t address, int interruptPin = -1); // optional interrupt pin in second argument
    void pinMode(uint8_t pin, uint8_t IOMode);                      // pinMode
    uint8_t digitalRead(uint8_t pin);                               // digitalRead
    void digitalWrite(uint8_t pin, uint8_t value);                  // digitalWrite
    uint8_t stateOfPin(uint8_t pin);                                // Actual ISR
    bool begin();                                                   // Checks if PCA is responsive

private:
    //
    // low level methods
    //
    uint16_t I2CGetValue(uint8_t address, uint8_t reg);
    void I2CSetValue(uint8_t address, uint8_t reg, uint8_t value);

    union
    {
        struct
        {
            uint8_t _stateOfPins_low;  // low order byte
            uint8_t _stateOfPins_high; // high order byte
        };
        uint16_t _stateOfPins; // 16 bits presentation
    };
    union
    {
        struct
        {
            uint8_t _configurationRegister_low;  // low order byte
            uint8_t _configurationRegister_high; // high order byte
        };
        uint16_t _configurationRegister; // 16 bits presentation
    };
    union
    {
        struct
        {
            uint8_t _valueRegister_low;  // low order byte
            uint8_t _valueRegister_high; // high order byte
        };
        uint16_t _valueRegister;
    };
    uint8_t _address; // address of port this class is supporting
    int _error;       // error code from I2C
};

class PCA9685
{
    // объявление дружественного класса
    friend class KMTRC;

public:
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
    // May use a different Wire instance than Wire. Some chipsets, such as Due/Zero/etc.,
    // have a Wire1 class instance that uses the SDA1/SCL1 lines instead.
    // Supported i2c baud rates are 100kHz, 400kHz, and 1000kHz.
    PCA9685(TwoWire &i2cWire = Wire, PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_Linear);
#else
    // Minimum supported i2c baud rate is 100kHz, which means minimum supported processor
    // speed is 4MHz+ while running i2c standard mode. For 400kHz i2c baud rate, minimum
    // supported processor speed is 16MHz+ while running i2c fast mode.
    PCA9685(PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_Linear);
#endif

    // Should be called only once in setup(), before any init()'s, but after Wire.begin().
    // Only should be called once on any Wire instance to do a software reset, which
    // will affect all devices on that line. This helps when you're constantly rebuilding
    // and reuploading to ensure all the devices on that line are reset properly.
    void resetDevices();

    // Called in setup(). The i2c address here is the value of the A0, A1, A2, A3, A4 and
    // A5 pins ONLY, as the class takes care of its internal base address. i2cAddress
    // should be a value between 0 and 61, since only 62 boards can be addressed.
    void init(byte i2cAddress = 0, byte mode = PCA9685_MODE_OUTPUT_ONACK | PCA9685_MODE_OUTPUT_TPOLE);

#ifndef PCA9685_EXCLUDE_EXT_FUNC
    // Called in setup(). Used when instance talks through to AllCall/Sub1-Sub3 instances
    // as a proxy object. Using this method will disable any method that performs a read
    // or conflicts certain states.
    void initAsProxyAddresser(byte i2cAddress = 0xE0);
#endif

    byte getI2CAddress();
    PCA9685_PhaseBalancer getPhaseBalancer();

    // Min: 24Hz, Max: 1526Hz, Default: 200Hz (resolution widens as Hz goes higher)
    void setPWMFrequency(float pwmFrequency);

    // Turns channel either full on or full off
    void setChannelOn(int channel);
    void setChannelOff(int channel);

    // PWM amounts 0 - 4096, 0 full off, 4096 full on
    void setChannelPWM(int channel, uint16_t pwmAmount);
    void setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts);

#ifndef PCA9685_EXCLUDE_EXT_FUNC
    // Sets all channels, but won't distribute phases
    void setAllChannelsPWM(uint16_t pwmAmount);

    // Returns PWM amounts 0 - 4096, 0 full off, 4096 full on
    uint16_t getChannelPWM(int channel);

    // Enables multiple talk-through paths via i2c bus (lsb/bit0 must stay 0)
    // To use, create a new class instance using initAsSubAddressed() with said address
    void enableAllCallAddress(byte i2cAddress = 0xE0);
    void enableSub1Address(byte i2cAddress = 0xE2);
    void enableSub2Address(byte i2cAddress = 0xE4);
    void enableSub3Address(byte i2cAddress = 0xE8);
    void disableAllCallAddress();
    void disableSub1Address();
    void disableSub2Address();
    void disableSub3Address();

    // Allows external clock line to be utilized (once enabled cannot be disabled)
    void enableExtClockLine();
#endif

    byte getLastI2CError();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    void printModuleInfo();
    void checkForErrors();
#endif

private:
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
    TwoWire *_i2cWire; // Wire class instance to use
#endif
    byte _i2cAddress;                     // Module's i2c address
    PCA9685_PhaseBalancer _phaseBalancer; // Phase balancer scheme to distribute load
    bool _isProxyAddresser;               // Instance is a proxy for sub addressing (disables certain functionality)
    byte _lastI2CError;                   // Last i2c error

    void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);

    void writeChannelBegin(int channel);
    void writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
    void writeChannelEnd();

    void writeRegister(byte regAddress, byte value);
    byte readRegister(byte regAddress);

#ifdef PCA9685_ENABLE_SOFTWARE_I2C
    uint8_t _readBytes;
#endif
    void i2cWire_beginTransmission(uint8_t);
    uint8_t i2cWire_endTransmission(void);
    uint8_t i2cWire_requestFrom(uint8_t, uint8_t);
    size_t i2cWire_write(uint8_t);
    uint8_t i2cWire_read(void);
};

// #ifndef PCA9685_EXCLUDE_SERVO_EVAL

// Class to assist with calculating Servo PWM values from angle values
class PCA9685_ServoEvaluator
{
    // объявление дружественного класса
    friend class KMTRC;

public:
    // Uses a linear interpolation method to quickly compute PWM output value. Uses
    // default values of 2.5% and 12.5% of phase length for -90/+90.
    PCA9685_ServoEvaluator(uint16_t n90PWMAmount = 102, uint16_t p90PWMAmount = 512);

    // Uses a cubic spline to interpolate due to an offsetted zero angle that isn't
    // exactly between -90/+90. This takes more time to compute, but gives a more
    // accurate PWM output value along the entire range.
    PCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount);

    ~PCA9685_ServoEvaluator();

    // Returns the PWM value to use given the angle (-90 to +90)
    uint16_t pwmForAngle(float angle);

private:
    float *_coeff;   // a,b,c,d coefficient values
    bool _isCSpline; // Cubic spline tracking, for _coeff length
};

class KMTRC_Ultrasonic
{
    friend class KMTRC;

public:
    KMTRC_Ultrasonic(uint8_t sigPin) : KMTRC_Ultrasonic(sigPin, sigPin) {};
    KMTRC_Ultrasonic(uint8_t trigPin, uint8_t echoPin, unsigned long timeOut = 20000UL);
    unsigned int read(uint8_t und = CM);
    void setTimeout(unsigned long timeOut) { timeout = timeOut; }
    void setMaxDistance(unsigned long dist) { timeout = dist * CM * 2; }

private:
    uint8_t trig;
    uint8_t echo;
    bool threePins = false;
    unsigned long previousMicros;
    unsigned long timeout;
    unsigned int timing();
};

#define MAX_ESP32_ENCODERS PCNT_UNIT_MAX
#define _INT16_MAX 32766
#define _INT16_MIN -32766

enum encType
{
    single,
    half,
    full
};

enum puType
{
    UP,
    DOWN,
    NONE
};

class ESP32Encoder;

typedef void (*enc_isr_cb_t)(void *);

class ESP32Encoder
{
    friend class KMTRC;

public:
    /**
     * @brief Construct a new ESP32Encoder object
     *
     * @param always_interrupt set to true to enable interrupt on every encoder pulse, otherwise false
     * @param enc_isr_cb callback executed on every encoder ISR, gets a pointer to
     * 	the ESP32Encoder instance as an argument, no effect if always_interrupt is
     * 	false
     */
    ESP32Encoder(bool always_interrupt = false, enc_isr_cb_t enc_isr_cb = nullptr, void *enc_isr_cb_data = nullptr);
    ~ESP32Encoder();
    void attachHalfQuad(int aPintNumber, int bPinNumber);
    void attachFullQuad(int aPintNumber, int bPinNumber);
    void attachSingleEdge(int aPintNumber, int bPinNumber);
    int64_t getCount();
    int64_t clearCount();
    int64_t pauseCount();
    int64_t resumeCount();
    void detatch();
    boolean isAttached() { return attached; }
    void setCount(int64_t value);
    void setFilter(uint16_t value);
    static ESP32Encoder *encoders[MAX_ESP32_ENCODERS];

    bool always_interrupt;
    gpio_num_t aPinNumber;
    gpio_num_t bPinNumber;
    pcnt_unit_t unit;
    bool fullQuad = false;
    int countsMode = 2;
    volatile int64_t count = 0;
    pcnt_config_t r_enc_config;
    static enum puType useInternalWeakPullResistors;
    enc_isr_cb_t _enc_isr_cb;
    void *_enc_isr_cb_data;

private:
    static pcnt_isr_handle_t user_isr_handle;
    static bool attachedInterrupt;
    void attach(int aPintNumber, int bPinNumber, enum encType et);
    int64_t getCountRaw();
    bool attached;
    bool direction;
    bool working;
};