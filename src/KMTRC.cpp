#include "KMTRC.h"

PCA9685 pwmControl;
KMTRC_PCA9555 io_expand(0x20);

// create an instance of the Stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
KMTRC_Stepper stepper1(STEPS, 13, 12, 14, 15); // stepper phases for our nema17 steppermotor and new move function
KMTRC_Stepper stepper2(STEPS, 9, 8, 11, 10);   // stepper phases for our nema17 steppermotor and new move function

ESP32Encoder encoder;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

KMTRC_Ultrasonic uSonic1(USTRIGPIN, USEHOPIN);
KMTRC_Ultrasonic uSonic2(US2TRIGPIN, US2EHOPIN);

KMTRC::KMTRC(/* args */)
{
  // Может тут всё и инициализировать из begin() ???
}

KMTRC::~KMTRC()
{
}

void KMTRC::begin(void)
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.println("Serial start");
#endif

  bool status;

  // Старт I2C и расширителя портов
  status = io_expand.begin();
  Serial.println(status);
  Serial.println("PCA9555 start");
  // io_expand.setClock(400000);

  for (uint8_t i = 0; i < 8; i++)
  {
    io_expand.pinMode(i, INPUT);
  }
  for (uint8_t i = 8; i < 16; i++)
  {
    io_expand.pinMode(i, OUTPUT);
  }

  for (uint8_t i = 8; i < 16; i++)
  {
    io_expand.digitalWrite(i, LOW);
  }

  pwmControl.init();
  pwmControl.setPWMFrequency(50.0f);

  for (uint8_t i = 0; i < 16; i++)
  {
    pwmControl.setChannelOn(i);
  }
}

//------------------------------------------------DC_Motors-------------------------------------------------------------------------------------------------

void KMTRC::moveWheel(uint8_t motor, int v)
{
  // Ораничение скорости движения
  if (v > 100)
    v = 100;
  if (v < -100)
    v = -100;

  if (v > 0)
  {
    DC_Motor(motor, v * 2.55, FORWARD);
  }
  else if (v < 0)
  {
    DC_Motor(motor, -v * 2.55, BACKWARD);
  }
  else
  {
    DC_Motor(motor, 0, BRAKE);
  }

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.println("Motor move");
#endif
}

void KMTRC::DC_Motor(uint8_t num, uint8_t speed, uint8_t action)
{
  setMotorSpeed(num, speed); // Задаем скорость (1-255)
  runMotor(num, action);     // Запустаем вращение вперед

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.println("Run dc motor");
#endif
}

void KMTRC::setMotorSpeed(uint8_t num, uint8_t speed)
{
  // pwmControl.setPWMFrequency(1500.0f);
  switch (num)
  {
  case 1:
    // PWMpin = 8;
    PWMpin = 11; // Для двигателя 1
    break;
  case 2:
    // PWMpin = 9;
    PWMpin = 10; // Для двигателя 2
    break;
  case 3:
    // PWMpin = 10;
    PWMpin = 9; // Для двигателя 3
    break;
  case 4:
    // PWMpin = 11;
    PWMpin = 8; // Для двигателя 4
    break;
  default:
    break;
  }

  setPWM(PWMpin, speed * 16);
  //  Serial.println(PWMpin);
}

void KMTRC::runMotor(uint8_t num, uint8_t cmd)
{
  if (num == 4)
  {
    // IN1pin = 6 + 8;
    // IN2pin = 7 + 8;
    IN1pin = 1 + 8;
    IN2pin = 0 + 8;
  }

  else if (num == 3)
  {
    // IN1pin = 4 + 8;
    // IN2pin = 5 + 8;
    IN1pin = 3 + 8;
    IN2pin = 2 + 8;
  }

  else if (num == 2)
  {
    // IN1pin = 2 + 8;
    // IN2pin = 3 + 8;
    IN1pin = 5 + 8;
    IN2pin = 4 + 8;
  }

  else if (num == 1)
  {
    // IN1pin = 0 + 8;
    // IN2pin = 1 + 8;
    IN1pin = 6 + 8;
    IN2pin = 7 + 8;
  }
  switch (cmd)
  {
  case FORWARD:
    setPin(IN2pin, LOW); // take low first to avoid 'break'
    setPin(IN1pin, HIGH);
    // Serial.println(IN2pin);
    break;
  case BACKWARD:
    setPin(IN1pin, LOW); // take low first to avoid 'break'
    setPin(IN2pin, HIGH);
    break;
  case RELEASE:
    setPin(IN2pin, LOW); // take low first to avoid 'break'
    setPin(IN1pin, LOW);
    break;
  case BRAKE:
    setPin(IN2pin, LOW); // take low first to avoid 'break'
    setPin(IN1pin, LOW);

    setPin(IN2pin, HIGH);
    setPin(IN1pin, HIGH);
    break;
  }
}

//-------------------------------------- Mecanum Wheels move functions-------------------------------------------------------------------------------------------------
void KMTRC::mecanumWheelsDrive(uint8_t speed, uint8_t direction)
{
  switch (direction)
  {

  default:
    break;
  }
}
void KMTRC::mecanumMoveForward(uint8_t speed)
{
  DC_Motor(1, speed, FORWARD);
  DC_Motor(4, speed, FORWARD);

  DC_Motor(2, speed, FORWARD);
  DC_Motor(3, speed, FORWARD);
}

void KMTRC::mecanumMoveBackward(uint8_t speed)
{
  DC_Motor(1, speed, BACKWARD);
  DC_Motor(4, speed, BACKWARD);

  DC_Motor(2, speed, BACKWARD);
  DC_Motor(3, speed, BACKWARD);
}

void KMTRC::mecanumrotateRight(uint8_t speed)
{
  DC_Motor(1, speed, BACKWARD);
  DC_Motor(4, speed, BACKWARD);

  DC_Motor(2, speed, FORWARD);
  DC_Motor(3, speed, FORWARD);
}

void KMTRC::mecanumrotateLeft(uint8_t speed)
{
  DC_Motor(1, speed, FORWARD);
  DC_Motor(4, speed, FORWARD);

  DC_Motor(2, speed, BACKWARD);
  DC_Motor(3, speed, BACKWARD);
}

void KMTRC::mecanumMoveSidewaysRight(uint8_t speed)
{
  DC_Motor(1, speed, BACKWARD);
  DC_Motor(4, speed, FORWARD);

  DC_Motor(2, speed, FORWARD);
  DC_Motor(3, speed, BACKWARD);
}

void KMTRC::mecanumMoveSidewaysLeft(uint8_t speed)
{
  DC_Motor(1, speed, FORWARD);
  DC_Motor(4, speed, BACKWARD);

  DC_Motor(2, speed, BACKWARD);
  DC_Motor(3, speed, FORWARD);
}

void KMTRC::mecanumMoveRightForward(uint8_t speed)
{
  DC_Motor(1, speed, RELEASE);
  DC_Motor(4, speed, FORWARD);

  DC_Motor(2, speed, FORWARD);
  DC_Motor(3, speed, RELEASE);
}

void KMTRC::mecanumMoveLeftForward(uint8_t speed)
{
  DC_Motor(1, speed, FORWARD);
  DC_Motor(4, speed, RELEASE);

  DC_Motor(2, speed, RELEASE);
  DC_Motor(3, speed, FORWARD);
}

void KMTRC::mecanumStopMoving(uint8_t speed)
{
  DC_Motor(1, speed, RELEASE);
  DC_Motor(4, speed, RELEASE);

  DC_Motor(2, speed, RELEASE);
  DC_Motor(3, speed, RELEASE);
}

//----------------------------------------------Analog Servos-------------------------------------------------------------------------------------------------
void KMTRC::setAnalogServoAngle(uint8_t num, uint16_t angle)
{
  PCA9685_ServoEvaluator pwmServo1(102, 512);
  PCA9685_ServoEvaluator pwmServo2(102, 512);
  PCA9685_ServoEvaluator pwmServo3(102, 512);
  PCA9685_ServoEvaluator pwmServo4(102, 512);
  PCA9685_ServoEvaluator pwmServo5(102, 512);
  PCA9685_ServoEvaluator pwmServo6(102, 512);

  switch (num)
  {
  case 1:
    pwmControl.setChannelPWM(0, pwmServo1.pwmForAngle(angle));
    break;
  case 2:
    pwmControl.setChannelPWM(1, pwmServo2.pwmForAngle(angle));
    break;
  case 3:
    pwmControl.setChannelPWM(2, pwmServo3.pwmForAngle(angle));
    break;
  case 4:
    pwmControl.setChannelPWM(3, pwmServo4.pwmForAngle(angle));
    break;
  case 5:
    pwmControl.setChannelPWM(4, pwmServo5.pwmForAngle(angle));
    break;
  case 6:
    pwmControl.setChannelPWM(5, pwmServo6.pwmForAngle(angle));
    break;
  default:
    break;
  }
}

//------------------------------------------------Steppers-------------------------------------------------------------------------------------------------

void KMTRC::setStepperSpeed(int stepper_num, int stepper_speed)
{
  if (stepper_num == 1)
  {
    stepper1.setStepperSpeed(stepper_speed);
  }
  else if (stepper_num == 2)
  {
    stepper2.setStepperSpeed(stepper_speed);
  }
}

void KMTRC::setStepperStep(int stepper_num, int steps)
{
  if (stepper_num == 1)
  {
    stepper1.stepperStep(steps);
  }
  else if (stepper_num == 2)
  {
    stepper2.stepperStep(steps);
  }
}

/*
 *   constructor for four-pin version
 *   Sets which wires should control the motor.
 */
KMTRC_Stepper::KMTRC_Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                             int motor_pin_3, int motor_pin_4)
{
  this->step_number = 0;                   // which step the motor is on
  this->direction = 0;                     // motor direction
  this->last_step_time = 0;                // timestamp in us of the last step taken
  this->number_of_steps = number_of_steps; // total number of steps for this motor

  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1; // Фаза A
  this->motor_pin_2 = motor_pin_2; // Фаза B
  this->motor_pin_3 = motor_pin_3; // Фаза C
  this->motor_pin_4 = motor_pin_4; // Фаза D
}

KMTRC_Stepper::~KMTRC_Stepper()
{
}

/*
 * Sets the speed in revs per minute
 */
void KMTRC_Stepper::setStepperSpeed(long whatSpeed)
{
  this->step_delay = 60L * 1000L * 1000L / this->number_of_steps / whatSpeed;
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void KMTRC_Stepper::stepperStep(int steps_to_move)
{
  int steps_left = abs(steps_to_move); // how many steps to take

  // determine direction based on whether steps_to_mode is + or -:
  if (steps_to_move > 0)
  {
    this->direction = 1;
  }
  if (steps_to_move < 0)
  {
    this->direction = 0;
  }

  // decrement the number of steps, moving one step each time:
  while (steps_left > 0)
  {
    unsigned long now = micros();
    // move only if the appropriate delay has passed:
    if (now - this->last_step_time >= this->step_delay)
    {
      // get the timeStamp of when you stepped:
      this->last_step_time = now;
      // increment or decrement the step number,
      // depending on direction:
      if (this->direction == 1)
      {
        this->step_number++;
        if (this->step_number == this->number_of_steps)
        {
          this->step_number = 0;
        }
      }
      else
      {
        if (this->step_number == 0)
        {
          this->step_number = this->number_of_steps;
        }
        this->step_number--;
      }
      // decrement the steps left:
      steps_left--;
      // step the motor to step number 0, 1, ..., {3 or 10}
      // stepMotor(this->step_number % 4);
      stepMotorALT(this->step_number % 4);
    }
    else
    {
      yield();
    }
  }
}

/*
 * Moves the motor forward or backwards.
 */
// Alternate stepper motor function for our nema17 stepper motor
void KMTRC_Stepper::stepMotorALT(int thisStep)
{
  switch (thisStep)
  {
  case 0: // 1010
    io_expand.digitalWrite(motor_pin_1, HIGH);
    io_expand.digitalWrite(motor_pin_2, HIGH);
    io_expand.digitalWrite(motor_pin_3, LOW);
    io_expand.digitalWrite(motor_pin_4, LOW);
    break;
  case 1: // 0110
    io_expand.digitalWrite(motor_pin_1, LOW);
    io_expand.digitalWrite(motor_pin_2, HIGH);
    io_expand.digitalWrite(motor_pin_3, HIGH);
    io_expand.digitalWrite(motor_pin_4, LOW);
    break;
  case 2: // 0101
    io_expand.digitalWrite(motor_pin_1, LOW);
    io_expand.digitalWrite(motor_pin_2, LOW);
    io_expand.digitalWrite(motor_pin_3, HIGH);
    io_expand.digitalWrite(motor_pin_4, HIGH);
    break;
  case 3: // 1001
    io_expand.digitalWrite(motor_pin_1, HIGH);
    io_expand.digitalWrite(motor_pin_2, LOW);
    io_expand.digitalWrite(motor_pin_3, LOW);
    io_expand.digitalWrite(motor_pin_4, HIGH);
    break;
  }
  // delay(10);
}

//------------------------------------------------Encoder-------------------------------------------------------------------------------------------------

void KMTRC::encoderSetup(uint8_t num)
{
  switch (num)
  {
  case 1:
    encoder.attachHalfQuad(23, 25);
    break;

  case 2:
    encoder2.attachHalfQuad(26, 27);
    break;

  case 3:
    encoder3.attachHalfQuad(0, 2);
    break;

  case 4:
    encoder4.attachHalfQuad(4, 5);
    encoder4.clearCount();
    break;
  }
}

long KMTRC::encPosition(uint8_t num)
{
  long encPos = 0;
  switch (num)
  {
  case 1:
    encPos = encoder.getCount();
    break;

  case 2:
    encPos = encoder2.getCount();
    break;

  case 3:
    encPos = encoder3.getCount();
    break;

  case 4:
    encPos = encoder4.getCount();
    break;
  }

  return encPos;
}

void KMTRC::clearEncoderData(uint8_t num)
{
  switch (num)
  {
  case 1:
    encoder.clearCount();
    break;

  case 2:
    encoder2.clearCount();
    break;

  case 3:
    encoder3.clearCount();
    break;

  case 4:
    encoder4.clearCount();
    break;
  }
}

static const char *TAG = "ESP32Encoder";

enum puType ESP32Encoder::useInternalWeakPullResistors = DOWN;
ESP32Encoder *ESP32Encoder::encoders[MAX_ESP32_ENCODERS] = {
    NULL,
};

bool ESP32Encoder::attachedInterrupt = false;
pcnt_isr_handle_t ESP32Encoder::user_isr_handle = NULL;

ESP32Encoder::ESP32Encoder(bool always_interrupt_, enc_isr_cb_t enc_isr_cb, void *enc_isr_cb_data) : always_interrupt{always_interrupt_},
                                                                                                     aPinNumber{(gpio_num_t)0},
                                                                                                     bPinNumber{(gpio_num_t)0},
                                                                                                     unit{(pcnt_unit_t)-1},
                                                                                                     fullQuad{false},
                                                                                                     countsMode{2},
                                                                                                     count{0},
                                                                                                     r_enc_config{},
                                                                                                     _enc_isr_cb(enc_isr_cb),
                                                                                                     _enc_isr_cb_data(enc_isr_cb_data),
                                                                                                     attached{false},
                                                                                                     direction{false},
                                                                                                     working{false}
{
}

ESP32Encoder::~ESP32Encoder() {}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
#define COUNTER_H_LIM h_lim_lat
#define COUNTER_L_LIM l_lim_lat

static void IRAM_ATTR esp32encoder_pcnt_intr_handler(void *arg)
{
  ESP32Encoder *esp32enc = {};
  uint32_t intr_status = PCNT.int_st.val;
  for (uint8_t i = 0; i < PCNT_UNIT_MAX; i++)
  {
    if (intr_status & (BIT(i)))
    {
      pcnt_unit_t unit = static_cast<pcnt_unit_t>(i);
      esp32enc = ESP32Encoder::encoders[i];
      if (PCNT.status_unit[i].COUNTER_H_LIM)
      {
        esp32enc->count += esp32enc->r_enc_config.counter_h_lim;
        pcnt_counter_clear(unit);
      }
      else if (PCNT.status_unit[i].COUNTER_L_LIM)
      {
        esp32enc->count += esp32enc->r_enc_config.counter_l_lim;
        pcnt_counter_clear(unit);
      }
      else if (esp32enc->always_interrupt && (PCNT.status_unit[i].thres0_lat || PCNT.status_unit[i].thres1_lat))
      {
        int16_t c;
        pcnt_get_counter_value(unit, &c);
        esp32enc->count += c;
        pcnt_set_event_value(unit, PCNT_EVT_THRES_0, -1);
        pcnt_set_event_value(unit, PCNT_EVT_THRES_1, 1);
        pcnt_event_enable(unit, PCNT_EVT_THRES_0);
        pcnt_event_enable(unit, PCNT_EVT_THRES_1);
        pcnt_counter_clear(unit);
        if (esp32enc->_enc_isr_cb)
        {
          esp32enc->_enc_isr_cb(esp32enc->_enc_isr_cb_data);
        }
      }
      PCNT.int_clr.val = BIT(i); // clear the interrupt
    }
  }
}

void ESP32Encoder::detatch()
{
  pcnt_counter_pause(unit);
  ESP32Encoder::encoders[unit] = NULL;
}

void ESP32Encoder::attach(int a, int b, enum encType et)
{
  if (attached)
  {
    ESP_LOGE(TAG, "attach: already attached");
    return;
  }
  int index = 0;
  for (; index < MAX_ESP32_ENCODERS; index++)
  {
    if (ESP32Encoder::encoders[index] == NULL)
    {
      encoders[index] = this;
      break;
    }
  }
  if (index == MAX_ESP32_ENCODERS)
  {
    ESP_LOGE(TAG, "Too many encoders, FAIL!");
    return;
  }

  // Set data now that pin attach checks are done
  fullQuad = et != single;
  unit = (pcnt_unit_t)index;
  this->aPinNumber = (gpio_num_t)a;
  this->bPinNumber = (gpio_num_t)b;

  // Set up the IO state of hte pin
  gpio_pad_select_gpio(aPinNumber);
  gpio_pad_select_gpio(bPinNumber);
  gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
  gpio_set_direction(bPinNumber, GPIO_MODE_INPUT);
  if (useInternalWeakPullResistors == DOWN)
  {
    gpio_pulldown_en(aPinNumber);
    gpio_pulldown_en(bPinNumber);
  }
  if (useInternalWeakPullResistors == UP)
  {
    gpio_pullup_en(aPinNumber);
    gpio_pullup_en(bPinNumber);
  }
  // Set up encoder PCNT configuration
  r_enc_config.pulse_gpio_num = aPinNumber; // Rotary Encoder Chan A
  r_enc_config.ctrl_gpio_num = bPinNumber;  // Rotary Encoder Chan B

  r_enc_config.unit = unit;
  r_enc_config.channel = PCNT_CHANNEL_0;

  r_enc_config.pos_mode = fullQuad ? PCNT_COUNT_DEC : PCNT_COUNT_DIS; // Count Only On Rising-Edges
  r_enc_config.neg_mode = PCNT_COUNT_INC;                             // Discard Falling-Edge

  r_enc_config.lctrl_mode = PCNT_MODE_KEEP;    // Rising A on HIGH B = CW Step
  r_enc_config.hctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step

  r_enc_config.counter_h_lim = _INT16_MAX;
  r_enc_config.counter_l_lim = _INT16_MIN;

  pcnt_unit_config(&r_enc_config);

  if (et == full)
  {
    // set up second channel for full quad
    r_enc_config.pulse_gpio_num = bPinNumber; // make prior control into signal
    r_enc_config.ctrl_gpio_num = aPinNumber;  // and prior signal into control

    r_enc_config.unit = unit;
    r_enc_config.channel = PCNT_CHANNEL_1; // channel 1

    r_enc_config.pos_mode = PCNT_COUNT_DEC; // Count Only On Rising-Edges
    r_enc_config.neg_mode = PCNT_COUNT_INC; // Discard Falling-Edge

    r_enc_config.lctrl_mode = PCNT_MODE_REVERSE; // prior high mode is now low
    r_enc_config.hctrl_mode = PCNT_MODE_KEEP;    // prior low mode is now high

    r_enc_config.counter_h_lim = _INT16_MAX;
    r_enc_config.counter_l_lim = _INT16_MIN;

    pcnt_unit_config(&r_enc_config);
  }
  else
  {                                           // make sure channel 1 is not set when not full quad
    r_enc_config.pulse_gpio_num = bPinNumber; // make prior control into signal
    r_enc_config.ctrl_gpio_num = aPinNumber;  // and prior signal into control

    r_enc_config.unit = unit;
    r_enc_config.channel = PCNT_CHANNEL_1; // channel 1

    r_enc_config.pos_mode = PCNT_COUNT_DIS; // disabling channel 1
    r_enc_config.neg_mode = PCNT_COUNT_DIS; // disabling channel 1

    r_enc_config.lctrl_mode = PCNT_MODE_DISABLE; // disabling channel 1
    r_enc_config.hctrl_mode = PCNT_MODE_DISABLE; // disabling channel 1

    r_enc_config.counter_h_lim = _INT16_MAX;
    r_enc_config.counter_l_lim = _INT16_MIN;

    pcnt_unit_config(&r_enc_config);
  }

  // Filter out bounces and noise
  setFilter(250); // Filter Runt Pulses

  /* Enable events on maximum and minimum limit values */
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit, PCNT_EVT_L_LIM);
  pcnt_counter_pause(unit); // Initial PCNT init
  /* Register ISR handler and enable interrupts for PCNT unit */
  if (!attachedInterrupt)
  {
    esp_err_t er = pcnt_isr_register(esp32encoder_pcnt_intr_handler, (void *)NULL, (int)0,
                                     (pcnt_isr_handle_t *)&ESP32Encoder::user_isr_handle);
    if (er != ESP_OK)
    {
      ESP_LOGE(TAG, "Encoder wrap interrupt failed");
    }
    attachedInterrupt = true;
  }
  if (always_interrupt)
  {
    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, -1);
    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, 1);
    pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    pcnt_event_enable(unit, PCNT_EVT_THRES_1);
  }
  pcnt_counter_clear(unit);
  pcnt_intr_enable(unit);
  pcnt_counter_resume(unit);
}

void ESP32Encoder::attachHalfQuad(int aPintNumber, int bPinNumber)
{
  attach(aPintNumber, bPinNumber, half);
}

void ESP32Encoder::attachSingleEdge(int aPintNumber, int bPinNumber)
{
  attach(aPintNumber, bPinNumber, single);
}

void ESP32Encoder::attachFullQuad(int aPintNumber, int bPinNumber)
{
  attach(aPintNumber, bPinNumber, full);
}

void ESP32Encoder::setCount(int64_t value)
{
  count = value - getCountRaw();
}

int64_t ESP32Encoder::getCountRaw()
{
  int16_t c;
  pcnt_get_counter_value(unit, &c);
  return c;
}

int64_t ESP32Encoder::getCount()
{
  return count + getCountRaw();
}

int64_t ESP32Encoder::clearCount()
{
  count = 0;
  return pcnt_counter_clear(unit);
}

int64_t ESP32Encoder::pauseCount()
{
  return pcnt_counter_pause(unit);
}

int64_t ESP32Encoder::resumeCount()
{
  return pcnt_counter_resume(unit);
}

void ESP32Encoder::setFilter(uint16_t value)
{
  if (value > 1023)
    value = 1023;
  if (value == 0)
  {
    pcnt_filter_disable(unit);
  }
  else
  {
    pcnt_set_filter_value(unit, value);
    pcnt_filter_enable(unit);
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------

int KMTRC::ADCRead(uint8_t pin)
{
  switch (pin)
  {
  case 1:
    return analogRead(35);
  case 2:
    return analogRead(34);
  case 3:
    return analogRead(33);
  default:
    return 0;
  }
}

int KMTRC::DIORead(uint8_t pin)
{
  switch (pin)
  {
  case 1:
    return digitalRead(12);
  case 2:
    return digitalRead(13);
  case 3:
    return digitalRead(14);
  default:
    return 0;
  }
}

void KMTRC::setLedDIO4(uint8_t freq)
{
  setPWM(13, freq);
}

bool KMTRC::ReadButton(uint8_t num)
{
  bool buttonStatus = false;

  if (io_expand.digitalRead(num - 1))
  {
    buttonStatus = true;
    return buttonStatus;
  }

  return buttonStatus;
}

unsigned int KMTRC::distUltrasonic1(void)
{
  unsigned int usDist = 0;
  usDist = uSonic1.read(CM);
  return usDist;
}

unsigned int KMTRC::distUltrasonic2(void)
{
  unsigned int usDist = 0;
  usDist = uSonic2.read(CM);
  return usDist;
}

void KMTRC::setBuzzer(uint32_t freq, long duration)
{
  tone(15, freq, duration);
}

void KMTRC::readAccelGyroSensor(uint8_t port)
{
}

//-------------------------------------------IO_Expander functions--------------------------------------------------------------------

void KMTRC::setPin(uint8_t pin, bool value)
{
  if (value == LOW)
  {
    if (pin >= 100)
    {
      pwmControl.setChannelPWM(pin - 100, 0);
    }
    else
    {
      io_expand.digitalWrite(pin, LOW);
    }
  }
  else
  {
    if (pin >= 100)
    {
      pwmControl.setChannelPWM(pin - 100, 4096);
    }
    else
    {
      io_expand.digitalWrite(pin, HIGH);
    }
  }
}

KMTRC_PCA9555::KMTRC_PCA9555(uint8_t address, int interruptPin)
{
  _address = address; // save the address id
  _valueRegister = 0;
  Wire.begin(32, 33); // start I2C communication

  /*setClock modifies the clock frequency for I2C communication
   * clockFrequency: the value (in Hertz) of desired communication clock.
   * The PCA9555 supports a 400kHz clock.
   * Accepted values are:
   *    10000 low speed mode, supported on some processors
   *    100000, standard mode
   *    400000, fast mode
   */
  Wire.setClock(400000);
}

// Checks if PCA9555 is responsive. Refer to Wire.endTransmission() from Arduino for details.
bool KMTRC_PCA9555::begin()
{
  Wire.beginTransmission(_address);
  Wire.write(0x02); // Test Address
  _error = Wire.endTransmission();

  if (_error != 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

/**
 * @name pinMode
 * @param pin       pin number
 * @param IOMode    mode of pin INPUT or OUTPUT
 * sets the mode of this IO pin
 */
void KMTRC_PCA9555::pinMode(uint8_t pin, uint8_t IOMode)
{

  //
  // check if valid pin first
  //
  if (pin <= 15)
  {
    //
    // now set the correct bit in the configuration register
    //
    if (IOMode == OUTPUT)
    {
      //
      // mask correct bit to 0 by inverting x so that only
      // the correct bit is LOW. The rest stays HIGH
      //
      _configurationRegister = _configurationRegister & ~(1 << pin);
    }
    else
    {
      //
      // or just the required bit to 1
      //
      _configurationRegister = _configurationRegister | (1 << pin);
    }
    //
    // write configuration register to chip
    //
    I2CSetValue(_address, NXP_CONFIG, _configurationRegister_low);
    I2CSetValue(_address, NXP_CONFIG + 1, _configurationRegister_high);
  }
}

/**
 * @name digitalRead Reads the high/low value of specified pin
 * @param pin
 * @return value of pin
 * Reads the selected pin.
 */
uint8_t KMTRC_PCA9555::digitalRead(uint8_t pin)
{
  uint16_t _inputData = 0;
  //
  // we wil only process pins <= 15
  //
  if (pin > 15)
    return 255;
  _inputData = I2CGetValue(_address, NXP_INPUT);
  _inputData |= I2CGetValue(_address, NXP_INPUT + 1) << 8;
  //
  // now mask the bit required and see if it is a HIGH
  //
  if ((_inputData & (1 << pin)) > 0)
  {
    //
    // the bit is HIGH otherwise we would return a LOW value
    //
    return HIGH;
  }
  else
  {
    return LOW;
  }
}

void KMTRC_PCA9555::digitalWrite(uint8_t pin, uint8_t value)
{
  //
  // check valid pin first
  //
  if (pin > 15)
  {
    _error = 255; // invalid pin
    return;       // exit
  }
  //
  // if the value is LOW we will and the register value with correct bit set to zero
  // if the value is HIGH we will or the register value with correct bit set to HIGH
  //
  if (value > 0)
  {
    //
    // this is a High value so we will or it with the value register
    //
    _valueRegister = _valueRegister | (1 << pin); // and OR bit in register
  }
  else
  {
    //
    // this is a LOW value so we have to AND it with 0 into the _valueRegister
    //
    _valueRegister = _valueRegister & ~(1 << pin); // AND all bits
  }
  I2CSetValue(_address, NXP_OUTPUT, _valueRegister_low);
  I2CSetValue(_address, NXP_OUTPUT + 1, _valueRegister_high);
}

// Returns to user the state of desired pin
uint8_t KMTRC_PCA9555::stateOfPin(uint8_t pin)
{
  if ((_stateOfPins & (1 << pin)) > 0)
  {
    //
    // the bit is HIGH otherwise we would return a LOW value
    //
    return HIGH;
  }
  else
  {
    return LOW;
  }
}

/**
 * @name I2CGetValue
 * @param address Address of I2C chip
 * @param reg    Register to read from
 * @return data in register
 * Reads the data from addressed chip at selected register. \n
 * If the value is above 255, an error is set. \n
 * error codes : \n
 * 256 = either 0 or more than one byte is received from the chip
 */
uint16_t KMTRC_PCA9555::I2CGetValue(uint8_t address, uint8_t reg)
{
  uint16_t _inputData;
  //
  // read the address input register
  //
  Wire.beginTransmission(address); // setup read registers
  Wire.write(reg);
  _error = Wire.endTransmission();
  //
  // ask for 2 bytes to be returned
  //
  if (Wire.requestFrom((int)address, 1) != 1)
  {
    //
    // we are not receing the bytes we need
    //
    return 256; // error code is above normal data range
  };
  //
  // read both bytes
  //
  _inputData = Wire.read();
  return _inputData;
}

/**
 * @name I2CSetValue(uint8_t address, uint8_t reg, uint8_t value)
 * @param address Address of I2C chip
 * @param reg    register to write to
 * @param value    value to write to register
 * Write the value given to the register set to selected chip.
 */
void KMTRC_PCA9555::I2CSetValue(uint8_t address, uint8_t reg, uint8_t value)
{
  //
  // write output register to chip
  //
  Wire.beginTransmission(address); // setup direction registers
  Wire.write(reg);                 // pointer to configuration register address 0
  Wire.write(value);               // write config register low byte
  _error = Wire.endTransmission();
}

//-------------------------------------------PWM_Controller functions--------------------------------------

void KMTRC::setPWM(uint8_t pin, uint16_t value)
{
  if (value > 4095)
  {
    pwmControl.setChannelPWM(pin, 4096);
  }
  else
  {
    pwmControl.setChannelPWM(pin, value);
  }
}

#ifndef PCA9685_ENABLE_SOFTWARE_I2C

PCA9685::PCA9685(TwoWire &i2cWire, PCA9685_PhaseBalancer phaseBalancer)
{
  _i2cWire = &i2cWire;
#else
PCA9685::PCA9685(PCA9685_PhaseBalancer phaseBalancer)
{
#endif
  _i2cAddress = 0;
  _phaseBalancer = phaseBalancer;
  _isProxyAddresser = false;
  _lastI2CError = 0;
}

void PCA9685::resetDevices()
{
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::resetDevices");
#endif

  i2cWire_beginTransmission(0x00);
  i2cWire_write(PCA9685_SW_RESET);
  i2cWire_endTransmission();

  delayMicroseconds(10);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
//    checkForErrors();
#endif
}

void PCA9685::init(byte i2cAddress, byte mode)
{
  if (_isProxyAddresser)
    return;

  // I2C 7-bit address is B 1 A5 A4 A3 A2 A1 A0
  // RW bit added by Arduino core TWI library
  _i2cAddress = PCA9685_I2C_BASE_ADDRESS | (i2cAddress & 0x3F);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::init i2cAddress: 0x");
  Serial.println(_i2cAddress, HEX);
#endif

  writeRegister(PCA9685_MODE1_REG, PCA9685_MODE_RESTART | PCA9685_MODE_AUTOINC);
  writeRegister(PCA9685_MODE2_REG, mode);
}

#ifndef PCA9685_EXCLUDE_EXT_FUNC

void PCA9685::initAsProxyAddresser(byte i2cAddress)
{
  _i2cAddress = i2cAddress & 0xFE;
  _isProxyAddresser = true;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::initAsProxyAddresser i2cAddress: 0x");
  Serial.println(_i2cAddress, HEX);
#endif
}

#endif

byte PCA9685::getI2CAddress()
{
  return _i2cAddress;
}

PCA9685_PhaseBalancer PCA9685::getPhaseBalancer()
{
  return _phaseBalancer;
}

void PCA9685::setPWMFrequency(float pwmFrequency)
{
  if (pwmFrequency < 0 || _isProxyAddresser)
    return;

  // This equation comes from section 7.3.5 of the datasheet, but the rounding has been
  // removed because it isn't needed. Lowest freq is 23.84, highest is 1525.88.
  int preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
  if (preScalerVal > 255)
    preScalerVal = 255;
  if (preScalerVal < 3)
    preScalerVal = 3;
  //  Serial.print("PCA9685::setPWMFrequency pwmFrequency: ");
  //  Serial.print(pwmFrequency);
  //  Serial.print(", preScalerVal: 0x");
  //  Serial.println(preScalerVal, HEX);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::setPWMFrequency pwmFrequency: ");
  Serial.print(pwmFrequency);
  Serial.print(", preScalerVal: 0x");
  Serial.println(preScalerVal, HEX);
#endif

  // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_RESTART) | PCA9685_MODE_SLEEP));
  writeRegister(PCA9685_PRESCALE_REG, (byte)preScalerVal);

  // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
  writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_SLEEP) | PCA9685_MODE_RESTART));
  delayMicroseconds(500);
}

void PCA9685::setChannelOn(int channel)
{
  if (channel < 0 || channel > 15)
    return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::setChannelOn");
#endif

  writeChannelBegin(channel);
  writeChannelPWM(PCA9685_PWM_FULL, 0); // time_on = FULL; time_off = 0;
  writeChannelEnd();
}

void PCA9685::setChannelOff(int channel)
{
  if (channel < 0 || channel > 15)
    return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::setChannelOff");
#endif

  writeChannelBegin(channel);
  writeChannelPWM(0, PCA9685_PWM_FULL); // time_on = 0; time_off = FULL;
  writeChannelEnd();
}

void PCA9685::setChannelPWM(int channel, uint16_t pwmAmount)
{
  if (channel < 0 || channel > 15)
    return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::setChannelPWM");
#endif

  writeChannelBegin(channel);

  uint16_t phaseBegin, phaseEnd;
  getPhaseCycle(channel, pwmAmount, &phaseBegin, &phaseEnd);

  writeChannelPWM(phaseBegin, phaseEnd);

  writeChannelEnd();
}

void PCA9685::setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts)
{
  if (begChannel < 0 || begChannel > 15 || numChannels < 0)
    return;
  if (begChannel + numChannels > 16)
    numChannels -= (begChannel + numChannels) - 16;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::setChannelsPWM numChannels: ");
  Serial.println(numChannels);
#endif

  // In avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
  // how many channels can be written at once. Therefore, we loop around until all
  // channels have been written out into their registers.

  while (numChannels > 0)
  {
    writeChannelBegin(begChannel);

    int maxChannels = min(numChannels, (BUFFER_LENGTH - 1) / 4);
    while (maxChannels-- > 0)
    {
      uint16_t phaseBegin, phaseEnd;
      getPhaseCycle(begChannel++, *pwmAmounts++, &phaseBegin, &phaseEnd);

      writeChannelPWM(phaseBegin, phaseEnd);
      --numChannels;
    }

    writeChannelEnd();
    if (_lastI2CError)
      return;
  }
}

#ifndef PCA9685_EXCLUDE_EXT_FUNC

void PCA9685::setAllChannelsPWM(uint16_t pwmAmount)
{
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::setAllChannelsPWM");
#endif

  writeChannelBegin(-1); // Special value for ALLLED registers

  uint16_t phaseBegin, phaseEnd;
  getPhaseCycle(-1, pwmAmount, &phaseBegin, &phaseEnd);

  writeChannelPWM(phaseBegin, phaseEnd);

  writeChannelEnd();
}

uint16_t PCA9685::getChannelPWM(int channel)
{
  if (channel < 0 || channel > 15 || _isProxyAddresser)
    return 0;

  byte regAddress = PCA9685_LED0_REG + (channel << 2);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::getChannelPWM channel: ");
  Serial.print(channel);
  Serial.print(", regAddress: 0x");
  Serial.println(regAddress, HEX);
#endif

  i2cWire_beginTransmission(_i2cAddress);
  i2cWire_write(regAddress);
  if (i2cWire_endTransmission())
  {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
//        checkForErrors();
#endif
    return 0;
  }

  int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, (uint8_t)4);
  if (bytesRead != 4)
  {
    while (bytesRead-- > 0)
      i2cWire_read();
    _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
//        checkForErrors();
#endif
    return 0;
  }

  uint16_t phaseBegin = (uint16_t)i2cWire_read();
  phaseBegin |= (uint16_t)i2cWire_read() << 8;
  uint16_t phaseEnd = (uint16_t)i2cWire_read();
  phaseEnd |= (uint16_t)i2cWire_read() << 8;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("  PCA9685::getChannelPWM phaseBegin: ");
  Serial.print(phaseBegin);
  Serial.print(", phaseEnd: ");
  Serial.println(phaseEnd);
#endif

  // See datasheet section 7.3.3
  uint16_t retVal;
  if (phaseEnd >= PCA9685_PWM_FULL)
    // Full OFF
    // Figure 11 Example 4: full OFF takes precedence over full ON
    // See also remark after Table 7
    retVal = 0;
  else if (phaseBegin >= PCA9685_PWM_FULL)
    // Full ON
    // Figure 9 Example 3
    retVal = PCA9685_PWM_FULL;
  else if (phaseBegin <= phaseEnd)
    // start and finish in same cycle
    // Section 7.3.3 example 1
    retVal = phaseEnd - phaseBegin;
  else
    // span cycles
    // Section 7.3.3 example 2
    retVal = (phaseEnd + PCA9685_PWM_FULL) - phaseBegin;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("  PCA9685::getChannelPWM retVal: ");
  Serial.println(retVal);
#endif

  return retVal;
}

void PCA9685::enableAllCallAddress(byte i2cAddress)
{
  if (_isProxyAddresser)
    return;

  i2cAddress &= 0xFE;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::enableAllCallAddress i2cAddress: 0x");
  Serial.println(i2cAddress, HEX);
#endif

  writeRegister(PCA9685_ALLCALL_REG, i2cAddress);

  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_ALLCALL));
}

void PCA9685::enableSub1Address(byte i2cAddress)
{
  if (_isProxyAddresser)
    return;

  i2cAddress &= 0xFE;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::enableSub1Address i2cAddress: 0x");
  Serial.println(i2cAddress, HEX);
#endif

  writeRegister(PCA9685_SUBADR1_REG, i2cAddress);

  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_SUBADR1));
}

void PCA9685::enableSub2Address(byte i2cAddress)
{
  if (_isProxyAddresser)
    return;

  i2cAddress &= 0xFE;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::enableSub2Address i2cAddress: 0x");
  Serial.println(i2cAddress, HEX);
#endif

  writeRegister(PCA9685_SUBADR2_REG, i2cAddress);

  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_SUBADR2));
}

void PCA9685::enableSub3Address(byte i2cAddress)
{
  if (_isProxyAddresser)
    return;

  i2cAddress &= 0xFE;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("PCA9685::enableSub3Address i2cAddress: 0x");
  Serial.println(i2cAddress, HEX);
#endif

  writeRegister(PCA9685_SUBADR3_REG, i2cAddress);

  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_SUBADR3));
}

void PCA9685::disableAllCallAddress()
{
  if (_isProxyAddresser)
    return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::disableAllCallAddress");
#endif

  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE_ALLCALL));
}

void PCA9685::disableSub1Address()
{
  if (_isProxyAddresser)
    return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::disableSub1Address");
#endif

  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE_SUBADR1));
}

void PCA9685::disableSub2Address()
{
  if (_isProxyAddresser)
    return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::disableSub2Address");
#endif

  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE_SUBADR2));
}

void PCA9685::disableSub3Address()
{
  if (_isProxyAddresser)
    return;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::disableSub3Address");
#endif

  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg &= ~PCA9685_MODE_SUBADR3));
}

void PCA9685::enableExtClockLine()
{
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.println("PCA9685::enableExtClockLine");
#endif

  // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
  byte mode1Reg = readRegister(PCA9685_MODE1_REG);
  writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_RESTART) | PCA9685_MODE_SLEEP));
  writeRegister(PCA9685_MODE1_REG, (mode1Reg |= PCA9685_MODE_EXTCLK));

  // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
  writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE_SLEEP) | PCA9685_MODE_RESTART));
  delayMicroseconds(500);
}

#endif

byte PCA9685::getLastI2CError()
{
  return _lastI2CError;
}

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT

static const char *textForI2CError(byte errorCode)
{
  switch (errorCode)
  {
  case 0:
    return "Success";
  case 1:
    return "Data too long to fit in transmit buffer";
  case 2:
    return "Received NACK on transmit of address";
  case 3:
    return "Received NACK on transmit of data";
  default:
    return "Other error";
  }
}

/*void PCA9685::checkForErrors() {
    if (_lastI2CError) {
        Serial.print("  PCA9685::checkErrors lastI2CError: ");
        Serial.print(_lastI2CError);
        Serial.print(": ");
        Serial.println(textForI2CError(getLastI2CError()));
    }
}*/

#endif

void PCA9685::getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd)
{
  // Set delay
  if (channel < 0)
  {
    // All channels
    *phaseBegin = 0;
  }
  else if (_phaseBalancer == PCA9685_PhaseBalancer_Linear)
  {
    // Distribute high phase area over entire phase range to balance load.
    *phaseBegin = channel * (4096 / 16);
  }
  else if (_phaseBalancer == PCA9685_PhaseBalancer_Weaved)
  {
    // Distribute high phase area over entire phase range to balance load.
    *phaseBegin = phaseDistTable[channel];
  }
  else
  {
    *phaseBegin = 0;
  }

  // See datasheet section 7.3.3
  if (pwmAmount == 0)
  {
    // Full OFF => time_off[12] = 1;
    *phaseEnd = PCA9685_PWM_FULL;
  }
  else if (pwmAmount >= PCA9685_PWM_FULL)
  {
    // Full ON => time_on[12] = 1; time_off = ignored;
    *phaseBegin |= PCA9685_PWM_FULL;
    *phaseEnd = 0;
  }
  else
  {
    *phaseEnd = *phaseBegin + pwmAmount;
    if (*phaseEnd >= PCA9685_PWM_FULL)
      *phaseEnd -= PCA9685_PWM_FULL;
  }
}

void PCA9685::writeChannelBegin(int channel)
{
  byte regAddress;

  if (channel != -1)
    regAddress = PCA9685_LED0_REG + (channel * 0x04);
  else
    regAddress = PCA9685_ALLLED_REG;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("  PCA9685::writeChannelBegin channel: ");
  Serial.print(channel);
  Serial.print(", regAddress: 0x");
  Serial.println(regAddress, HEX);
#endif

  i2cWire_beginTransmission(_i2cAddress);
  i2cWire_write(regAddress);
}

void PCA9685::writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd)
{
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("  PCA9685::writeChannelPWM phaseBegin: ");
  Serial.print(phaseBegin);
  Serial.print(", phaseEnd: ");
  Serial.println(phaseEnd);
#endif

  i2cWire_write(lowByte(phaseBegin));
  i2cWire_write(highByte(phaseBegin));
  i2cWire_write(lowByte(phaseEnd));
  i2cWire_write(highByte(phaseEnd));
}

void PCA9685::writeChannelEnd()
{
  i2cWire_endTransmission();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
//    checkForErrors();
#endif
}

void PCA9685::writeRegister(byte regAddress, byte value)
{
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("  PCA9685::writeRegister regAddress: 0x");
  Serial.print(regAddress, HEX);
  Serial.print(", value: 0x");
  Serial.println(value, HEX);
#endif

  i2cWire_beginTransmission(_i2cAddress);
  i2cWire_write(regAddress);
  i2cWire_write(value);
  i2cWire_endTransmission();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
//    checkForErrors();
#endif
}

byte PCA9685::readRegister(byte regAddress)
{
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("  PCA9685::readRegister regAddress: 0x");
  Serial.println(regAddress, HEX);
#endif

  i2cWire_beginTransmission(_i2cAddress);
  i2cWire_write(regAddress);
  if (i2cWire_endTransmission())
  {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
//        checkForErrors();
#endif
    return 0;
  }

  int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, (uint8_t)1);
  if (bytesRead != 1)
  {
    while (bytesRead-- > 0)
      i2cWire_read();
    _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
//        checkForErrors();
#endif
    return 0;
  }

  byte retVal = i2cWire_read();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  Serial.print("    PCA9685::readRegister retVal: 0x");
  Serial.println(retVal, HEX);
#endif

  return retVal;
}

#ifdef PCA9685_ENABLE_SOFTWARE_I2C
bool __attribute__((noinline)) i2c_start(uint8_t addr);
void __attribute__((noinline)) i2c_stop(void) asm("ass_i2c_stop");
bool __attribute__((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write");
uint8_t __attribute__((noinline)) i2c_read(bool last);
#endif

void PCA9685::i2cWire_beginTransmission(uint8_t addr)
{
  _lastI2CError = 0;
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
  _i2cWire->beginTransmission(addr);
#else
  i2c_start(addr);
#endif
}

uint8_t PCA9685::i2cWire_endTransmission(void)
{
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
  return (_lastI2CError = _i2cWire->endTransmission());
#else
  i2c_stop();
  return (_lastI2CError = 0);
#endif
}

uint8_t PCA9685::i2cWire_requestFrom(uint8_t addr, uint8_t len)
{
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
  return _i2cWire->requestFrom(addr, len);
#else
  i2c_start(addr | 0x01);
  return (_readBytes = len);
#endif
}

size_t PCA9685::i2cWire_write(uint8_t data)
{
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
  return _i2cWire->write(data);
#else
  return (size_t)i2c_write(data);
#endif
}

uint8_t PCA9685::i2cWire_read(void)
{
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
  return (uint8_t)(_i2cWire->read() & 0xFF);
#else
  if (_readBytes > 1)
  {
    _readByes -= 1;
    return (uint8_t)(i2c_read(false) & 0xFF);
  }
  else
  {
    _readBytes = 0;
    return (uint8_t)(i2c_read(true) & 0xFF);
  }
#endif
}

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT

void PCA9685::printModuleInfo()
{
  Serial.println("");
  Serial.println(" ~~~ PCA9685 Module Info ~~~");

  Serial.println("");
  Serial.println("i2c Address:");
  Serial.print("0x");
  Serial.println(_i2cAddress, HEX);

  Serial.println("");
  Serial.println("Phase Balancer:");
  switch (_phaseBalancer)
  {
  case PCA9685_PhaseBalancer_None:
    Serial.println("PCA9685_PhaseBalancer_None");
    break;
  case PCA9685_PhaseBalancer_Linear:
    Serial.println("PCA9685_PhaseBalancer_Linear");
    break;
  case PCA9685_PhaseBalancer_Weaved:
    Serial.println("PCA9685_PhaseBalancer_Weaved");
    break;
  default:
    Serial.println("");
    break;
  }

  if (!_isProxyAddresser)
  {
    Serial.println("");
    Serial.println("Proxy Addresser:");
    Serial.println("false");

    Serial.println("");
    Serial.println("Mode1 Register:");
    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    Serial.print("0x");
    Serial.print(mode1Reg, HEX);
    Serial.print(", Bitset:");
    if (mode1Reg & PCA9685_MODE_RESTART)
      Serial.print(" PCA9685_MODE_RESTART");
    if (mode1Reg & PCA9685_MODE_EXTCLK)
      Serial.print(" PCA9685_MODE_EXTCLK");
    if (mode1Reg & PCA9685_MODE_AUTOINC)
      Serial.print(" PCA9685_MODE_AUTOINC");
    if (mode1Reg & PCA9685_MODE_SLEEP)
      Serial.print(" PCA9685_MODE_SLEEP");
    if (mode1Reg & PCA9685_MODE_SUBADR1)
      Serial.print(" PCA9685_MODE_SUBADR1");
    if (mode1Reg & PCA9685_MODE_SUBADR2)
      Serial.print(" PCA9685_MODE_SUBADR2");
    if (mode1Reg & PCA9685_MODE_SUBADR3)
      Serial.print(" PCA9685_MODE_SUBADR3");
    if (mode1Reg & PCA9685_MODE_ALLCALL)
      Serial.print(" PCA9685_MODE_ALLCALL");
    Serial.println("");

    Serial.println("");
    Serial.println("Mode2 Register:");
    byte mode2Reg = readRegister(PCA9685_MODE2_REG);
    Serial.print("0x");
    Serial.print(mode2Reg, HEX);
    Serial.print(", Bitset:");
    if (mode2Reg & PCA9685_MODE_INVRT)
      Serial.print(" PCA9685_MODE_INVRT");
    if (mode2Reg & PCA9685_MODE_OUTPUT_ONACK)
      Serial.print(" PCA9685_MODE_OUTPUT_ONACK");
    if (mode2Reg & PCA9685_MODE_OUTPUT_TPOLE)
      Serial.print(" PCA9685_MODE_OUTPUT_TPOLE");
    if (mode2Reg & PCA9685_MODE_OUTNE_HIGHZ)
      Serial.print(" PCA9685_MODE_OUTNE_HIGHZ");
    if (mode2Reg & PCA9685_MODE_OUTNE_LOW)
      Serial.print(" PCA9685_MODE_OUTNE_LOW");
    Serial.println("");

    Serial.println("");
    Serial.println("SubAddress1 Register:");
    byte subAdr1Reg = readRegister(PCA9685_SUBADR1_REG);
    Serial.print("0x");
    Serial.println(subAdr1Reg, HEX);

    Serial.println("");
    Serial.println("SubAddress2 Register:");
    byte subAdr2Reg = readRegister(PCA9685_SUBADR2_REG);
    Serial.print("0x");
    Serial.println(subAdr2Reg, HEX);

    Serial.println("");
    Serial.println("SubAddress3 Register:");
    byte subAdr3Reg = readRegister(PCA9685_SUBADR3_REG);
    Serial.print("0x");
    Serial.println(subAdr3Reg, HEX);

    Serial.println("");
    Serial.println("AllCall Register:");
    byte allCallReg = readRegister(PCA9685_ALLCALL_REG);
    Serial.print("0x");
    Serial.println(allCallReg, HEX);
  }
  else
  {
    Serial.println("");
    Serial.println("Proxy Addresser:");
    Serial.println("true");
  }
}

#endif

#ifndef PCA9685_EXCLUDE_SERVO_EVAL

PCA9685_ServoEvaluator::PCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t p90PWMAmount)
{
  n90PWMAmount = constrain(n90PWMAmount, 0, PCA9685_PWM_FULL);
  p90PWMAmount = constrain(p90PWMAmount, n90PWMAmount, PCA9685_PWM_FULL);

  _coeff = new float[2];
  _isCSpline = false;

  _coeff[0] = n90PWMAmount;
  _coeff[1] = (p90PWMAmount - n90PWMAmount) / 180.0f; // 240.0f;
}

PCA9685_ServoEvaluator::PCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount)
{
  n90PWMAmount = constrain(n90PWMAmount, 0, PCA9685_PWM_FULL);
  zeroPWMAmount = constrain(zeroPWMAmount, n90PWMAmount, PCA9685_PWM_FULL);
  p90PWMAmount = constrain(p90PWMAmount, zeroPWMAmount, PCA9685_PWM_FULL);

  if (p90PWMAmount - zeroPWMAmount != zeroPWMAmount - n90PWMAmount)
  {
    _coeff = new float[8];
    _isCSpline = true;

    // Cubic spline code adapted from: https://shiftedbits.org/2011/01/30/cubic-spline-interpolation/
    /* "THE BEER-WARE LICENSE" (Revision 42): Devin Lane wrote this [part]. As long as you retain
     * this notice you can do whatever you want with this stuff. If we meet some day, and you
     * think this stuff is worth it, you can buy me a beer in return. */

    float x[3] = {0, 90, 180};
    float y[3] = {(float)n90PWMAmount, (float)zeroPWMAmount, (float)p90PWMAmount};
    float c[3], b[2], d[2], h[2], l[1], u[2], a[1], z[2]; // n = 3

    h[0] = x[1] - x[0];
    u[0] = z[0] = 0;
    c[2] = 0;

    for (int i = 1; i < 2; ++i)
    {
      h[i] = x[i + 1] - x[i];
      l[i - 1] = (2 * (x[i + 1] - x[i - 1])) - h[i - 1] * u[i - 1];
      u[i] = h[i] / l[i - 1];
      a[i - 1] = (3 / h[i]) * (y[i + 1] - y[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
      z[i] = (a[i - 1] - h[i - 1] * z[i - 1]) / l[i - 1];
    }

    for (int i = 1; i >= 0; --i)
    {
      c[i] = z[i] - u[i] * c[i + 1];
      b[i] = (y[i + 1] - y[i]) / h[i] - (h[i] * (c[i + 1] + 2 * c[i])) / 3;
      d[i] = (c[i + 1] - c[i]) / (3 * h[i]);

      _coeff[4 * i + 0] = y[i]; // a
      _coeff[4 * i + 1] = b[i]; // b
      _coeff[4 * i + 2] = c[i]; // c
      _coeff[4 * i + 3] = d[i]; // d
    }
  }
  else
  {
    _coeff = new float[2];
    _isCSpline = false;

    _coeff[0] = n90PWMAmount;
    _coeff[1] = (p90PWMAmount - n90PWMAmount) / 180.0f; // 240.0f;
  }
}

PCA9685_ServoEvaluator::~PCA9685_ServoEvaluator()
{
  if (_coeff)
    delete[] _coeff;
}

uint16_t PCA9685_ServoEvaluator::pwmForAngle(float angle)
{
  float retVal;
  angle = constrain(angle, 0, 180);

  if (!_isCSpline)
  {
    retVal = _coeff[0] + (_coeff[1] * angle);
  }
  else
  {
    if (angle <= 90)
    {
      retVal = _coeff[0] + (_coeff[1] * angle) + (_coeff[2] * angle * angle) + (_coeff[3] * angle * angle * angle);
    }
    else
    {
      angle -= 90;
      retVal = _coeff[4] + (_coeff[5] * angle) + (_coeff[6] * angle * angle) + (_coeff[7] * angle * angle * angle);
    }
  }

  return (uint16_t)constrain((int)roundf(retVal), 0, PCA9685_PWM_FULL);
};

#endif

//--------------------------------------------Ultrasonic functions------------------------------------------

KMTRC_Ultrasonic::KMTRC_Ultrasonic(uint8_t trigPin, uint8_t echoPin, unsigned long timeOut)
{
  trig = trigPin;
  echo = echoPin;
  threePins = trig == echo ? true : false;

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  timeout = timeOut;
}

unsigned int KMTRC_Ultrasonic::timing()
{
  unsigned int maxTime = 0;

  //  unsigned int duration;
  if (threePins)
  {
    pinMode(trig, OUTPUT);
  }

  digitalWrite(trig, LOW);
  delayMicroseconds(4);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);

  digitalWrite(trig, LOW);

  if (threePins)
  {
    pinMode(trig, INPUT);
  }

  previousMicros = micros();
  // if (digitalRead(echo)) return false;
  //  max_time = micros() + _maxEchoTime + MAX_SENSOR_DELAY;
  //  while (!digitalRead(_echo))
  //    if (micros() > max_time) return false;
  //  max_time = micros() + _maxEchoTime;

  //  while(!mcp_mot.digitalRead(echo) && (micros() - previousMicros) <= timeout); // wait for the echo pin HIGH or timeout
  while (!digitalRead(echo) && (micros() - previousMicros) <= timeout)
    ; // wait for the echo pin HIGH or timeout

  previousMicros = micros();
  //  while(mcp_mot.digitalRead(echo)  && (micros() - previousMicros) <= timeout); // wait for the echo pin LOW or timeout
  while (digitalRead(echo) && (micros() - previousMicros) <= timeout)
    ; // wait for the echo pin LOW or timeout

  // delayMicroseconds(200);
  //   duration = pulseIn(echo, HIGH);
  return micros() - previousMicros; // duration
  // delayMicroseconds(100);
}

/*
 * If the unit of measure is not passed as a parameter,
 * sby default, it will return the distance in centimeters.
 * To change the default, replace CM by INC.
 */
unsigned int KMTRC_Ultrasonic::read(uint8_t und)
{
  return timing() / und / 2; // distance by divisor
}
