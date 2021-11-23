// BLDC driver standalone example
#include <SimpleFOC.h>

//Enable this preprocessor switch to make this code compatible with hall sensor
//remember to activate the ENABLE_HALL_SENSOR switch from main.cpp, BLDCMotor.cpp and HallSensor.cpp
#define ENABLE_HALL_SENSOR

#ifdef ENABLE_HALL_SENSOR
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

#define YELLOW_HS   2
#define GREEN_HS    4
#define BLUE_HS     7

#endif


#define POLE_PAIR   10

#define PHASE_AH_PIN 6
#define PHASE_AL_PIN 5

#define PHASE_BH_PIN 9
#define PHASE_BL_PIN 10

#define PHASE_CH_PIN 11
#define PHASE_CL_PIN 3

#define ENABLE_MOSFET_AND_DRIVER_POWER  8
#define STATUS_LED                      13
#define BREAK                           12 
#define THROTTLE                        A0

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
/**
 * @brief BLDCMotor constructor sets 
 * pole pair
 * phase resistance
 * TorqueControlType::voltage
 */
BLDCMotor motor = BLDCMotor(POLE_PAIR);
// BLDC driver instance
/**
 * @brief BLDCDriver6PWM sets the following
 * pin for pwm_Ah
 * pin for pwm_Al
 * pin for pwm_Bh
 * pin for pwm_Bl
 * pin for pwm_Ch
 * pin for pwm_Cl
 * 
 * enable pin if available.
 * 
 * voltage power supply to 12V
 * voltage limit not set
 * 
 * dead zone to 0.02 that is 2%
 */
BLDCDriver6PWM driver = BLDCDriver6PWM(PHASE_AH_PIN, PHASE_AL_PIN, PHASE_BH_PIN,PHASE_BL_PIN, PHASE_CH_PIN, PHASE_CL_PIN);






#ifdef ENABLE_HALL_SENSOR
HallSensor sensor = HallSensor(YELLOW_HS, GREEN_HS, BLUE_HS, POLE_PAIR);
// Interrupt routine intialisation
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}
// If no available hadware interrupt pins use the software interrupt
// PciListenerImp listenA(sensor.pinA, doA);
PciListenerImp listenB(sensor.pinB, doB);
PciListenerImp listenC(sensor.pinC, doC);
#endif

float target_velocity = 0;
unsigned int throttle_value = 0;
unsigned int filtered_throttle_value = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  Serial.begin(115200);
#ifdef ENABLE_HALL_SENSOR
  sensor.pullup = Pullup::USE_EXTERN;
  sensor.init();
  sensor.enableInterrupts(doA);
  PciManager.registerListener(&listenB);
  PciManager.registerListener(&listenC);
  motor.linkSensor(&sensor);
#endif



  // driver config
  // power supply voltage [V]
  /**
   * @brief by default voltage power supply is set to 12v, configured above.
   * user can change it's value accordingly.
   */
  driver.voltage_power_supply = 12;
  // daad_zone [0,1] - default 0.02 - 2%
  /**
   * @brief dead_zone is configured above to 2%
   * but user can change it accordingly.
   */
  driver.dead_zone = 0.05;
  driver.init();
  driver.enable();
  _delay(1000);
  motor.linkDriver(&driver);

  // limiting motor movements
  //changing hall sensor alignment voltage value
#ifdef ENABLE_HALL_SENSOR  
  // aligning voltage [V]
  motor.voltage_sensor_align = 2;
#endif
  /**
   * @brief motor.voltage_limit is used in motor.init function.
   * default value of voltage_limit is 12V
   * 
   */
  motor.voltage_limit = 5;   // [V]
  /**
   * @brief defualt velocity_limit is 20
   * velocity limit is used in motor.init function.
   */
  motor.velocity_limit = 20; // [rad/s] cca 50rpm

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SinePWM;
 
  //torque control type
  motor.torque_controller = TorqueControlType::voltage;

  // open loop control config
#ifdef ENABLE_HALL_SENSOR
  motor.controller = MotionControlType::velocity;
#endif
#ifndef ENABLE_HALL_SENSOR
  motor.controller = MotionControlType::velocity_openloop;
#endif
  motor.useMonitoring(Serial);
  motor.init();

  pinMode(ENABLE_MOSFET_AND_DRIVER_POWER, OUTPUT);
  digitalWrite(ENABLE_MOSFET_AND_DRIVER_POWER, LOW);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  pinMode(BREAK, INPUT_PULLUP);

#ifdef ENABLE_HALL_SENSOR
  motor.initFOC(4.19f, CW);
#endif
  // add target command T
  command.add('T', doTarget, "target velocity");

  
#ifdef ENABLE_HALL_SENSOR
  Serial.println("Sensor ready");
#endif
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
#ifdef ENABLE_HALL_SENSOR
  motor.loopFOC();
#endif

//managing break
  if(digitalRead(BREAK) == 0) { 
    target_velocity = 0.0f;
    // Serial.println("break is pressed");
  }else{
    // Serial.println("break is released");
    target_velocity = (float)filtered_throttle_value;
  }
//reading throttle values
throttle_value = ((throttle_value<<5) - throttle_value + analogRead(THROTTLE))>>5;
filtered_throttle_value = (unsigned int)map(throttle_value, 145, 900, 0, 4);
// Serial.println(filtered_throttle_value);
//setting target velocity to the motor and moving the motor
  motor.move(target_velocity);
//reading the user input for target velocity and setting it to the target_velocity variable.
  // command.run();

  // Serial.println(target_velocity);
}
