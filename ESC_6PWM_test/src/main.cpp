// BLDC driver standalone example
#include <SimpleFOC.h>

//Enable this preprocessor switch to make this code compatible with hall sensor
//remember to activate the ENABLE_HALL_SENSOR switch from main.cpp, BLDCMotor.cpp and HallSensor.cpp
// #define ENABLE_HALL_SENSOR

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

#define ENABLE_MOSFET_AND_DRIVER_POWER 8
#define STATUS_LED  13

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
// Hall sensor instance
// HallSensor(int hallA, int hallB , int cpr, int index)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
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






//target variable
/**
 * @brief target_velocity is a user defined variable
 * 
 */
float target_velocity = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  Serial.begin(115200);
#ifdef ENABLE_HALL_SENSOR
  // monitoring port
  // Serial.begin(115200);

  // check if you need internal pullups
  sensor.pullup = Pullup::USE_EXTERN;
  
  // initialise encoder hardware
  sensor.init();
  
  // software interrupts
  // hardware interrupt enable
  sensor.enableInterrupts(doA);//, doB, doC);
  // PciManager.registerListener(&listenA);
  PciManager.registerListener(&listenB);
  PciManager.registerListener(&listenC);
  // link the motor to the sensor
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

  /**
   * @brief init function does the following
   * sets pwm_Ah as output
   * sets pwm_Al as output
   * sets pwm_Bh as output
   * sets pwm_Bl as output
   * sets pwm_Ch as output
   * sets pwm_Cl as output
   * 
   * checks if enable pin is available then configures it as output.
   * 
   * checks if voltage_limit is not set or if voltage_limit > voltage_power_supply then
   * sets the voltage_limit to voltage_power_supply.
   * 
   * configures6PWMs by calling
   *    configureComplementyPair(pwm_Ah,pwm_Al) this function return 1 on successful config else -1
   *    configureComplementyPair(pwm_Bh,pwm_Bl) this function return 1 on successful config else -1
   *    configureComplementyPair(pwm_Ch,pwm_Cl) this function return 1 on successful config else -1
   * configure6PWM function returns 3 on successful configuraton of three phases pins.
   * 
   * at the end init function returns 3 on successful initilization.
   */
  driver.init();

  // enable driver
  /**
   * @brief enable function does the following
   *  
   * first see if enable pin is available then set it to high and then
   * sets pwm width to 0 for all the three complementry channel pairs, by using
   *    setPwm(0,0,0) ===>>> where Ua = 0, Ub = 0, Uc = 0;
   *        this function constrains phase voltage Ua in (0, voltage_limit) and store in Ua
   *        this function constrains phase voltage Ub in (0, voltage_limit) and store in Ub
   *        this function constrains phase voltage Uc in (0, voltage_limit) and store in Uc
   *        
   *        calculating the duty cycle [0,1]
   *        this function constrains Ua/voltage_power_supply in (0, 1) and store in dc_a
   *        this function constrains Ub/voltage_power_supply in (0, 1) and store in dc_b
   *        this function constrains Uc/voltage_power_supply in (0, 1) and store in dc_c
   * 
   *        then calls writeDutyCycle6PWM(dc_a, dc_b, dc_c, dead_zone, pwmA_h,pwmA_l, pwmB_h,pwmB_l, pwmC_h,pwmC_l)
   *        writeDutyCycle6PWM calls 
   *         
   *        _setPwmPair(pinA_h, pinA_l, dc_a*255.0(duty_cycle at scale of 255), dead_zone*255.0(dead_time));
   *              calculated (duty_cycle-dead_time/2) and constrain it on (0,255) and store in pwm_h
   *              pwm_h is the width of pwm to be generated at pinA_h
   *              analogWrite(pinA_h, pwm_h);
   * 
   *              calculate (duty_cycle+dead_time/2) and constrain it on (0,255) and store in pwm_l
   *              pwm_l is the width of pwm to be generated at pinA_l
   *              
   *              check if pwm_l is 0 or 255, if yes then use digitalWrite on pinA_l
   *              else use analogWrite(pinA_l, pwm_l); 
   * 
   *        _setPwmPair(pinB_h, pinB_l, dc_b*255.0, dead_zone*255.0);
   *              same as phase A
   *        _setPwmPair(pinC_h, pinC_l, dc_c*255.0, dead_zone*255.0);
   *              same as phase A 
   */
  driver.enable();

  _delay(1000);

  // link the motor and the driver
  /**
   * @brief linkDriver functin passes the local driver object to the driver object of BLDCMotor
   * 
   */
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

  // comment out if not needed
  motor.useMonitoring(Serial);
  // init motor hardware
  /**
   * @brief motor.init() does the following
   *    it first checks if monitor_port is set then print MOT: Init
   *    
   *    if no current_sense is available and phase resistance is set by the user then calculate
   *    new voltage limit
   * 
   *    if voltage_limit > driver->voltage_limit then voltage_limit = driver->voltage_limit
   * 
   *    constrain voltage for sensor alignment
   * 
   *    update the controller limits, this is related to PID
   * 
   *    P_angle.limit = velocity_limit;, related to PID
   * 
   *    based on monitor_port variable, MOT: Enable driver is printed.
   * 
   *    now this init function call the enable function that does the following
   *        it calls driver->enable function
   *        sets pwm to zero using driver->setPWM(0,0,0)
   *        then writes enabled = 1 to show the status.
   */
  motor.init();
  

  pinMode(ENABLE_MOSFET_AND_DRIVER_POWER, OUTPUT);
  digitalWrite(ENABLE_MOSFET_AND_DRIVER_POWER, LOW);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);


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
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();
#endif


  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  /**
   * @brief move takes the target_velocity, and does the following
   *      get angular velocity
   *          for open loop shaft_velocity = shaft_velocity(no change)
   *      do nothing if enable variable is 0;
   * 
   *      downsampling (optional)
   * 
   *      set internal target variable using target_velocity variable.
   * 
   *      switch(controller){
   *          case MotionControlType::torque:............break;
   *          case MotionControlType::angle:............break;
   *          case MotionControlType::velocity:............break;
   *          case MotionControlType::velocity_openLoop
   *              shaft_velocity_sp = target;
   *              voltage.q = velocityOpenloop(shaft_velocity_sp); returns the voltage that is set to the motor,
   *                velocityOpenLoop() function calculates voltage.q based on the shaft_velocity_sp by calculating
   *                shaft_angle and then normializes it, using shaft_angle = _normalizeAngle(shaft_angle+target_velocity*Ts(time between two move() execution))
   *                now sets the max allowed voltage value voltage_limit using setPhaseVoltage(voltage_limit,0,shaft_angle*pole_pair))
   *                setPhaseVoltage(Uq,Ud,angle)
   *                    switch(foc_modulation){//default value of foc_modulation is sinPWM
   *                        case FOCModulationType::Trapezoid_120.........break;
   *                        case FOCModulationType::Trapezoid_150.........break;
   *                        case FOCModulationType::SinePWM
   *                            normalizes the angle
   *                            calculate Ua, Ub, Uc.
   *                            break;
   *                        case FOCModulationType::SpaceVectorPWM
   *                    }
   *                   set the voltages in driver using driver->setPwm(Ua, Ub, Uc);
   *                now velocityOpenLoop return Uq i.e voltage_limit value.
   *              voltage.d = 0;
   *              break;
   *          case MotionControlType::angle_openloop:............break;
   *      }
   */
  motor.move(target_velocity);

  // user communication
  /**
   * @brief for receiving the input from user and setting it to target variable.
   * 
   */
  command.run();

  // Serial.println(target_velocity);
}
