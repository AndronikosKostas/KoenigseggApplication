/**
 * This file contains functions to..
 *
 */

#include "hwio.h"

/* Defines and types ---------------------------------- */
#define MIN 2
#define MAX 8

// Struct for storing PID controller state
typedef struct {
    // controler gains //
    float kp, ki, kd;  

    // controller memory //
    float integral;    
    float prevError;  

    // output limits //
    float min;
    float max;

    // controller output //
    float out;

} pidController;


/* Function prototypes -------------------------------- */

void applicationLoop10ms(void);

/**
 * Apply the output and the appropriate direction to the actuator
 * \param[in] direction Desired direction for actuator
 * \param[in] output Output to be sent to the actuator from the controller
 */
void applyOutputToActuator(float output, tActuatorDirection direction);

/**
 * Init the PID Controller
 * \param[in] pid Instance of the pidController struct
 * \param[in] kp Proportial gain of the controller
 * \param[in] ki Integration gain of the controller
 * \param[in] kd Derivative gain of the controller
 */
void pidInit(pidController *pid, double kp, double ki, double kd);

/**
 * Set direction of actuator
 * \param[in] pid Instance of the pidController struct
 * \param[in] setpoint Ideal position
 * \param[in] measurement Measurement from the sensor
 * \param[in] dt Time step
 * \return the output signal that will be an input to the actuator
 */
float pidUpdate(pidController *pid, float setpoint, float measurement, float dt);

/**
 * Transformation of the volts to percentage
 * \param[in] sensorValueInVolts
 * \return the corresponding percentage
 */
float sensorVoltageToPercentage(float sensorValueInVolts);

/* External variables and functions ------------------- */


/* Global variables ----------------------------------- */

uint8_t requestedPositionPercent;

/* Static variables ----------------------------------- */

/**
 * Application loop called every 10ms
 */


// main // 

int main(void) {

    // pidController instance // 
    pidController pid;
    // Initialize PID controller with constants (random) //
    pidInit(&pid, 0.7, 0.2, 0.3);  

    // get the requested position //
    float setpoint = requestedPositionPercent;   
    float measured_value = 0;
    // Time step of 10ms //
    float dt = 0.01;  

    // No need for any actuation yet //
    hwioSetActuatorEnable(ACTUATOR_OFF);
    tActuatorDirection dir;

    applicationLoop10ms()
    {
        // get the requested position //
        requestedPositionPercent = getRequestedPosition();
        // calculate the feedback from sensor //
        T_VOLT_FLT sensorVolts =  hwioGetSensorVoltage();
        // convert the measurement of the sensor to percentage //
        float measurement =  sensorVoltageToPercentage(sensorVolts);
        // calculation of the output //
        float output = pid_update(&pid, requestedPositionPercent, measurement, dt);
        if(output < pid -> out)
            dir = DIR_RETRACT;
        else{
            dir = DIR_EXTEND;
        }
        pid -> out = output ;
        // send the output to the actuator //
        applyOutputToActuator(output, dir);
  }
    return 0;
}

// Initialization of the PID controller //
void pidInit(pidController *pid, double kp, double ki, double kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->integral = 0;
  pid->prevError = 0;

  pid->min = 0; // 0 % 
  pid->max = 1; // 100 % 

  pid->out = 0;
}

// Update of the PID controller //
float pidUpdate(pidController *pid, float setpoint, float measurement, float dt) {

    // Calculate error
    float error = setpoint - measurement;

    // Update integral
    pid->integral += error * dt;

    // Calculate derivative
    derivative = (error - pid->prev_error) / dt;

    // Calculate output
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // Store error for next iteration
    pid -> prevError = error;
    
    return output;
}

// Sensor transfer function //
float sensorVoltageToPercentage(float sensorValueInVolts)
{
    if(sensorValueInVolts < MIN)
        return 0;
    else if(sensorValueInVolts > MAX)
        return 1;
    else{
        return (sensorValueInVolts* 0.4 + 0.2) / 10 ; // random transformation
    }
}

// parsing the output to the actuator //
void applyOutputToActuator(float output, tActuatorDirection direction)
{
    // enable the actuator //
    hwioSetActuatorEnable(ACTUATOR_ON);
    // set the direction // 
    hwioSetActuatorDirection(direction);

    // here would be a function that takes as an argument the output and actually turns the mechanism (driver of the mechanism) //
    // wait for this function to send an ISR that will signals the end //

    // disable the actuator //
    hwioSetActuatorEnable(ACTUATOR_OFF);
}

void applicationLoop10ms()
{
    // Timer ISR each 10ms
}

