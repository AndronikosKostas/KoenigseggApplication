/**
 * \file        hwio.h
 * \brief       Interface to hardware I/O layer
 * \details     Detailed file description
 *
 */

// Defines and types ------------------------------------
typedef enum
{
    DIR_RETRACT = 0,
    DIR_EXTEND
} tActuatorDirection;

typedef enum
{
    ACTUATOR_OFF = 0,
    ACTUATOR_ON
} tActuatorEnable;

typedef float T_VOLT_FLT;

// Function declarations --------------------------------

/**
 * Get the sensor value
 * \return Sensor value in Volts
 */
T_VOLT_FLT hwioGetSensorVoltage(void);

/**
 * Set direction of actuator
 * \param[in] aDir Desired direction for actuator
 */
void hwioSetActuatorDirection(tActuatorDirection aDir);

/**
 * Set enable status of actuator
 * \param[in] aEn Desired enabled status for actuator
 */
void hwioSetActuatorEnable(tActuatorEnable aEn);

/**
 * Get requested position from the mechanism
 * \return Requested position in percent
 */
uint8_t getRequestedPosition(void);
