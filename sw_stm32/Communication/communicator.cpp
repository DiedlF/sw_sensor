/***********************************************************************//**
 * @file		communicator.cpp
 * @brief		Main module for data acquisition and signal output
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/
#include "system_configuration.h"
#include "main.h"

#include <FreeRTOS_wrapper.h>
#include "navigator.h"
#include "variometer.h"
#include "NMEA_format.h"
#include "common.h"
#include "organizer.h"
#include "CAN_output.h"
#include "CAN_output_task.h"
#include "D_GNSS_driver.h"
#include "GNSS_driver.h"
#include "CAN_distributor.h"
#include "uSD_handler.h"
#include "persistent_data.h"
#include "communicator.h"
#include "system_state.h"

extern "C" void sync_logger (void);

COMMON Semaphore setup_file_handling_completed(1,0,(char *)"SETUP");

COMMON output_data_t __ALIGNED(1024) output_data = { 0 };
COMMON GNSS_type GNSS (output_data.obs.c);

COMMON Queue < communicator_command_t> communicator_command_queue(2);

extern RestrictedTask NMEA_task;
extern RestrictedTask communicator_task;
extern bool landing_detected;

static ROM bool TRUE=true;
static ROM bool FALSE=false;

typedef struct
{
  float3vector * source;
  float3vector * destination;
  float3vector sum;
  unsigned counter;
} vector_average_organizer_t;

void report_horizon_avalability( void)
{
  if( configuration (HORIZON) )
	update_system_state_clear( HORIZON_NOT_AVAILABLE);
  else
	update_system_state_set( HORIZON_NOT_AVAILABLE);
}

static ROM TaskParameters_t usart_3_task_param =
  {
      USART_3_runnable,
      "USART3",
      256,
      (void *)&TRUE,
      (STANDARD_TASK_PRIORITY+1) | portPRIVILEGE_BIT, // first: start privileged
      0,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { USART_3_RX_buffer, USART_3_RX_BUFFER_SIZE_ROUND_UP, portMPU_REGION_READ_ONLY },
      { 0, 0, 0 }
    }
  };

void communicator_runnable (void*)
{
  vector_average_organizer_t vector_average_organizer={0};
  vector_average_collection_t vector_average_collection={0};

  bool have_first_GNSS_fix = false;
  bool fine_tune_sensor_attitude = false;

  // wait until configuration file read if one is given
  setup_file_handling_completed.wait();

  organizer_t organizer;

  organizer.initialize_before_measurement();
  report_horizon_avalability();

  uint16_t GNSS_count = 0;

  GNSS_configration_t GNSS_configuration =
      (GNSS_configration_t) round(configuration (GNSS_CONFIGURATION));

  uint8_t count_10Hz = 1; // de-synchronize CAN output by 1 cycle

  GNSS.clear_sat_fix_type();

  switch (GNSS_configuration)
    {
    case GNSS_M9N:
      {
	TaskParameters_t parameters = usart_3_task_param;
	parameters.pvParameters = (void *)&FALSE;

	acquire_privileges();
	RestrictedTask t( parameters);
	drop_privileges();

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);

	organizer.update_GNSS_data (output_data.obs.c);
	update_system_state_set( GNSS_AVAILABLE);
	GNSS_new_data_ready = false;
      }
      break;
    case GNSS_F9P_F9H: // extra task for 2nd GNSS module required
      {
	  {
	    TaskParameters_t parameters = usart_3_task_param;
	    parameters.pvParameters = (void *)&FALSE;

	    acquire_privileges();
	    RestrictedTask t( parameters);
	    drop_privileges();

	    Task usart4_task (USART_4_runnable, "D-GNSS", 256, 0, STANDARD_TASK_PRIORITY + 1);
	  }

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);

	organizer.update_GNSS_data (output_data.obs.c);
	update_system_state_set( GNSS_AVAILABLE | D_GNSS_AVAILABLE);
	GNSS_new_data_ready = false;
      }
      break;
    case GNSS_F9P_F9P: // no extra task for 2nd GNSS module
      {
	acquire_privileges();
	RestrictedTask t( usart_3_task_param);
	drop_privileges();

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);

	organizer.update_GNSS_data (output_data.obs.c);
	update_system_state_set( GNSS_AVAILABLE | D_GNSS_AVAILABLE);
	GNSS_new_data_ready = false;
      }
      break;
    default:
      ASSERT(false);
    }

  for( int i=0; i<100; ++i) // wait 1 s until measurement stable
    notify_take (true);

  organizer.initialize_after_first_measurement(output_data);

  NMEA_task.resume();
  CAN_task.resume();

  unsigned synchronizer_10Hz = 10; // re-sampling 100Hz -> 10Hz
  unsigned GNSS_watchdog = 0;

  // this is the MAIN data acquisition and processing loop
  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

      if (GNSS_new_data_ready) // triggered after 75ms or 200ms, GNSS-dependent
	{
	  organizer.update_GNSS_data (output_data.obs.c);
	  GNSS_new_data_ready = false;
	  update_system_state_set( GNSS_AVAILABLE);
	  if( GNSS_configuration > GNSS_M9N)
	    update_system_state_set( D_GNSS_AVAILABLE);

	  if( (have_first_GNSS_fix == false) && ((output_data.obs.c.sat_fix_type & SAT_FIX) != 0))
	    {
	      have_first_GNSS_fix = true;
	      organizer.update_magnetic_induction_data( output_data.obs.c.latitude, output_data.obs.c.longitude);
	    }

	  GNSS_watchdog=0;
	}
      else
	{
	  if( GNSS_watchdog < 20)
	      ++GNSS_watchdog;
	  else // we got no data form GNSS receiver
	    {
	      output_data.obs.c.sat_fix_type = SAT_FIX_NONE;
	      update_system_state_clear( GNSS_AVAILABLE | D_GNSS_AVAILABLE);
	    }
	}

      organizer.on_new_pressure_data( output_data.obs.m.static_pressure, output_data.obs.m.pitot_pressure);
      organizer.update_at_100_Hz(output_data);

      // service external commands if any
      communicator_command_t command;
      if( communicator_command_queue.receive(command, 0))
	{
	  switch( command)
	  {
	    case MEASURE_CALIB_LEFT:
	      vector_average_organizer.source=&(output_data.obs.m.acc);
	      vector_average_organizer.destination=&(vector_average_collection.acc_observed_left);
	      vector_average_organizer.destination->zero();
	      vector_average_organizer.counter=VECTOR_AVERAGE_COUNT_SETUP;
	      break;
	    case MEASURE_CALIB_RIGHT:
	      vector_average_organizer.source=&(output_data.obs.m.acc);
	      vector_average_organizer.destination=&(vector_average_collection.acc_observed_right);
	      vector_average_organizer.destination->zero();
	      vector_average_organizer.counter=VECTOR_AVERAGE_COUNT_SETUP;
	      break;
	    case MEASURE_CALIB_LEVEL:
	      vector_average_organizer.source=&(output_data.obs.m.acc);
	      vector_average_organizer.destination=&(vector_average_collection.acc_observed_level);
	      vector_average_organizer.destination->zero();
	      vector_average_organizer.counter=VECTOR_AVERAGE_COUNT_SETUP;
	      break;
	    case SET_SENSOR_ROTATION:

	      // make sure that we have all three measurements
	      if( vector_average_collection.acc_observed_left.abs() < 0.001f)
		break;
	      if( vector_average_collection.acc_observed_right.abs() < 0.001f)
		break;
	      if( vector_average_collection.acc_observed_level.abs() < 0.001f)
		break;

	      organizer.update_sensor_orientation_data( vector_average_collection);
	      organizer.initialize_before_measurement();
	      report_horizon_avalability();
	      break;
	    case FINE_TUNE_CALIB:  // names "straight flight" in Larus Display Menu
	      vector_average_organizer.source=&(output_data.obs.m.acc);
	      vector_average_organizer.destination=&(vector_average_collection.acc_observed_level);
	      vector_average_organizer.destination->zero();
	      vector_average_organizer.counter=VECTOR_AVERAGE_COUNT_SETUP;
	      fine_tune_sensor_attitude = true;
	      break;

	    case SOME_EEPROM_VALUE_HAS_CHANGED:
	      organizer.initialize_before_measurement();
	      report_horizon_avalability();
	      break;

	    case NO_COMMAND:
	      break;
	  }
	}

      if( vector_average_organizer.counter != 0)
	{
	  vector_average_organizer.sum += *(vector_average_organizer.source);
	  -- vector_average_organizer.counter;

	  // if measurement complete now
	  if( vector_average_organizer.counter == 0)
	    {
	      float inverse_count = 1.0f / VECTOR_AVERAGE_COUNT_SETUP;
	      *(vector_average_organizer.destination) = vector_average_organizer.sum * inverse_count;

	      // in this case we do not wait for another command but re-calculate immediately
	      if( fine_tune_sensor_attitude)
		{
		  fine_tune_sensor_attitude=false;
		  organizer.fine_tune_sensor_orientation( vector_average_collection);
		  organizer.initialize_before_measurement();
		  report_horizon_avalability();
		}
	    }
	}

      --synchronizer_10Hz;
      if( synchronizer_10Hz == 0)
	{
	  synchronizer_10Hz = 10;

	  landing_detected |= organizer.update_at_10Hz (output_data);
//	  if( landing_detected) // todo patch
	  extern bool user_initiated_reset;
	  if( user_initiated_reset)
	    {
	      user_initiated_reset = false; // todo patch

	      organizer.cleanup_after_landing();
	      landing_detected = false;
	    }
	}

      // service the GNSS LED
      ++GNSS_count;
      GNSS_count &= 0xff;

      if( vector_average_organizer.counter != 0)
	{
	  HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_SET);
	}
      else
       switch( GNSS_configuration)
      {
	case GNSS_F9P_F9H:
	case GNSS_F9P_F9P:
	  switch(output_data.obs.c.sat_fix_type)
	  {
	    case SAT_FIX:
		  HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
			((GNSS_count & 0xe0) == 0xe0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	    break;
	    case SAT_HEADING | SAT_FIX:
		  HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
		      ((GNSS_count & 0x80) && (GNSS_count & 0x20)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	    break;
	    default:
		HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	      break;
	  }
	  break;
	case GNSS_M9N:
	  if(output_data.obs.c.sat_fix_type == SAT_FIX)
	    HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
	        ((GNSS_count & 0xe0) == 0xe0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  else
	    HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  break;
	default:
	  ASSERT( false);
	  break;
      }

      // service the red error LED
      HAL_GPIO_WritePin ( LED_ERROR_GPIO_Port, LED_ERROR_Pin,
	    essential_sensors_available( GNSS_configuration > GNSS_M9N) ? GPIO_PIN_RESET : GPIO_PIN_SET);

      if (++count_10Hz >= 10) // resample 100Hz -> 10Hz
	{
	  count_10Hz = 0;
	  trigger_CAN ();
	}

      organizer.report_data ( output_data);
      sync_logger (); // kick logger @ 100 Hz
    }
}

#define STACKSIZE 2048
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  { communicator_runnable, "COM",
  STACKSIZE, 0,
  COMMUNICATOR_PRIORITY, stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { (void *)0x080C0000, 0x00040000, portMPU_REGION_READ_ONLY }, // EEPROM
      { &temporary_mag_calculation_data, 8192, portMPU_REGION_READ_WRITE}
    }
  };

COMMON RestrictedTask communicator_task (p);

void
sync_communicator (void) // global synchronization service function
{
  communicator_task.notify_give ();
}
