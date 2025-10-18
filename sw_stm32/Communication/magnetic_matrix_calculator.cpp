
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "embedded_math.h"
#include "soft_iron_compensator.h"

//#include "magnetic_matrix_calculator.h"
//#include "magnetic_induction_report.h"
//COMMON compass_calibrator_3D_t compass_calibrator_3D;

COMMON Semaphore calculation_trigger;

void trigger_soft_iron_compensator_calculation(void)
{
  calculation_trigger.signal();
}

void trigger_compass_calibrator_3D_calculation(void)
{
  calculation_trigger.signal();
}

static void magnetic_calculator_runnable ( void *)
{
  while( true)
    {
      calculation_trigger.wait();
      soft_iron_compensator.calculate();
    }
}

#define STACKSIZE 256
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];
soft_iron_compensator_t __ALIGNED( SOFT_IRON_DATA_SIZE) soft_iron_compensator;

static TaskParameters_t p =
{
  magnetic_calculator_runnable,
  "MAG_CALC",
  STACKSIZE,
  0,
  MAG_CALCULATOR_PRIORITY,
  stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { (void *)&soft_iron_compensator, SOFT_IRON_DATA_SIZE, portMPU_REGION_READ_WRITE},
      { 0, 0, 0}
    }
};

static RestrictedTask magnetic_calculator_task (p);

