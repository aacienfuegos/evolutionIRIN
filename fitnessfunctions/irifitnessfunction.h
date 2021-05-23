#ifndef IRIFITNESSFUNCTION_H_
#define IRIFITNESSFUNCTION_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>
#include "general.h"
using namespace std;

class CIriFitnessFunction;

#include "fitnessfunction.h"
#include "simulator.h"
#include "sensor.h"
#include "batterysensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "lightsensor.h"
#include "bluelightsensor.h"
#include "contactsensor.h"

/******************************************************************************/
/******************************************************************************/

class CIriFitnessFunction : public CFitnessFunction
{
public:
    CIriFitnessFunction(const char* pch_name, CSimulator* pc_simulator,
                                    unsigned int un_collisions_allowed_per_epuck);
		~CIriFitnessFunction();
    virtual double GetFitness();
		virtual void SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval);

protected:
		unsigned int m_unNumberOfSteps;
		double 			m_fComputedFitness;
		unsigned int m_unState;

		CEpuck* m_pcEpuck;
		CBatterySensor* m_seBattery;

		unsigned int m_unCollisionsNumber;
		unsigned int m_unGreyFlag;
		unsigned int m_unGreyCounter;

};

/******************************************************************************/
/******************************************************************************/

#endif

