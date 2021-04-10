#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_wrtie_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
		/* ROBOT */
    CEpuck* m_pcEpuck;
   
	 	/* SENSORS */
		CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;   
		CEncoderSensor* m_seEncoder;  

		/* Global Variables */
		double 		m_fLeftSpeed;
		double 		m_fRightSpeed;
		double**	m_fActivationTable;
		int 			m_nWriteToFile;
		double 		m_fTime;
    double    fBattToForageInhibitor;
    double    fGoalToForageInhibitor;
   
    /* Odometry */
    float     m_fOrientation; 
    dVector2  m_vPosition;
    int       m_nState;
    dVector2  *m_vPositionsPlanning;
    int       m_nPathPlanningStops;
    int       m_nRobotActualGridX;
    int       m_nRobotActualGridY;

    int       m_nForageStatus;
    
    int       m_nNestFound;
    int       m_nNestGridX;
    int       m_nNestGridY;
    
    int       m_nPreyFound;
    int       m_nPreyGridX;
    int       m_nPreyGridY;

    int       m_nPathPlanningDone;
		/* Functions */

		void ExecuteBehaviors   ( void );
		void Coordinator        ( void );

    void CalcPositionAndOrientation ( double *f_encoder );
    string  pathFind                ( const int &xStart, const int &yStart, const int &xFinish, const int &yFinish );
  
    void PrintMap ( int *print_map  );
    /* Behaviors */
		void ObstacleAvoidance  ( unsigned int un_priority );
		void Navigate           ( unsigned int un_priority );
		void GoLoad             ( unsigned int un_priority );
		void Forage             ( unsigned int un_priority );

    void ComputeActualCell  ( unsigned int un_priority );
    void PathPlanning       ( unsigned int un_priority );
		void GoGoal             ( unsigned int un_priority );
};

#endif
