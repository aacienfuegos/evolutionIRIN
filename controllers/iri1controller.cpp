/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib> 
#include <cstdio>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "encodersensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"


/******************************************************************************/
/******************************************************************************/

extern gsl_rng* rng;
extern long int rngSeed;

const int mapGridX          = 20;
const int mapGridY          = 20;
double    mapLengthX        = 3.0;
double    mapLengthY        = 3.0;
int       robotStartGridX   = 10; 
int       robotStartGridY   = 10;

const   int n=mapGridX; // horizontal size of the map
const   int m=mapGridY; // vertical size size of the map
static  int map[n][m];
static  int onlineMap[n][m];
static  int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static  int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static  int dir_map[n][m]; // map of directions
const   int dir=8; // number of possible directions to go at any position
//if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

#define ERROR_DIRECTION 0.05 
#define ERROR_POSITION  0.02
/******************************************************************************/
/******************************************************************************/

using namespace std;
/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS	5

#define AVOID_PRIORITY 		 0
#define RELOAD_PRIORITY 	 1
#define FORAGE_PRIORITY		 2
#define NAVIGATE_PRIORITY  3
#define GO_GOAL_PRIORITY   4

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.6
/* Threshold to define the battery discharged */
#define BATTERY_THRESHOLD 0.5
/* Threshold to reduce the speed of the robot */
#define NAVIGATE_LIGHT_THRESHOLD 0.9

#define SPEED 500

#define NO_OBSTACLE 0
#define OBSTACLE    1
#define START       2
#define PATH        3
#define END         4
#define NEST        5
#define PREY        6
/******************************************************************************/
/******************************************************************************/

class node
{
  // current position
  int xPos;
  int yPos;
  // total distance already travelled to reach the node
  int level;
  // priority=level+remaining distance estimate
  int priority;  // smaller: higher priority

  public:
  node(int xp, int yp, int d, int p) 
  {xPos=xp; yPos=yp; level=d; priority=p;}

  int getxPos() const {return xPos;}
  int getyPos() const {return yPos;}
  int getLevel() const {return level;}
  int getPriority() const {return priority;}

  void updatePriority(const int & xDest, const int & yDest)
  {
    priority=level+estimate(xDest, yDest)*10; //A*
  }

  // give better priority to going strait instead of diagonally
  void nextLevel(const int & i) // i: direction
  {
    level+=(dir==8?(i%2==0?10:14):10);
  }

  // Estimation function for the remaining distance to the goal.
  const int & estimate(const int & xDest, const int & yDest) const
  {
    static int xd, yd, d;
    xd=xDest-xPos;
    yd=yDest-yPos;         

    // Euclidian Distance
    d=static_cast<int>(sqrt(xd*xd+yd*yd));

    // Manhattan distance
    //d=abs(xd)+abs(yd);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));

    return(d);
  }
};

/******************************************************************************/
/******************************************************************************/

// Determine priority (in the priority queue)
bool operator < ( const node & a, const node & b )
{
  return a.getPriority() > b.getPriority();
}


/******************************************************************************/
/******************************************************************************/
CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	m_nWriteToFile = n_write_to_file;
	
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor (SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);

	
	/* Initialize Motor Variables */
	m_fLeftSpeed  = 0.0;
	m_fRightSpeed = 0.0;

  /* Initialize Inhibitors */
  fBattToForageInhibitor = 1.0;
  fGoalToForageInhibitor = 1.0;

  /* Initialize Activation Table */
	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}
  
  /* Odometry */
  m_nState              = 0;
  m_nPathPlanningStops  = 0;
  m_fOrientation        = 0.0;
  m_vPosition.x         = 0.0;
  m_vPosition.y         = 0.0;

  /* Set Actual Position to robot Start Grid */
  m_nRobotActualGridX = robotStartGridX;
  m_nRobotActualGridY = robotStartGridY;

  /* Init onlineMap */
  for ( int y = 0 ; y < m ; y++ )
    for ( int x = 0 ; x < n ; x++ )
      onlineMap[x][y] = OBSTACLE;

  /* DEBUG */
  PrintMap(&onlineMap[0][0]);
  /* DEBUG */

  /* Initialize status of foraging */
  m_nForageStatus = 0;

  /* Initialize Nest/Prey variables */
  m_nNestGridX  = 0;
  m_nNestGridY  = 0;
  m_nPreyGridX  = 0;
  m_nPreyGridY  = 0;
  m_nPreyFound  = 0;
  m_nNestFound  = 0;

  /* Initialize PAthPlanning Flag*/
  m_nPathPlanningDone = 0;
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
  m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

	if (m_nWriteToFile ) 
	{
	/* INIT: WRITE TO FILES */
	/* Write robot position and orientation */
		FILE* filePosition = fopen("outputFiles/robotPosition", "a");
		fprintf(filePosition,"%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
		fclose(filePosition);

		/* Write robot wheels speed */
		FILE* fileWheels = fopen("outputFiles/robotWheels", "a");
		fprintf(fileWheels,"%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileWheels);
		/* END WRITE TO FILES */
	}


}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Release Inhibitors */
	fBattToForageInhibitor = 1.0;
	fGoalToForageInhibitor = 1.0;
	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);

  /* Execute Behaviors */
	ObstacleAvoidance ( AVOID_PRIORITY );
	GoLoad            ( RELOAD_PRIORITY );

  ComputeActualCell ( GO_GOAL_PRIORITY );
  PathPlanning      ( GO_GOAL_PRIORITY );
  GoGoal            ( GO_GOAL_PRIORITY );
	
  Forage            ( FORAGE_PRIORITY );
	Navigate          ( NAVIGATE_PRIORITY );
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator ( void )
{
  /* Create counter for behaviors */ 
	int       nBehavior;
  /* Create angle of movement */
  double    fAngle = 0.0;
  /* Create vector of movement */
  dVector2  vAngle;
  vAngle.x = 0.0;
  vAngle.y = 0.0;

  /* For every Behavior */
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
	{
    /* If behavior is active */
		if ( m_fActivationTable[nBehavior][2] == 1.0 )
		{
      /* DEBUG */
      printf("Behavior %d: %2f\n", nBehavior, m_fActivationTable[nBehavior][0]);
      /* DEBUG */
      vAngle.x += m_fActivationTable[nBehavior][1] * cos(m_fActivationTable[nBehavior][0]);
      vAngle.y += m_fActivationTable[nBehavior][1] * sin(m_fActivationTable[nBehavior][0]);
		}
	}

  /* Calc angle of movement */
  fAngle = atan2(vAngle.y, vAngle.x);
  /* DEBUG */
  printf("fAngle: %2f\n", fAngle);
  printf("\n");
  /* DEBUG */
  
  if (fAngle > 0)
  {
    m_fLeftSpeed = SPEED*(1 - fmin(fAngle, ERROR_DIRECTION)/ERROR_DIRECTION);
    m_fRightSpeed = SPEED;
  }
  else
  {
    m_fLeftSpeed = SPEED;
    m_fRightSpeed = SPEED*(1 - fmin(-fAngle, ERROR_DIRECTION)/ERROR_DIRECTION);
  }

  if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ObstacleAvoidance ( unsigned int un_priority )
{
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);

	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += prox[i] * cos ( proxDirections[i] );
		vRepelent.y += prox[i] * sin ( proxDirections[i] );

		if ( prox[i] > fMaxProx )
			fMaxProx = prox[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = 1.0; //fMaxProx;

	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_GREEN);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
	}
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/avoidOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
	
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Navigate ( unsigned int un_priority )
{
  /* Direction Angle 0.0 and always active. We set its vector intensity to 0.5 if used */
	m_fActivationTable[un_priority][0] = 0.0;
	m_fActivationTable[un_priority][1] = 0.1;
	m_fActivationTable[un_priority][2] = 1.0;

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}
		
/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoLoad ( unsigned int un_priority )
{
	/* Leer Battery Sensores */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);

	double fMaxLight = 0.0;
	const double* lightDirections = m_seLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
  /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxLight;

	/* If battery below a BATTERY_THRESHOLD */
	if ( battery[0] < BATTERY_THRESHOLD )
	{
    /* Inibit Forage */
		fBattToForageInhibitor = 0.0;
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_RED);
		
    /* Mark behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
	}	

	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Forage ( unsigned int un_priority )
{
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	
	double fMaxLight = 0.0;
	const double* lightDirections = m_seLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	
  /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = 1 - fMaxLight;
  
  /* If with a virtual puck */
	if ( ( groundMemory[0] * fBattToForageInhibitor * fGoalToForageInhibitor ) == 1.0 )
	{
		/* Set Leds to BLUE */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLUE);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
		
	}
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/forageOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, fBattToForageInhibitor, groundMemory[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ComputeActualCell ( unsigned int un_priority )
{
	/* Leer Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Suelo Memory */
  double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
  
  CalcPositionAndOrientation (encoder);

  /* DEBUG */
  //printf("POS: X: %2f, %2f\r", m_vPosition.x, m_vPosition.y );
  /* DEBUG */

  /* Calc increment of position, correlating grid and metrics */
  double fXmov = mapLengthX/((double)mapGridX);
  double fYmov = mapLengthY/((double)mapGridY);
  
  /* Compute X grid */
  double tmp = m_vPosition.x;
  tmp += robotStartGridX * fXmov + 0.5*fXmov;
  m_nRobotActualGridX = (int) (tmp/fXmov);
  
  /* Compute Y grid */
  tmp = -m_vPosition.y;
  tmp += robotStartGridY * fYmov + 0.5*fYmov;
  m_nRobotActualGridY = (int) (tmp/fYmov);
  
  
  /* DEBUG */
  printf("GRID: X: %d, %d\n", m_nRobotActualGridX, m_nRobotActualGridY);
  /* DEBUG */
  
  /* Update no-obstacles on map */
  if (  onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] != NEST &&
        onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] != PREY )
    onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = NO_OBSTACLE;
 
  /* If looking for nest and arrived to nest */
  if ( m_nForageStatus == 1 && groundMemory[0] == 0)
  {
    /* update forage status */
    m_nForageStatus = 0;
    /* Asumme Path Planning is done */
    m_nPathPlanningDone = 0;
    /* Restart PathPlanning state */
    m_nState = 0;
    /* Mark nest on map */
    onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = NEST;
    /* Flag that nest was found */
    m_nNestFound = 1;
    /* Update nest grid */
    m_nNestGridX = m_nRobotActualGridX;
    m_nNestGridY = m_nRobotActualGridY;
    /* DEBUG */
    PrintMap(&onlineMap[0][0]);
    /* DEBUG */
  }//end looking for nest
  
  /* If looking for prey and prey graspped */
  else if ( m_nForageStatus == 0 && groundMemory[0] == 1)
  {
    /* Update forage Status */
    m_nForageStatus = 1;
    /* Asumme Path Planning is done */
    m_nPathPlanningDone = 0;
    /* Restart PathPlanning state */
    m_nState = 0;
    /* Mark prey on map */
    onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = PREY;
    /* Flag that nest was found */
    m_nPreyFound = 1;
    /* Update nest grid */
    m_nPreyGridX = m_nRobotActualGridX;
    m_nPreyGridY = m_nRobotActualGridY;
    /* DEBUG */
    PrintMap(&onlineMap[0][0]);
    /* DEBUG */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PathPlanning ( unsigned int un_priority )
{
  /* Clear Map */
  for ( int y = 0 ; y < m ; y++ )
    for ( int x = 0 ; x < n ; x++ )
      map[x][y]= NO_OBSTACLE;

  if ( m_nNestFound == 1 && m_nPreyFound == 1 && m_nPathPlanningDone == 0)
  {
    m_nPathPlanningStops=0;
    m_fActivationTable[un_priority][2] = 1;
    
    /* Obtain start and end desired position */
    int xA, yA, xB, yB;
    if ( m_nForageStatus == 1)
    {
      xA=m_nRobotActualGridX;
      yA=m_nRobotActualGridY;
      xB=m_nNestGridX;
      yB=m_nNestGridY;
    }
    else
    {
      xA=m_nRobotActualGridX;
      yA=m_nRobotActualGridY;
      xB=m_nPreyGridX;
      yB=m_nPreyGridY;
    }

    /* DEBUG */
    printf("START: %d, %d - END: %d, %d\n", xA, yA, xB, yB);
    /* DEBUG */

    /* Obtain Map */
    for ( int y = 0 ; y < m ; y++ )
      for ( int x = 0 ; x < n ; x++ )
        if (onlineMap[x][y] != NO_OBSTACLE && onlineMap[x][y] != NEST && onlineMap[x][y] != PREY)
          map[x][y] = OBSTACLE;

    
    /* Obtain optimal path */
    string route=pathFind(xA, yA, xB, yB);
    /* DEBUG */
    if(route=="") cout<<"An empty route generated!"<<endl;
    cout << "Route:" << route << endl;
    printf("route Length: %d\n", route.length());
    /* DEBUG */

    /* Obtain number of changing directions */
    for (int i = 1 ; i < route.length() ; i++)
      if (route[i-1] != route[i])
        m_nPathPlanningStops++;
   
    /* Add last movement */
    m_nPathPlanningStops++;
    /* DEBUG */
    printf("STOPS: %d\n", m_nPathPlanningStops);
    /* DEBUG */

    /* Define vector of desired positions. One for each changing direction */
    m_vPositionsPlanning = new dVector2[m_nPathPlanningStops]; 

    /* Calc increment of position, correlating grid and metrics */
    double fXmov = mapLengthX/mapGridX;
    double fYmov = mapLengthY/mapGridY;

    /* Get actual position */
    dVector2 actualPos;
    //actualPos.x = robotStartGridX * fXmov;
    actualPos.x = m_nRobotActualGridX * fXmov;
    //actualPos.y = robotStartGridY * fYmov;
    actualPos.y = m_nRobotActualGridY * fYmov;

    /* Fill vector of desired positions */
    int stop = 0;
    int counter = 0;
    /* Check the route and obtain the positions*/
    for (int i = 1 ; i < route.length() ; i++)
    {
      /* For every position in route, increment countr */
      counter++;
      /* If a direction changed */
      if ((route[i-1] != route[i])) 
      {
        /* Obtain the direction char */
        char c;
        c = route.at(i-1);

        /* Calc the new stop according to actual position and increment based on the grid */
        m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov*dx[atoi(&c)];
        m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov*dy[atoi(&c)];

        /* Update position for next stop */
        actualPos.x = m_vPositionsPlanning[stop].x;
        actualPos.y = m_vPositionsPlanning[stop].y;

        /* Increment stop */
        stop++;
        /* reset counter */
        counter = 0;
      }

      /* If we are in the last update, calc last movement */
      if (i==(route.length()-1))
      {
        /* Increment counter */
        counter++;
        /* Obtain the direction char */
        char c;
        c = route.at(i);

        /* DEBUG */
        //printf("COUNTER: %d, CHAR: %c\n", counter, c);
        /* END DEBUG */

        /* Calc the new stop according to actual position and increment based on the grid */
        m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov*dx[atoi(&c)];// - robotStartGridX * fXmov;
        m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov*dy[atoi(&c)];// - robotStartGridY * fYmov;

        /* Update position for next stop */
        actualPos.x = m_vPositionsPlanning[stop].x;
        actualPos.y = m_vPositionsPlanning[stop].y;

        /* Increment stop */
        stop++;
        /* reset counter */
        counter = 0;
      }

    }

    /* DEBUG */
    if(route.length()>0)
    {
      int j; char c;
      int x=xA;
      int y=yA;
      map[x][y]=START;
      for ( int i = 0 ; i < route.length() ; i++ )
      {
        c = route.at(i);
        j = atoi(&c); 
        x = x+dx[j];
        y = y+dy[j];
        map[x][y] = PATH;
      }
      map[x][y]=END;

      PrintMap(&onlineMap[0][0]);
      printf("\n\n");
      PrintMap(&map[0][0]);
    }
    /* END DEBUG */

    /* DEBUG */
    //printf("Start: %2f, %2f\n", m_nRobotActualGridX * fXmov, m_nRobotActualGridY * fXmov);
    //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
    /* END DEBUG */

    /* Convert to simulator coordinates */
    for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    {
      m_vPositionsPlanning[i].x -= (mapGridX * fXmov)/2;
      m_vPositionsPlanning[i].y -= (mapGridY * fYmov)/2;
      m_vPositionsPlanning[i].y = - m_vPositionsPlanning[i].y;
    }
    /* DEBUG */
    //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
    /* END DEBUG */


    /* Convert to robot coordinates. FAKE!!.
     * Notice we are only working with initial orientation = 0.0 */
    for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    {
      /* Traslation */ 
      m_vPositionsPlanning[i].x -= ( (robotStartGridX * fXmov) - (mapGridX * fXmov)/2 );
      m_vPositionsPlanning[i].y += ( (robotStartGridY * fXmov) - (mapGridY * fYmov)/2);
      /* Rotation */
      //double compass = m_pcEpuck->GetRotation();
      //m_vPositionsPlanning[i].x = m_vPositionsPlanning[i].x * cos (compass) - m_vPositionsPlanning[i].y  * sin(compass);
      //m_vPositionsPlanning[i].y = m_vPositionsPlanning[i].x * sin (compass) + m_vPositionsPlanning[i].y  * cos(compass);
    }
    /* DEBUG */
    //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
    /* END DEBUG */

    m_nPathPlanningDone = 1;
  }
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoGoal ( unsigned int un_priority )
{

  if ( ((m_nNestFound * fBattToForageInhibitor) == 1 ) && ( (m_nPreyFound *  fBattToForageInhibitor )== 1 ) )
  {
    /* Enable Inhibitor to Forage */
    fGoalToForageInhibitor = 0.0;

    /* If something not found at the end of planning, reset plans */
    if (m_nState >= m_nPathPlanningStops )
    {
      printf(" --------------- LOST!!!!!!!! --------------\n");
      m_nNestFound  = 0;
      m_nPreyFound  = 0;
      m_nState      = 0;
      return;
    }

    /* DEBUG */
    printf("PlanningX: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].x, m_vPosition.x );
    printf("PlanningY: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].y, m_vPosition.y );
    /* DEBUG */
    
    double fX = (m_vPositionsPlanning[m_nState].x - m_vPosition.x);
    double fY = (m_vPositionsPlanning[m_nState].y - m_vPosition.y);
    double fGoalDirection = 0;

    /* If on Goal, return 1 */
    if ( ( fabs(fX) <= ERROR_POSITION ) && ( fabs(fY) <= ERROR_POSITION ) )
      m_nState++;

    fGoalDirection = atan2(fY, fX);

    /* Translate fGoalDirection into local coordinates */
    fGoalDirection -= m_fOrientation;
    /* Normalize Direction */
    while ( fGoalDirection > M_PI) fGoalDirection -= 2 * M_PI;
    while ( fGoalDirection < -M_PI) fGoalDirection+=2*M_PI;

    m_fActivationTable[un_priority][0] = fGoalDirection;
    m_fActivationTable[un_priority][1] = 1;
    m_fActivationTable[un_priority][2] = 1;
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::CalcPositionAndOrientation (double *f_encoder)
{
  /* DEBUG */ 
  //printf("Encoder: %2f, %2f\n", f_encoder[0], f_encoder[1]);
  /* DEBUG */ 
  
  /* Remake kinematic equations */
  double fIncU = (f_encoder[0]+ f_encoder[1] )/ 2;
  double fIncTetha = (f_encoder[1] - f_encoder[0])/ CEpuck::WHEELS_DISTANCE;

  /* Substitute arc by chord, take care of 0 division */
  if (fIncTetha != 0.0)
    fIncU = ((f_encoder[0]/fIncTetha)+(CEpuck::WHEELS_DISTANCE/2))* 2.0 * sin (fIncTetha/2.0);

  /* Update new Position */
  m_vPosition.x += fIncU * cos(m_fOrientation + fIncTetha/2);
  m_vPosition.y += fIncU * sin(m_fOrientation + fIncTetha/2);
  
  /* Update new Orientation */
  m_fOrientation += fIncTetha;

  /* Normalize Orientation */
  while(m_fOrientation < 0) m_fOrientation += 2*M_PI;
  while(m_fOrientation > 2*M_PI) m_fOrientation -= 2*M_PI;
}

/******************************************************************************/
/******************************************************************************/

// A-star algorithm.
// The route returned is a string of direction digits.
string CIri1Controller::pathFind( const int & xStart, const int & yStart, 
    const int & xFinish, const int & yFinish )
{
  static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
  static int pqi; // pq index
  static node* n0;
  static node* m0;
  static int i, j, x, y, xdx, ydy;
  static char c;
  pqi=0;

  // reset the node maps
  for ( y=0 ; y < m ; y++ )
  {
    for ( x = 0 ; x < n ; x++ )
    {
      closed_nodes_map[x][y]=0;
      open_nodes_map[x][y]=0;
    }
  }

  // create the start node and push into list of open nodes
  n0=new node(xStart, yStart, 0, 0);
  n0->updatePriority(xFinish, yFinish);
  pq[pqi].push(*n0);
  //open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

  // A* search
  while(!pq[pqi].empty())
  {
    // get the current node w/ the highest priority
    // from the list of open nodes
    n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
        pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

    x=n0->getxPos(); y=n0->getyPos();

    pq[pqi].pop(); // remove the node from the open list
    open_nodes_map[x][y]=0;
    // mark it on the closed nodes map
    closed_nodes_map[x][y]=1;

    // quit searching when the goal state is reached
    //if((*n0).estimate(xFinish, yFinish) == 0)
    if(x==xFinish && y==yFinish) 
    {
      // generate the path from finish to start
      // by following the directions
      string path="";
      while(!(x==xStart && y==yStart))
      {
        j=dir_map[x][y];
        c='0'+(j+dir/2)%dir;
        path=c+path;
        x+=dx[j];
        y+=dy[j];
      }

      // garbage collection
      delete n0;
      // empty the leftover nodes
      while(!pq[pqi].empty()) pq[pqi].pop();           
      return path;
    }

    // generate moves (child nodes) in all possible directions
    for ( i = 0 ; i < dir ; i++ )
    {
      xdx=x+dx[i]; ydy=y+dy[i];

      if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1 
            || closed_nodes_map[xdx][ydy]==1))
      {
        // generate a child node
        m0=new node( xdx, ydy, n0->getLevel(), 
            n0->getPriority());
        m0->nextLevel(i);
        m0->updatePriority(xFinish, yFinish);

        // if it is not in the open list then add into that
        if(open_nodes_map[xdx][ydy]==0)
        {
          open_nodes_map[xdx][ydy]=m0->getPriority();
          pq[pqi].push(*m0);
          // mark its parent node direction
          dir_map[xdx][ydy]=(i+dir/2)%dir;
        }
        else if(open_nodes_map[xdx][ydy]>m0->getPriority())
        {
          // update the priority info
          open_nodes_map[xdx][ydy]=m0->getPriority();
          // update the parent direction info
          dir_map[xdx][ydy]=(i+dir/2)%dir;

          // replace the node
          // by emptying one pq to the other one
          // except the node to be replaced will be ignored
          // and the new node will be pushed in instead
          while(!(pq[pqi].top().getxPos()==xdx && 
                pq[pqi].top().getyPos()==ydy))
          {                
            pq[1-pqi].push(pq[pqi].top());
            pq[pqi].pop();       
          }
          pq[pqi].pop(); // remove the wanted node

          // empty the larger size pq to the smaller one
          if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
          while(!pq[pqi].empty())
          {                
            pq[1-pqi].push(pq[pqi].top());
            pq[pqi].pop();       
          }
          pqi=1-pqi;
          pq[pqi].push(*m0); // add the better node instead
        }
        else delete m0; // garbage collection
      }
    }
    delete n0; // garbage collection
  }
  return ""; // no route found
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PrintMap ( int *print_map )
{
      
  /* DEBUG */
  for ( int x = 0 ; x < n ; x++ )
  {
    for ( int y = 0 ; y < m ; y++ )
    {
      if ( print_map[y*n + x] == 0 )
        cout<<".";
      else if(print_map[y*n+x]==1)
        cout<<"O"; //obstacle
      else if(print_map[y*n+x]==2)
        cout<<"S"; //start
      else if(print_map[y*n+x]==3)
        cout<<"R"; //route
      else if(print_map[y*n+x]==4)
        cout<<"F"; //finish
      else if(print_map[y*n+x]==5)
        cout<<"N"; //finish
      else if(print_map[y*n+x]==6)
        cout<<"P"; //finish
    }
    cout<<endl;
  }
  /* DEBUG */
}
