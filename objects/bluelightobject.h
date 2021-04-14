#ifndef BLUELIGHTOBJECT_H_
#define BLUELIGHTOBJECT_H_

/******************************************************************************/
/******************************************************************************/

#include "geometry.h"

class CBlueLightObject;
	
/******************************************************************************/
/******************************************************************************/

class CBlueLightObject : public CGeometry
{
public:
    CBlueLightObject(const char* pch_name);
    ~CBlueLightObject();

	void SetColor(float fGrey);
	void GetColor(float *fGrey);

	void SetHeight ( float fHeight);
	void GetHeight ( float *fHeight);
	
	void SetCenter(dVector2 dCenter);
	void GetCenter(dVector2 *dCenter);

	void SetIntRadius(float fRadius);
	void GetIntRadius(float *fRadius);

	void SetExtRadius(float fRadius);
	void GetExtRadius(float *fRadius);
	
	void Switch 	( int n_value );
	int GetStatus ( void );

	int GetTiming ( unsigned int n_step_number );

	void SetVaccines(int nVaccines);
	int GetVaccines(void);

	void SetVaccinesThreshold(int nVaccines);
	int GetVaccinesThreshold(void);

	void SetVaccinesCapacity(int nVaccinesCapacity);
	int GetVaccinesCapacity(void);

	int ResetVaccines ( unsigned int n_step_number );
	int GetOutStepNumber ( void );
	void SetOutStepNumber ( unsigned int n_step_number );

  void Reset ( void );

private:
	float m_fGrey;
	dVector2 m_dCenter;
	float m_fIntRadius;
	float m_fExtRadius;
	float m_fHeight;
	
	int m_nVaccines;
	int m_nVaccinesThreshold;
	int m_nVaccinesCapacity;
	unsigned int m_nOutStepNumber;
	unsigned int RECOVERY_TIME;
	
	int m_nActivation;
};

/******************************************************************************/
/******************************************************************************/

#endif
