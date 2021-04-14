#include "bluelightobject.h"

/******************************************************************************/
/******************************************************************************/

CBlueLightObject::CBlueLightObject(const char* pch_name) : CGeometry(pch_name)
{
	m_dCenter.x=0.0;
	m_dCenter.y=0.0;
	m_fIntRadius=0.0;
	m_fExtRadius=0.0;
	m_fGrey=1.0;

	m_nVaccinesCapacity = 1;
	m_nVaccines = m_nVaccinesCapacity;
	m_nVaccinesThreshold = 0;
	m_nOutStepNumber=0;
	RECOVERY_TIME = 1000;

	m_nActivation = true;
}

/******************************************************************************/
/******************************************************************************/

CBlueLightObject::~CBlueLightObject(){
	
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetColor(float fGrey){
	m_fGrey=fGrey;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::GetColor(float* fGrey){
	(*fGrey)=m_fGrey;
}

/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetHeight(float fHeight){
	m_fHeight=fHeight;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::GetHeight(float* fHeight){
	(*fHeight)=m_fHeight;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetCenter(dVector2 dCenter){
	m_dCenter=dCenter;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::GetCenter(dVector2 *dCenter){
	(*dCenter) = m_dCenter;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetIntRadius(float fRadius){
	m_fIntRadius=fRadius;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::GetIntRadius(float* fRadius){
	(*fRadius)=m_fIntRadius;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetExtRadius(float fRadius){
	m_fExtRadius=fRadius;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::GetExtRadius(float* fRadius){
	(*fRadius)=m_fExtRadius;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::Switch ( int n_value )
{
	if ( n_value == 0)
		m_nActivation = false;
	else 
		m_nActivation = true;
}

/******************************************************************************/
/******************************************************************************/

int CBlueLightObject::GetStatus ( void )
{
	return m_nActivation;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::Reset ( void )
{
  m_nActivation = true;
}

/******************************************************************************/
/******************************************************************************/

int CBlueLightObject::GetTiming ( unsigned int n_step_number )
{
  
  //printf("Act: Blue: %d\n", m_nActivation);

	/* Create sequence */
  //if ( !(n_step_number % 500) )
  //{
    ///* toggle light */
    //m_nActivation ^= 0x1;
  //}

	/* default return true */
	return m_nActivation;

}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetVaccines(int nVaccines){
	m_nVaccines=nVaccines;
}

/******************************************************************************/
/******************************************************************************/

int CBlueLightObject::GetVaccines ( void )
{
	return m_nVaccines;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetVaccinesThreshold(int nVaccinesThreshold){
	m_nVaccinesThreshold=nVaccinesThreshold;
}

/******************************************************************************/
/******************************************************************************/

int CBlueLightObject::GetVaccinesThreshold ( void )
{
	return m_nVaccinesThreshold;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetVaccinesCapacity (int nVaccinesCapacity)
{
	m_nVaccinesCapacity = nVaccinesCapacity;
}

/******************************************************************************/
/******************************************************************************/

int CBlueLightObject::GetVaccinesCapacity ( void )
{
	return m_nVaccinesCapacity;
}

/******************************************************************************/
/******************************************************************************/

void CBlueLightObject::SetOutStepNumber(unsigned int n_step_number){
	m_nOutStepNumber = n_step_number;
}

/******************************************************************************/
/******************************************************************************/

int CBlueLightObject::GetOutStepNumber ( void )
{
	return m_nOutStepNumber;
}

/******************************************************************************/
/******************************************************************************/

int CBlueLightObject::ResetVaccines ( unsigned int n_step_number)
{
  if(m_nActivation) m_nOutStepNumber = n_step_number;
  if ( (n_step_number - m_nOutStepNumber) >= RECOVERY_TIME )
  {
    /* toggle light */
    m_nActivation = 1;
	m_nVaccines = m_nVaccinesCapacity;
	
	// Reset Out Step Number
	m_nOutStepNumber = n_step_number;

  }

	/* default return true */
	return m_nActivation;

}
