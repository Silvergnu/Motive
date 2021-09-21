#include "RigidBody.h"


// Initialize rigid body object with no information and an invalid ID of -1
RigidBody::RigidBody()
{
	ID=-1;
	x=0;
	y=0;
	z=0;
	qx=0;
	qy=0;
	qz=0;
	qw=0;
	Updated=0;
}


// Initialize rigid body object with given data
RigidBody::RigidBody(int ID,float x, float y, float z, float qx, float qy, float qz, float qw)
{
	this->ID=ID;
	this->x=x;
	this->y=y;
	this->z=z;
	this->qx=qx;
	this->qy=qy;
	this->qz=qz;
	this->qw=qw;
	this->Updated = 1;
}

	
int RigidBody::getID()
{
	return ID;		
}

void RigidBody::setID(int newID)
{
	this->ID=newID;
}

void RigidBody::getPosition(float *x, float *y, float *z)
{
	*x=this->x;
	*y=this->y;
	*z=this->z;
}

void RigidBody::setPosition(float x, float y, float z)
{
	this->x=x;
	this->y=y;
	this->z=z;
}


void RigidBody::getOrienation(float *qx, float *qy, float *qz, float *qw)
{
	*qx=this->qx;
	*qy=this->qy;
	*qz=this->qz;
	*qw=this->qw;
}

void RigidBody::setOrientation(float qx, float qy, float qz, float qw)
{
	this->qx=qx;
	this->qy=qy;
	this->qz=qz;
	this->qw=qw;
}

void RigidBody::setUpdateFlag(char value)
{
	this->Updated = value;
}

char RigidBody::isUpdated()
{
	return Updated;
}

void RigidBody::UpdateData(float x, float y, float z, float qx, float qy, float qz, float qw)
{

	this->x = x;
	this->y = y;
	this->z = z;
	this->qx = qx;
	this->qy = qy;
	this->qz = qz;
	this->qw = qw;
	this->Updated = 1;

}

/**************************************************
** getData:
**	All pointers in the parameters will be updated with
**	the data of this rigid body object
**  Also the "Updated" flag is reset to zero
***************************************************/
void RigidBody::getData(float *x, float *y, float *z, float *qx, float *qy, float *qz, float *qw)
{

	*x = this->x;
	*y = this->y;
	*z = this->z;
	*qx = this->qx;
	*qy = this->qy;
	*qz = this->qz;
	*qw = this->qw;
	this->Updated = 0;

}







