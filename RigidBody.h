#ifndef RIGIDBODY_H
#define RIGIDBODY_H

class RigidBody
{

private:

	int ID;			// unique identifier of rigid body
	float x,y,z;		// absolute position
	float qx,qy,qz,qw;	// orientation quaternions
	char Updated;		// info update flag

public:

	RigidBody();
	RigidBody(int ID, float x, float y, float z, float qx, float qy, float qz, float qw);

	int getID();
	void setID(int newID);

	void getPosition(float *x, float *y, float *z);
	void setPosition(float x, float y, float z);


	void getOrienation(float *qx, float *qy, float *qz, float *qw);
	void setOrientation(float qx, float qy, float qz, float qw);

	void getData(float *x, float *y, float *z, float *qx, float *qy, float *qz, float *qw);

	void setUpdateFlag(char value);
	char isUpdated();
	void UpdateData(float x, float y, float z, float qx, float qy, float qz, float qw);
};
#endif //RIGIDBODY_H

