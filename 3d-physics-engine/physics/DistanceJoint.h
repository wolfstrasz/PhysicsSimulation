#pragma once
/*
	All joints to implement
	Distance Joint : This keeps bodies a set distance apart
	Ball Joint : This limits translation to the pivot of two objects
	Hinge Joint : This allows for rotation around a single axis
	Slider Joint : This limits rotationand translation to a single axis
	Fixed Joint : This does not allow movement
	Motor Joint : This produces some kind of force
*/

#include "Particle.h"
class DistanceJoint : public Rigidbody {
protected:
	Particle* p1;
	Particle* p2;
	float length;

public:
	
	// Physics system functors
	void Initialize(Particle* p1, Particle* p2, float len);
	void SolveConstraints( const std::vector<OBB>& constraints);
	void Render();
};