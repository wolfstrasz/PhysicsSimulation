#pragma once
#include "Rigidbody.h"

#define GRAVITY_CONST vec3(0.0f, -9.82f, 0.0f)


class RigidbodyWithVolume : public Rigidbody {
	// New class body


public:
	vec3 position;
	vec3 velocity;
	vec3 forces; // Sum of all forces
	float mass;
	float cor; // Coefficient of restitution
	float friction;

	// support for angular velocity
	vec3 orientation;
	vec3 angVel;
	vec3 torques;

	// volume (should come from hierarchy of a Shape and just keep shape)
	OBB box;
	Sphere sphere;

public:
	inline RigidbodyWithVolume() : cor(0.5f), mass(1.0f), friction(0.6f) {
		type = RIGIDBODY_TYPE::BASE;
	}

	inline RigidbodyWithVolume(RIGIDBODY_TYPE t) : cor(0.5f), mass(1.0f), friction(0.6f) { 
		type = t;
	}

	~RigidbodyWithVolume() {}

	// Overwritten funcitons
	void Render();
	void Update(float dt); // Update Position
	void ApplyForces();

	// Unique functions
	void SynchCollisionVolumes();
	float InvMass();

	// Support for Linear Velocity
	void AddLinearImpulse(const vec3& impulse);

	// Support for Angular Velocity
	mat4 InvTensor(); // The Moment of Inertia can be expressed as a 3x3 matrix called an Inertia Tensor. Can check online for all shapes.
	virtual void AddRotationalImpulse(const vec3& point, const vec3& impulse);

};


// Collision resolutions
CollisionManifold FindCollisionFeatures(RigidbodyWithVolume& ra, RigidbodyWithVolume& rb);
void ApplyImpulse(RigidbodyWithVolume& A, RigidbodyWithVolume& B, const CollisionManifold& M, int c);