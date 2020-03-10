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
	void AddLinearImpulse(const vec3& impulse);
};