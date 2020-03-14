#pragma once

#include "RigidbodyWithVolume.h"
#include "Spring.h"
#include "Cloth.h"

class PhysicsSystem {
protected:
	std::vector<Rigidbody*> bodies;
	std::vector<OBB> constraints;
	std::vector<Spring> springs;
	std::vector<Cloth*> cloths;

	// Collision varss
	std::vector<Rigidbody*> colliders1;
	std::vector<Rigidbody*> colliders2;
	std::vector<CollisionManifold> results;

public:
	float LinearProjectionPercent;// [0.2 to 0.8], Smaller = less jitter / more penetration
	float PenetrationSlack; // [0.01 to 0.1],  Smaller = more accurate
	int ImpulseIteration; // More iterations = more accurate physics (6-8 should be fine)
	bool DoLinearProjection; // A bit more heavy if used but not sinking

public:
	void Update(float deltaTime);
	void Render();

	void AddRigidbody(Rigidbody* body);
	void AddConstraint(const OBB& constraint);
	void AddSpring(const Spring& spring);
	void AddCloth(Cloth* cloth);

	void ClearRigidbodys();
	void ClearConstraints();
	void ClearSprings();
	void ClearCloths();

	PhysicsSystem();
};



