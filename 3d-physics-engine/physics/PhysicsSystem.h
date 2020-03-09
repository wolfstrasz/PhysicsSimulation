#pragma once

#include "Rigidbody.h"

class PhysicsSystem {
protected:
	std::vector<Rigidbody*> bodies;
	std::vector<OBB> constraints;
public:
	void Update(float deltaTime);
	void Render();
	void AddRigidbody(Rigidbody* body);
	void AddConstraint(const OBB& constraint);
	void ClearRigidbodys();
	void ClearConstraints();
};