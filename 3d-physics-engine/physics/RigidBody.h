#pragma once
#include <vector>
#include "../maths/geometry3d.h"

class Rigidbody {
public:
	Rigidbody() { }
	virtual ~Rigidbody() { }

	virtual void Update(float deltaTime) { }
	virtual void Render() { }
	virtual void ApplyForces() { }
	virtual void SolveConstraints(const std::vector<OBB>& constraints) { }
};