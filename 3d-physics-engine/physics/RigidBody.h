#pragma once
#include <vector>
#include "../maths/geometry3d.h"

enum class RIGIDBODY_TYPE {
	BASE, PARTICLE, SPHERE, BOX
};

class Rigidbody {
public:
	RIGIDBODY_TYPE type;
	inline Rigidbody() { type = RIGIDBODY_TYPE::BASE; }
	virtual ~Rigidbody() { }

	virtual void Update(float deltaTime) { }
	virtual void Render() { }
	virtual void ApplyForces() { }
	virtual void SolveConstraints(const std::vector<OBB>& constraints) { }

	inline bool HasVolume() { return type == RIGIDBODY_TYPE::SPHERE || type == RIGIDBODY_TYPE::BOX; }
};