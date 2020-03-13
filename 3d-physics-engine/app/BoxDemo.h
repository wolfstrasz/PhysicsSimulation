#pragma once
#include "Demo.h"

#include "../maths/geometry3d.h"
#include "../physics/PhysicsSystem.h"
#include "../physics/RigidbodyWithVolume.h"

class BoxDemo : public Demo{

protected:
	PhysicsSystem physicsSystem;
	std::vector<RigidbodyWithVolume> bodies;
	RigidbodyWithVolume groundBox;

protected:
	void ResetDemo();

public:
	void Initialize(int width, int height);
	void Render();
	void Update(float dt);
};