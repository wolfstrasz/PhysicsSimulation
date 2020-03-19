#pragma once
#include "Demo.h"
#include "../maths/geometry3d.h"
#include "../physics/PhysicsSystem.h"
#include "../physics/RigidbodyWithVolume.h"



class BowlingDemo : public Demo {
protected:
	PhysicsSystem physicsSystem;
	std::vector<RigidbodyWithVolume> bodies;
	RigidbodyWithVolume groundBox;

	bool isPaused;
	int numSteps;
	int stepSize;
	bool use_spheres;
	bool drop;
protected:

	void ResetDemo();
	float Random(float min, float max);
	vec3 Random(vec3 min, vec3 max);

public:
	void Initialize(int width, int height);
	void Render();
	void Update(float dt);
};