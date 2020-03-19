#pragma once

#include "Demo.h"
#include "../maths/geometry3d.h"
#include "../physics/PhysicsSystem.h"
#include "../physics/Cloth.h"
#include "../physics/RigidBodyWithVolume.h"

class ClothDemo : public Demo {
protected:
	PhysicsSystem physicsSystem;
	Cloth cloth;
	OBB ground;
	std::vector<OBB> renderObjects;


	int num_part;
	float part_dist;
	float k;
	float d;

protected:
	void ResetDemo();

public:
	inline ClothDemo() : Demo() { }

	void Initialize(int width, int height);
	void Render();
	void Update(float dt);
};
