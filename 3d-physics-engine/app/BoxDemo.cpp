#include "BoxDemo.h"

#include "FixedFunctionPrimitives.h"
#include "../include/glad/glad.h"
#include <iostream>

void BoxDemo::Initialize(int width, int height) {
	Demo::Initialize(width, height);
	std::cout << "Creating new box demo\n";

	physicsSystem.RenderRandomColors = true;
	physicsSystem.ImpulseIteration = 8;
	physicsSystem.DoLinearProjection = true;

	glPointSize(5.0f);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float val[] = { 0.5f, 1.0f, -1.5f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);
	
	camera.SetTarget(vec3(3.75622f, 2.98255f, 0.0f));
	camera.SetZoom(12.0f);
	camera.SetRotation(vec2(-67.9312f, 19.8f));

	ResetDemo();
}


void BoxDemo::ResetDemo() {
	std::cout << "Resetting the box demo\n";
	physicsSystem.ClearRigidbodys();
	physicsSystem.ClearConstraints();

	bodies.clear();
	bodies.resize(3);

	bodies[0].type = RIGIDBODY_TYPE::BOX;
	bodies[0].position = vec3(0.5f, 6, 0);
#ifndef LINEAR_ONLY
	bodies[0].orientation = vec3(0.0f, 0.0f, 0.4f);
#endif

	bodies[2].type = RIGIDBODY_TYPE::BOX;
	bodies[2].position = vec3(3, 9, 0);
#ifndef LINEAR_ONLY
	bodies[2].orientation = vec3(0.3f, 0.2f, 0.4f);
#endif

	bodies[1].type = RIGIDBODY_TYPE::SPHERE;
	bodies[1].position = vec3(0, 1, 0);
	bodies[1].mass = 5.0f;

	groundBox = RigidbodyWithVolume(RIGIDBODY_TYPE::BOX);
	groundBox.position = vec3(0, -0.5f, 0) * vec3(1, 0.5f, 1);
	groundBox.box.size = vec3(50, 1, 50) * 0.25f;
	groundBox.mass = 0.0f;
	groundBox.SynchCollisionVolumes();

	for (int i = 0; i < bodies.size(); ++i) {
		bodies[i].SynchCollisionVolumes();
		physicsSystem.AddRigidbody(&bodies[i]);
	}
	physicsSystem.AddRigidbody(&groundBox);
}

void BoxDemo::Render() {
	//std::cout << "Box demo Render\n";
	Demo::Render();

	float val[] = { 0.0f, 1.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);
	physicsSystem.Render();
}


void BoxDemo::Update(float dt) {
	std::cout << "Physics Update" << std::endl;
	//std::cout << "Box demo Update\n";
	Demo::Update(dt);
	std::cout << "Physics Update" << std::endl;

	physicsSystem.Update(dt);
}