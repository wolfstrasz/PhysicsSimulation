#include "BowlingDemo.h"
#include "../include/glad/glad.h"
#include "FixedFunctionPrimitives.h"

void BowlingDemo::Initialize(int width, int height) {
	Demo::Initialize(width, height);

	physicsSystem.RenderRandomColors = true;

	isPaused = true;
	numSteps = 0;
	stepSize = 1;

	glPointSize(5.0f);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float val[] = { 0.5f, 1.0f, -1.5f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);
	
	camera.SetTarget(vec3(3.75622f, 2.98255f, 0.0f));
	camera.SetZoom(1.0f);
	camera.SetRotation(vec2(-67.9312f, 19.8f));


	ResetDemo();
}

void BowlingDemo::ResetDemo() {
	physicsSystem.ClearRigidbodys();
	physicsSystem.ClearConstraints();

	bodies.clear();
	bodies.resize(8);

	bodies[0].type = RIGIDBODY_TYPE::SPHERE;
	for (int i = 1; i < 7; i++) {
		bodies[i].type = RIGIDBODY_TYPE::BOX;
		bodies[i].mass = 1.0f;
		bodies[i].cor = 0.75f;
		bodies[i].friction = 0.4f;
	}


	// Place bowling ball
	bodies[0].position = vec3(5, 1.1f, 9.0f);
	
	// Place bowling pins
	// line 3
	bodies[1].position = vec3(-25, 1.0f, 7);
	bodies[2].position = vec3(-25, 1.0f, 10);
	bodies[3].position = vec3(-25, 1.0f, 13);
	// line 2
	bodies[4].position = vec3(-25, 3.0f, 8.5f);
	bodies[5].position = vec3(-25, 3.0f, 11.5f);
	// line 1
	bodies[6].position = vec3(-25, 5.0f, 10);

	
	// Add liniear impulse to bowling ball (imp direction = target - my_pos)
	vec3 offset = vec3(0.13f, 0.4f, -0.45f);

	vec3 impulseDirection = (bodies[2].position - bodies[0].position) + offset;
	vec3 impulsePower = impulseDirection * 2.5f;
	impulsePower.y = 4.0f;
	// impulsePower = impulseDirection + vec3(0, 5, 0)
	bodies[0].AddLinearImpulse(impulsePower);
	

	// Place ground
	groundBox = RigidbodyWithVolume(RIGIDBODY_TYPE::BOX);
	groundBox.position = vec3(0.0f, -0.3f, 0.0f);
	groundBox.box.size = vec3(50.0f, 0.3f, 30.0f);
	groundBox.mass = 0.0f;

	bodies[7] = RigidbodyWithVolume(RIGIDBODY_TYPE::BOX);
	bodies[7].position = vec3(0.0f, -20.0f, 0.0f);

	bodies[7].box.size = vec3(50.0f, 0.1f, 30.0f);
	bodies[7].mass = 0.0f;

	for (int i = 0; i < 7; i++) {
		physicsSystem.AddRigidbody(&bodies[i]);
	}

	physicsSystem.AddRigidbody(&groundBox);
	physicsSystem.AddRigidbody(&bodies[7]);
}

void BowlingDemo::Render() {
	Demo::Render();

	float val[] = { 0.0f, 1.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	physicsSystem.Render();
}

void BowlingDemo::Update(float dt) {
	Demo::Update(dt);
	physicsSystem.Update(dt);

}