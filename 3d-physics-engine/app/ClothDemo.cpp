#include "ClothDemo.h"
#include "FixedFunctionPrimitives.h"
#include "glad/glad.h"
#include <iostream>

void ClothDemo::Initialize(int width, int height) {
	Demo::Initialize(width, height);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float val[] = { 0.5f, 1.0f, -1.5f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	camera.SetTarget(vec3(3.75622f, 2.98255f, 0.0f));
	camera.SetZoom(9.0f);
	camera.SetRotation(vec2(-67.9312f, 19.8f));

	num_part = 15;
	part_dist = 0.2f;
	k = -3.0f;
	d = 0.0f;
	ground.size = vec3(10.0f, 0.1f, 10.0f);




	ResetDemo();
}


void ClothDemo::ResetDemo() {
	physicsSystem.ClearRigidbodys();
	physicsSystem.ClearConstraints();
	physicsSystem.ClearSprings();
	physicsSystem.ClearCloths();
	renderObjects.clear();

	cloth.Initialize(num_part, part_dist, vec3(0, 6, 0));
	cloth.SetStructuralSprings(k, d);
	cloth.SetBendSprings(k, d);
	cloth.SetShearSprings(k, d);
	physicsSystem.AddCloth(&cloth);

	physicsSystem.AddConstraint(ground);
	renderObjects.resize(4);

	float d = 0.75f;
	renderObjects[0].position = vec3(d + 0.12f, 2.4f, d);
	renderObjects[1].position = vec3(-d, 2.3f, d);
	renderObjects[2].position = vec3(-d, 2.0f, -d);
	renderObjects[3].position = vec3(d, 2.3f, -d + 0.25f);

	for (int i = 0; i < 4; ++i) {
		//renderObjects[i].size = vec3(0.1f, 0.5f, 0.1f);

		renderObjects[i].size = vec3(0.3f, 0.5f, 0.3f);
		physicsSystem.AddConstraint(renderObjects[i]);
		renderObjects[i].size = vec3(0.1f, 0.5f, 0.1f);
	}

}



void ClothDemo::Render() {
	Demo::Render();

	// Mesh bending is not implemented, so we render the 
	// obstacle which the cloth is resting on with a cheat

	static const float constraintDiffuse[]{ 0.0f, 200.0f / 255.0f, 0.0f, 0.0f };
	static const float constraintAmbient[]{ 50.0f / 255.0f, 200.0f / 255.0f, 50.0f / 255.0f, 0.0f };
	static const float groundDiffuse[]{ 0.0f, 0.0f, 200.0f / 255.0f, 0.0f };
	static const float groundAmbient[]{ 50.0f / 255.0f, 50.0f / 255.0f, 200.0f / 255.0f, 0.0f };
	static const float val[] = { 0.0f, 1.0f, 0.0f, 0.0f };
	static const float zero[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, val);

	glColor3f(constraintDiffuse[0], constraintDiffuse[1], constraintDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, constraintAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, constraintDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
	for (int i = 0; i < renderObjects.size(); ++i) {
		::Render(renderObjects[i]);
	}

	glColor3f(groundDiffuse[0], groundDiffuse[1], groundDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, groundAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, groundDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
	::Render(ground);

	cloth.Render(false);

	//physicsSystem.Render();

	/*Line l(vec3(0, -8, 0), vec3(0, 8, 0));
	::Render(l);*/
}

void ClothDemo::Update(float dt) {
	Demo::Update(dt);

	physicsSystem.Update(dt);
}