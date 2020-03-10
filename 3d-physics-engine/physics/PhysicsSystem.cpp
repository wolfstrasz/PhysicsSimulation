#include "PhysicsSystem.h"

#include "PhysicsSystem.h"
#include "../app/FixedFunctionPrimitives.h"
#include "glad/glad.h"


void PhysicsSystem::AddRigidbody(Rigidbody* body) {
	bodies.push_back(body);
}
void PhysicsSystem::AddConstraint(const OBB& obb) {
	constraints.push_back(obb);
}
void PhysicsSystem::ClearRigidbodys() {
	bodies.clear();
}
void PhysicsSystem::ClearConstraints() {
	constraints.clear();
}

void PhysicsSystem::Render() {
	// Define material textures
	static const float rigidbodyDiffuse[]{200.0f / 255.0f, 0.0f, 0.0f, 0.0f };
	static const float rigidbodyAmbient[]{200.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f, 0.0f };
	static const float constraintDiffuse[]{0.0f, 200.0f / 255.0f, 0.0f, 0.0f };
	static const float constraintAmbient[]{50.0f / 255.0f, 200.0f / 255.0f, 50.0f / 255.0f, 0.0f };
	static const float zero[] = { 0.0f, 0.0f, 0.0f, 0.0f };

	// Render color for bodies
	glColor3f(rigidbodyDiffuse[0], rigidbodyDiffuse[1], rigidbodyDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, rigidbodyAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, rigidbodyDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);

	// Render bodies
	for (int i = 0, size = bodies.size(); i < size; ++i) {
		bodies[i]->Render();
	}

	// Colour for constraints
	glColor3f(constraintDiffuse[0], constraintDiffuse[1], constraintDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, constraintAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, constraintDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);

	for (int i = 0; i < constraints.size(); ++i) ::Render(constraints[i]);
}

void PhysicsSystem::Update(float deltaTime) {
	// Apply forces to bodies to integrate new position
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->ApplyForces();
	// Update position and render (sum forces, integrate for new position, check for colissions)
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->Update(deltaTime);
	// Check for new constraints
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->SolveConstraints(constraints);
}
