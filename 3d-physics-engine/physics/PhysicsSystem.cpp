#include "PhysicsSystem.h"
#include "RigidbodyWithVolume.h"

#include "../app/FixedFunctionPrimitives.h"
#include "glad/glad.h"

#include <iostream>
PhysicsSystem::PhysicsSystem() :
	LinearProjectionPercent(0.45f),
	PenetrationSlack(0.01f),
	ImpulseIteration(5),
	DoLinearProjection(true),
	RenderRandomColors (false)
{	// Number of object pairs depending on how heavy simulation is
	colliders1.reserve(100);
	colliders2.reserve(100);
	results.reserve(100);
}

void PhysicsSystem::AddRigidbody(Rigidbody* body) {
	bodies.push_back(body);
}

void PhysicsSystem::AddConstraint(const OBB& obb) {
	constraints.push_back(obb);
}

void PhysicsSystem::AddSpring(const Spring& spring)
{
	springs.push_back(spring);
}

void PhysicsSystem::AddCloth(Cloth* cloth)
{
	cloths.push_back(cloth);
}

void PhysicsSystem::ClearRigidbodys() {
	bodies.clear();
}

void PhysicsSystem::ClearConstraints()
{
	constraints.clear();
}

void PhysicsSystem::ClearSprings()
{
	springs.clear();
}

void PhysicsSystem::ClearCloths()
{
	cloths.clear();
}


void PhysicsSystem::Render() {


	// Define material textures
	static const float rigidbodyDiffuse[]{200.0f / 255.0f, 0.0f, 0.0f, 0.0f };
	static const float rigidbodyAmbient[]{200.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f, 0.0f };
	static const float groundDiffuse[]{ 0.0f, 0.0f, 200.0f / 255.0f, 0.0f };
	static const float groundAmbient[]{ 50.0f / 255.0f, 50.0f / 255.0f, 200.0f / 255.0f, 0.0f };
	static const float constraintDiffuse[]{0.0f, 200.0f / 255.0f, 0.0f, 0.0f };
	static const float constraintAmbient[]{50.0f / 255.0f, 200.0f / 255.0f, 50.0f / 255.0f, 0.0f };
	static const float zero[] = { 0.0f, 0.0f, 0.0f, 0.0f };

	std::vector<const float*> ambient;
	std::vector<const float*> diffuse;
	//std::cout << "RENDERING\n";
	// RENDER BODIES
	// ---------------------------------------------------------------------------
	if (RenderRandomColors) {
		ambient.push_back(rigidbodyAmbient);
		ambient.push_back(groundAmbient);
		ambient.push_back(constraintAmbient);
		diffuse.push_back(rigidbodyDiffuse);
		diffuse.push_back(groundDiffuse);
		diffuse.push_back(constraintDiffuse);
	}

	// Render color for bodies
	glColor3f(rigidbodyDiffuse[0], rigidbodyDiffuse[1], rigidbodyDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, rigidbodyAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, rigidbodyDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);

	// Render bodies
	for (int i = 0, size = bodies.size(); i < size; ++i) {
		if (RenderRandomColors) {
			int a_i = i % ambient.size();
			int d_i = i % diffuse.size();
			glColor3f(diffuse[d_i][0], diffuse[d_i][1], diffuse[d_i][2]);
			glLightfv(GL_LIGHT0, GL_AMBIENT, ambient[a_i]);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse[d_i]);
			glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
		}

		bodies[i]->Render();
	}

	// Render Constraints
	// ---------------------------------------------------------------------------
	if (constraints.size() > 0) {
		glColor3f(groundDiffuse[0], groundDiffuse[1], groundDiffuse[2]);
		glLightfv(GL_LIGHT0, GL_AMBIENT, groundAmbient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, groundDiffuse);
		glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
		::Render(constraints[0]);
	}
	glColor3f(constraintDiffuse[0], constraintDiffuse[1], constraintDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, constraintAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, constraintDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);

	for (int i = 1; i < constraints.size(); ++i) ::Render(constraints[i]);

	// RENDER SPRINGS
	// --------------------------------------------------------------------------------------
	GLboolean status;
	glGetBooleanv(GL_LIGHTING, &status);
	for (int i = 0, size = springs.size(); i < size; ++i) {
		for (int i = 0, size = springs.size(); i < size; ++i) {
			if (springs[i].GetP1() == 0 || springs[i].GetP2() == 0) {
				continue;
			}

			Line l(springs[i].GetP1()->GetPosition(), springs[i].GetP2()->GetPosition());
			::Render(l);
		}
	}
	if (status) {
		glEnable(GL_LIGHTING);
	}

	// Render Cloths
	for (int i = 0, size = cloths.size(); i < size; ++i) {
		cloths[i]->Render(false);
	}
}

void PhysicsSystem::Update(float deltaTime) {
	// Clear data
	colliders1.clear();
	colliders2.clear();
	results.clear();
	//std::cout << "Physics Update" << std::endl;

	// Collision detection
	// ----------------------------------------------------------------------------------
	CollisionManifold result;
	for (int i = 0, size = bodies.size(); i < size; ++i) {
	//	std::cout << "CM for body: " << i << std::endl;
		for (int j = i; j < size; ++j) {
			if (i == j) continue;
			ResetCollisionManifold(&result);

			if (bodies[i]->HasVolume() && bodies[j]->HasVolume()) {
				RigidbodyWithVolume* m1 = (RigidbodyWithVolume*)bodies[i];
				RigidbodyWithVolume* m2 = (RigidbodyWithVolume*)bodies[j];
				result = FindCollisionFeatures(*m1, *m2);
			}

			// Store both if colliding
			if (result.colliding) {
				colliders1.push_back(bodies[i]);
				colliders2.push_back(bodies[j]);
				results.push_back(result);
				//std::cout << "COLLISION!\n";
			}
		}
	}

	// Apply forces to systems
	// ----------------------------------------------------------------------------------
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->ApplyForces(); // Bodies
	for (int i = 0, size = cloths.size(); i < size; ++i) cloths[i]->ApplyForces(); // Cloths

	// Resolve collisions by applying impulses
	// ----------------------------------------------------------------------------------
	for (int k = 0; k < ImpulseIteration; ++k) { // Apply impulses
		for (int i = 0, size = results.size(); i < size; ++i) {
			for (int j = 0, jSize = results[i].contacts.size(); j < jSize; ++j) {
				if (colliders1[i]->HasVolume() && colliders2[i]->HasVolume()) {
					RigidbodyWithVolume* m1 = (RigidbodyWithVolume*)colliders1[i];
					RigidbodyWithVolume* m2 = (RigidbodyWithVolume*)colliders2[i];
					ApplyImpulse(*m1, *m2, results[i], j);
				}
			}
		}
	}

	// Integration of velocity and impulses
	// ----------------------------------------------------------------------------------
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->Update(deltaTime); // Bodies
	for (int i = 0, size = cloths.size(); i < size; ++i) cloths[i]->Update(deltaTime); // Cloths

	// LINEAR PROJECTION
	// ----------------------------------------------------------------------------------
	// NOTE: To remove sinking we need to do a Linear Projection and to
	//		 do it  we need to slightly adjust objects positions along collision normal
	if (DoLinearProjection) {
		for (int i = 0, size = results.size(); i < size; ++i) {
			if (!colliders1[i]->HasVolume() && !colliders2[i]->HasVolume()) {
				continue;
			}

			RigidbodyWithVolume* m1 = (RigidbodyWithVolume*)colliders1[i];
			RigidbodyWithVolume* m2 = (RigidbodyWithVolume*)colliders2[i];
			float totalMass = m1->InvMass() + m2->InvMass();

			if (totalMass == 0.0f) {
				continue;
			}

			float depth = fmaxf(results[i].depth - PenetrationSlack, 0.0f);
			float scalar = (totalMass == 0.0f) ? 0.0f : depth / totalMass;
			vec3 correction = results[i].normal * scalar * LinearProjectionPercent;

			m1->position = m1->position - correction * m1->InvMass();
			m2->position = m2->position + correction * m2->InvMass();

			m1->SynchCollisionVolumes();
			m2->SynchCollisionVolumes();
		}
	}

	// Apply spring forces
	// ----------------------------------------------------------------------------------
	for (int i = 0, size = springs.size(); i < size; ++i) springs[i].ApplyForce(deltaTime);			// Springs
	for (int i = 0, size = cloths.size(); i < size; ++i) cloths[i]->ApplySpringForces(deltaTime);	// Cloths

	// Solve constraints
	// ----------------------------------------------------------------------------------
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->SolveConstraints(constraints); // Bodies
	for (int i = 0, size = cloths.size(); i < size; ++i) cloths[i]->SolveConstraints(constraints); // Cloths
}
