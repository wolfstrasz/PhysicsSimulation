#include "PhysicsSystem.h"

#include "PhysicsSystem.h"
#include "../app/FixedFunctionPrimitives.h"
#include "glad/glad.h"


PhysicsSystem::PhysicsSystem() :
	LinearProjectionPercent(0.45f),
	PenetrationSlack(0.01f),
	ImpulseIteration(5),
	DoLinearProjection(true)
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

void PhysicsSystem::ClearRigidbodys() {
	bodies.clear();
}

void PhysicsSystem::ClearConstraints()
{
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
	// Clear data
	colliders1.clear();
	colliders2.clear();
	results.clear();

	CollisionManifold result;
	// Find colliding pairs of bodies
	for (int i = 0, size = bodies.size(); i < size; ++i) {
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
			}
		}
	}

	// Apply forces to bodies to integrate new position
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->ApplyForces();

	// Apply impulses to resolve collisions
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

	// Update position and render (sum forces, integrate for new position, check for colissions)
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->Update(deltaTime);


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

	// Solve constraints
	for (int i = 0, size = bodies.size(); i < size; ++i) bodies[i]->SolveConstraints(constraints);
}
