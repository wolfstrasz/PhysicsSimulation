#include "RigidbodyWithVolume.h"
#include "../app/FixedFunctionPrimitives.h"

void RigidbodyWithVolume::ApplyForces() { 
	forces = GRAVITY_CONST * mass;
}

void RigidbodyWithVolume::Render() {
	SynchCollisionVolumes();
	if (type == RIGIDBODY_TYPE::SPHERE) {
		::Render(sphere);
	}
	else if (type == RIGIDBODY_TYPE::BOX) {
		::Render(box);
	}
}

void RigidbodyWithVolume::Update(float dt) {
	const float damping = 0.98f;
	vec3 acceleration = forces * InvMass();
	velocity = velocity + acceleration * dt;
	velocity = velocity * damping;

	// integrate 
	position = position + velocity * dt;

	SynchCollisionVolumes();
}


// unique functions
// ------------------
void RigidbodyWithVolume::AddLinearImpulse(const vec3& impulse) {
	velocity = velocity + impulse;
}

float RigidbodyWithVolume::InvMass() {
	if (mass == 0.0f) { return 0.0f; }
	return 1.0f / mass;
}

void RigidbodyWithVolume::SynchCollisionVolumes() {
	sphere.position = position;
	box.position = position;
}
