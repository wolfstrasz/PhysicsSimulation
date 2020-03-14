#include "Spring.h"


void Spring::SetParticles(Particle* _p1, Particle* _p2) {
	p1 = _p1;
	p2 = _p2;
}

Particle* Spring::GetP1() {
	return p1;
}


Particle* Spring::GetP2() {
	return p2;
}

void Spring::SetConstants(float stiffness, float damping) {
	stiffnessCoef = stiffness;
	dampStr = damping;
}

void Spring::ApplyForce(float dt) {

	// Get relative position and velocity
	vec3 relPos = p2->GetPosition() - p1->GetPosition();
	vec3 relVel = p2->GetVelocity() - p1->GetVelocity();

	// Hooke's Law
	float x = Magnitude(relPos) - restingLength;
	float v = Magnitude(relVel);
	float spring_force = (-stiffnessCoef * x) + (-dampStr * v);

	// Turn force into impulse
	vec3 impulse = AsNormal(relPos) * spring_force;

	// Apply impulse
	p1->AddImpulse(impulse * p1->InvMass());
	p2->AddImpulse(impulse * -1.0f * p2->InvMass());
}