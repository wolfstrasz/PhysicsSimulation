#include "Particle.h"
#include "../maths/geometry3d.h"
#include "../app/FixedFunctionPrimitives.h"
#include "../dfns.h"

void Particle::SetPosition(const vec3& position) {
	m_position = m_oldPosition = position;
}
vec3 Particle::GetPosition() {
	return m_position;
}
void Particle::SetBounce(float bounce) {
	m_bounce = bounce;
}
float Particle::GetBounce() {
	return m_bounce;
}

Particle::Particle() :
	m_friction(0.95f),
	m_gravity(vec3(0.0f, -9.82f, 0.0f)),
	m_mass (1.0f),
	m_bounce (0.7f)
{
	type = RIGIDBODY_TYPE::PARTICLE;
}

void Particle::Render() {
	Sphere visual(m_position, 0.1f);
	::Render(visual);
}

void Particle::ApplyForces() {
	m_forces = m_gravity * m_mass;
}

void Particle::Update(float deltaTime) {

#ifdef EULER_INTEGRATION
	m_oldPosition = m_position;
	vec3 acceleration = m_forces * InvMass();
	m_velocity = m_velocity * m_friction + acceleration * deltaTime;
	m_position = m_position + m_velocity * deltaTime;
#else 
	// VERLET VELOCITY INTEGRATION
	m_oldPosition = m_position;
	vec3 acceleration = m_forces * (1.0f / m_mass);
	vec3 oldVelocity = m_velocity;
	m_velocity = m_velocity * m_friction + acceleration * deltaTime;
	m_position = m_position + (oldVelocity + m_velocity) * 0.5f * deltaTime;
#endif // EULER_INTEGRATION

}

void Particle::SolveConstraints(const std::vector<OBB>& constraints) {
#ifdef EULER_INTEGRATION
	int size = constraints.size();
	for (int i = 0; i < size; ++i) {

		// Use line test to check for every point the particle travelled to
		// be able to protect from tunneling 

		// Line to represent path our particle travelled
		Line traveled(m_oldPosition, m_position);
		
		// Check for collision
		if (Linetest(constraints[i], traveled)) {
			// If collided raycast to find point of intersection
			vec3 direction = AsNormal(m_velocity);
			Ray ray(m_oldPosition, direction);
			RaycastResult result;
			if (Raycast(constraints[i], ray, &result)) {
				// Move particle a bit above collision to allow rolling down
				m_position = result.point + result.normal * 0.002f;

				// Deconstruct velocity vector into components relative to collision normal
				vec3 vn = result.normal * Dot(result.normal, m_velocity); // parallel
				vec3 vt = m_velocity - vn; // perpendicular

				m_oldPosition = m_position;
				m_velocity = vt - vn * m_bounce;
				break;
			}
		}
	}
#else 
	int size = constraints.size();
	for (int i = 0; i < size; ++i) {

		// Use line test to check for every point the particle travelled to
		// be able to protect from tunneling 

		// Line to represent path our particle travelled
		Line traveled(m_oldPosition, m_position);

		// Check for collision
		if (Linetest(constraints[i], traveled)) {
			vec3 velocity = m_position - m_oldPosition; // need this velocity for Verlet

			vec3 direction = AsNormal(velocity);
			Ray ray(m_oldPosition, direction);
			RaycastResult result;

			if (Raycast(constraints[i], ray, &result)) {
				// Place object just a little above collision result
				m_position = result.point + result.normal * 0.003f;

				vec3 vn = result.normal * Dot(result.normal, velocity);
				vec3 vt = velocity - vn;

				m_oldPosition = m_position - (vt - vn * m_bounce);

			}
		}
	}
#endif
}

void Particle::AddImpulse(const vec3& impulse) {
	m_velocity = m_velocity + impulse;
}

float Particle::InvMass() {
	if (m_mass == 0.0f) {
		return 0.0f;
	}
	return 1.0f / m_mass;
}

void Particle::SetMass(float m) {
	if (m < 0) {
		m = 0;
	}
	m_mass = m;
}

void Particle::SetFriction(float f) {
	if (f < 0) {
		f = 0;
	}
	m_friction = f;
}

vec3 Particle::GetVelocity() {
	return m_velocity;
}



