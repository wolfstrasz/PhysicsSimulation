#pragma once
#include "Rigidbody.h";


class Particle : public Rigidbody {

private:
	vec3 m_position;
	vec3 m_oldPosition;
	vec3 m_forces;
	vec3 m_velocity;
	vec3 m_gravity;
	float m_mass;
	float m_bounce;
	float m_friction;
public:
	Particle();

	void Update(float deltaTime);
	void Render();
	void ApplyForces();
	void SolveConstraints(const std::vector<OBB>& constraints);

	void SetPosition(const vec3& pos);
	vec3 GetPosition();

	void SetBounce(float bounce);
	float GetBounce();
};