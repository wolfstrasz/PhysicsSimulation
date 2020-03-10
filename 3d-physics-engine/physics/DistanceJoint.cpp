#include "DistanceJoint.h"
#include "../app/FixedFunctionPrimitives.h"


void DistanceJoint::Initialize(Particle* p1, Particle* p2, float len) {
	this->p1 = p1;
	this->p2 = p2;
	length = len;
}

void DistanceJoint::Render() {
	vec3 pos1 = p1->GetPosition();
	vec3 pos2 = p2->GetPosition();
	Line l(pos1, pos2); // Create a visual line to render for joint
	::Render(l);
}

void DistanceJoint::SolveConstraints(const std::vector<OBB>& constraints) {
	vec3 delta = p2->GetPosition() - p1->GetPosition();
	float distance = Magnitude(delta);
	// Get correction distance
	float correction = (distance - length) / distance;

	// Apply distance correction
	p1->SetPosition(p1->GetPosition() + delta * 0.5f * correction);
	p2->SetPosition(p2->GetPosition() - delta * 0.5f * correction);

	p1->SolveConstraints(constraints);
	p2->SolveConstraints(constraints);

}