#pragma once

#include "Particle.h"

class Spring {
protected:
	Particle* p1;
	Particle* p2;
	// higher k = stiff sprint, lower k = loose spring
	float stiffnessCoef; // [-n to 0]
	float dampStr; // [0 to 1], default to 0
	float restingLength;


public:
	inline Spring(float stiffness, float dampening, float restLen)
		: stiffnessCoef(stiffness), dampStr(dampening), restingLength(restLen) { }

	Particle* GetP1();
	Particle* GetP2();

	void SetParticles(Particle* _p1, Particle* _p2);
	void SetConstants(float _k, float _b);
	void ApplyForce(float dt);
};
