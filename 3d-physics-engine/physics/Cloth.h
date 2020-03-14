#pragma once

#include "Spring.h"
#include <vector>

class Cloth {
protected:
	std::vector<Particle> verts;
	std::vector<Spring> structural; // spring between two hor/ver adj particles
	std::vector<Spring> shear;		// spring between two diagonally adj particles
	std::vector<Spring> bend;		// spring between two hor/ver skip through 1 particles

	float clothSize;

public:
	void Initialize(int gridSize, float distance, const vec3& position);

	// Setters
	void SetStructuralSprings(float stiffness, float damp);
	void SetShearSprings(float stiffness, float damp);
	void SetBendSprings(float stiffness, float damp);
	void SetParticleMass(float mass);

	// Physics system functors
	void ApplyForces();
	void Update(float dt);
	void ApplySpringForces(float dt);
	void SolveConstraints(const std::vector<OBB>& constraints);


	// Render
	void Render(bool debug);


	// Helper function
	void GenerateSpringForCloth(std::vector<Particle>& vs, std::vector<Spring>& sps, int i, int j);

};