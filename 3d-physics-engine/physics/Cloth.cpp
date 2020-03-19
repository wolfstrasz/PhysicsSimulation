#include "Cloth.h"

#include "Cloth.h"
#include "glad/glad.h"
#include "../app/FixedFunctionPrimitives.h"

void Cloth::Initialize(int gridSize, float distance,
	const vec3& position) {
	float k = -1.0f;
	float b = 0.0f;
	clothSize = gridSize;
	verts.clear();
	structural.clear();
	shear.clear();
	bend.clear();

	// reserve vertices
	verts.resize(gridSize * gridSize);

	// half size of cloth
	float hs = (float)(gridSize - 1) * 0.5f;

	if (gridSize < 3) gridSize = 3; // 9 particles for stable simulation

	// Create particles to represent cloth
	for (int x = 0; x < gridSize; ++x) {
		for (int z = 0; z < gridSize; ++z) {
			int i = z * gridSize + x;

			// Find world space pos
			float x_pos = ((float)x + position.x - hs) * distance;
			float z_pos = ((float)z + position.z - hs) * distance;

			// Set vertices data
			verts[i].SetPosition(vec3(x_pos, position.y, z_pos));
			verts[i].SetMass(1.0f);
			verts[i].SetBounce(0.0f);
			verts[i].SetFriction(0.9f);
		}
	}

	// Create structural hortizontal springs (left -> right)
	for (int x = 0; x < gridSize; ++x) {
		for (int z = 0; z < gridSize - 1; ++z) {
			int i = z * gridSize + x;
			int j = (z + 1) * gridSize + x;
			GenerateSpringForCloth(verts, structural, i, j);
		}
	}

	// Create structural vertical springs (up -> down)
	for (int x = 0; x < gridSize - 1; ++x) {
		for (int z = 0; z < gridSize; ++z) {
			int i = z * gridSize + x;
			int j = z * gridSize + (x + 1);
			GenerateSpringForCloth(verts, structural, i, j);
		}
	}

	// Create shear horizontal diagonal springs (left -> right)
	for (int x = 0; x < gridSize - 1; ++x) {
		for (int z = 0; z < gridSize - 1; ++z) {
			int i = z * gridSize + x;
			int j = (z + 1) * gridSize + (x + 1);
			GenerateSpringForCloth(verts, shear, i, j);
		}
	}

	// Create shear vertical diagonal (up -> down)
	for (int x = 1; x < gridSize; ++x) {
		for (int z = 0; z < gridSize - 1; ++z) {
			int i = z * gridSize + x;
			int j = (z + 1) * gridSize + (x - 1);
			GenerateSpringForCloth(verts, shear, i, j);
		}
	}

	// Create bending horizontal springs (left -> right)
	for (int x = 0; x < gridSize; ++x) {
		for (int z = 0; z < gridSize - 2; ++z) {
			int i = z * gridSize + x;
			int j = (z + 2) * gridSize + x;
			GenerateSpringForCloth(verts, bend, i, j);
		}
	}

	// Create bending vertical springs (up -> down)
	for (int x = 0; x < gridSize - 2; ++x) {
		for (int z = 0; z < gridSize; ++z) {
			int i = z * gridSize + x;
			int j = z * gridSize + (x + 2);
			GenerateSpringForCloth(verts, bend, i, j);
		}
	}
}

void Cloth::GenerateSpringForCloth(std::vector<Particle>& vs, std::vector<Spring>& sps, int i, int j) {
	// Set constants
	float k = -1.0f;
	float b = 0.0f;

	// get rest len
	vec3 iPos = vs[i].GetPosition();
	vec3 jPos = vs[j].GetPosition();
	float rest = Magnitude(iPos - jPos);
	// create spring
	Spring spring(k, b, rest);
	spring.SetParticles(&vs[i], &vs[j]);
	// Push to spring container
	sps.push_back(spring);
}

// Setters
void Cloth::SetStructuralSprings(float stiffness, float damp) {
	for (int i = 0; i < structural.size(); ++i) {
		structural[i].SetConstants(stiffness, damp);
	}
}

void Cloth::SetShearSprings(float stiffness, float damp) {
	for (int i = 0, size = shear.size(); i < size; ++i) {
		shear[i].SetConstants(stiffness, damp);
	}
}

void Cloth::SetBendSprings(float stiffness, float damp) {
	for (int i = 0, size = bend.size(); i < size; ++i) {
		bend[i].SetConstants(stiffness, damp);
	}
}

void Cloth::SetParticleMass(float mass) {
	for (int i = 0, size = verts.size(); i < size; ++i) {
		verts[i].SetMass(mass);
	}
}


// Particle system functors
void Cloth::ApplyForces() {
	for (int i = 0, size = verts.size(); i < size; ++i) {
		verts[i].ApplyForces();
	}
}

void Cloth::Update(float dt) {
	for (int i = 0, size = verts.size(); i < size; ++i) {
		verts[i].Update(dt);
	}
}

void Cloth::ApplySpringForces(float dt) {
	for (int i = 0, size = structural.size(); i < size; ++i) {
		structural[i].ApplyForce(dt);
	}
	for (int i = 0, size = shear.size(); i < size; ++i) {
		shear[i].ApplyForce(dt);
	}
	for (int i = 0, size = bend.size(); i < size; ++i) {
		bend[i].ApplyForce(dt);
	}
}


void Cloth::SolveConstraints(const std::vector<OBB>& constraints) {
	for (int i = 0, size = verts.size(); i < size; ++i) {
		verts[i].SolveConstraints(constraints);
	}
}

// Render
void Cloth::Render(bool debug) {

	static const float redDiffuse[]{ 200.0f / 255.0f, 0.0f, 0.0f, 0.0f };
	static const float redAmbient[]{ 200.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f, 0.0f };
	static const float zero[] = { 0.0f, 0.0f, 0.0f, 0.0f };

	glColor3f(redDiffuse[0], redDiffuse[1], redDiffuse[2]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, redAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, redDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, zero);

	
	// Loop to find particles to act as vertices
	for (int x = 0; x < clothSize - 1; ++x) {
		for (int z = 0; z < clothSize - 1; ++z) {
			int tl = z * clothSize + x;
			int bl = (z + 1) * clothSize + x;
			int tr = z * clothSize + (x + 1);
			int br = (z + 1) * clothSize + (x + 1);

			// Construct a quad to render
			Triangle t1(verts[tl].GetPosition(), verts[br].GetPosition(), verts[bl].GetPosition());
			Triangle t2(verts[tl].GetPosition(), verts[tr].GetPosition(), verts[br].GetPosition());

			::Render(t1, true);
			::Render(t2, true);
		}
	}

}