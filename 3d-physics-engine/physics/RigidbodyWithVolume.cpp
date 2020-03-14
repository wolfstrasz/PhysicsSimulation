#include "RigidbodyWithVolume.h"
#include "../app/FixedFunctionPrimitives.h"
#include "../maths/compare.h"

#define USE_ANGULAR_VELOCITY
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
	StopJitter(velocity);

	// Ang Vel for BOX
	if (type == RIGIDBODY_TYPE::BOX) {
		vec3 angAccel = MultiplyVector(torques, InvTensor());
		angVel = angVel + angAccel * dt;
		angVel = angVel * damping;
		StopJitter(velocity);

	} // If spheres have textures should ADD orientation for spheres as well !
	// and add rotation here!

	// Integrate Linear Velocity into position
	position = position + velocity * dt;

	// Integrate Angular Velocity into orientation :
	if (type == RIGIDBODY_TYPE::BOX) {
		orientation = orientation + angVel * dt;
	}

	SynchCollisionVolumes();
}


// Support for Linear Velocity
void RigidbodyWithVolume::AddLinearImpulse(const vec3& impulse) {
	velocity = velocity + impulse;
}


// Support for Angular Velocity
mat4 RigidbodyWithVolume::InvTensor()
{
	if (mass == 0) {
		return mat4(
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0
		);
	}
	// Values of diagonal of matrix (we have nonly sphere and box)
	float ix = 0.0f;
	float iy = 0.0f;
	float iz = 0.0f;
	float iw = 0.0f;

	// For sphere http://scienceworld.wolfram.com/physics/MomentofInertiaSphere.html
	if (mass != 0 && type == RIGIDBODY_TYPE::SPHERE) {
		float r2 = sphere.radius * sphere.radius;
		float fraction = (2.0f / 5.0f);
		ix = r2 * mass * fraction;
		iy = r2 * mass * fraction;
		iz = r2 * mass * fraction;
		iw = 1.0f;
	}
	// For Box
	else if (mass != 0 && type == RIGIDBODY_TYPE::BOX) {
		vec3 size = box.size * 2.0f;
		float fraction = (1.0f / 12.0f);
		float x2 = size.x * size.x;
		float y2 = size.y * size.y;
		float z2 = size.z * size.z;
		ix = (y2 + z2) * mass * fraction;
		iy = (x2 + z2) * mass * fraction;
		iz = (x2 + y2) * mass * fraction;
		iw = 1.0f;
	}

	// Return 
	return Inverse(mat4(
		ix, 0, 0, 0,
		0, iy, 0, 0,
		0, 0, iz, 0,
		0, 0, 0, iw));
}

void RigidbodyWithVolume::AddRotationalImpulse(const vec3& point, const vec3& impulse)
{
	vec3 centerOfMass = position;
	vec3 torque = Cross(point - centerOfMass, impulse);
	vec3 angAccel = MultiplyVector(torque, InvTensor());
	angVel = angVel + angAccel;
}




float RigidbodyWithVolume::InvMass() {
	if (mass == 0.0f) { return 0.0f; }
	return 1.0f / mass;
}

void RigidbodyWithVolume::SynchCollisionVolumes() {
	sphere.position = position;
	box.position = position;

	box.orientation = Rotation3x3(
		RAD2DEG(orientation.x),
		RAD2DEG(orientation.y),
		RAD2DEG(orientation.z)
	);
}

// Outside of CLASS

CollisionManifold FindCollisionFeatures(RigidbodyWithVolume& ra, RigidbodyWithVolume& rb)
{
	CollisionManifold result;
	ResetCollisionManifold(&result);

	if (ra.type == RIGIDBODY_TYPE::SPHERE) {
		if (rb.type == RIGIDBODY_TYPE::SPHERE) {
			result = FindCollisionFeatures(ra.sphere, rb.sphere);
		}
		else if (rb.type == RIGIDBODY_TYPE::BOX) {
			result = FindCollisionFeatures(rb.box, ra.sphere);
			result.normal = result.normal * -1.0f;
		}
	}
	else if (ra.type == RIGIDBODY_TYPE::BOX) {
		if (rb.type == RIGIDBODY_TYPE::BOX) {
			result = FindCollisionFeatures(ra.box, rb.box);
		}
		else if (rb.type == RIGIDBODY_TYPE::SPHERE) {
			result = FindCollisionFeatures(ra.box, rb.sphere);
		}
	}
	return result;
}
void ApplyImpulse(RigidbodyWithVolume& A, RigidbodyWithVolume& B, const CollisionManifold& M, int c) {
	
	// Calculations are done on the invs mass of bodies
	// Cuz if infinite mass => Invs_mass = 0 => impulse * mass = 0 => no movement
	float invMass1 = A.InvMass();
	float invMass2 = B.InvMass();
	float invMassSum = invMass1 + invMass2;

	// if both have infinite mass
	if (invMassSum == 0.0f)
		return;

#ifdef USE_ANGULAR_VELOCITY
	// Store point of contact relative to centre of mass
	vec3 r1 = M.contacts[c] - A.position;
	vec3 r2 = M.contacts[c] - B.position;

	// Store Inertia Tensors
	mat4 i1 = A.InvTensor();
	mat4 i2 = B.InvTensor();

	// Relative velocity
	vec3 relativeVel = (B.velocity + Cross(B.angVel, r2)) - (A.velocity + Cross(A.angVel, r1));
#else 
	// Relative velocity
	vec3 relativeVel = B.velocity - A.velocity;
#endif // USE_ANGULAR_VELOCITY

	// Relative collision normal
	vec3 relativeNorm = M.normal;
	Normalize(relativeNorm);

	// Check if they move away => do nothing
	if (Dot(relativeVel, relativeNorm) > 0.0f) 
		return;

	// Find magnitude of impulse to resolve collision
	float e = fminf(A.cor, B.cor);
	float numerator = (-(1.0f + e) * Dot(relativeVel, relativeNorm));

	float d1 = invMassSum;
#ifdef USE_ANGULAR_VELOCITY

	// Calculate impulse formula
	vec3 d2 = Cross(MultiplyVector(Cross(r1, relativeNorm), i1), r1);
	vec3 d3 = Cross(MultiplyVector(Cross(r2, relativeNorm), i2), r2);
	float denominator = d1 + Dot(relativeNorm, d2 + d3);
#else
	float denominator = d1;
#endif

	float j = (denominator == 0.0f) ? 0.0f : numerator / denominator;

	if (M.contacts.size() > 0.0f && j != 0.0f) {
		j /= (float)M.contacts.size();
	}

	// Obtain impulses and apply separating velocities
	vec3 impulse = relativeNorm * j;
	A.velocity = A.velocity - impulse * invMass1;
	B.velocity = B.velocity + impulse * invMass2; // As its dir is negative from normal => add it)

#ifdef USE_ANGULAR_VELOCITY
	A.angVel = A.angVel - MultiplyVector(Cross(r1, impulse), i1);
	B.angVel = B.angVel + MultiplyVector(Cross(r2, impulse), i2);
#endif

	// -------------------------------------------------------------------------------------
	// Calculate friction normal
	vec3 fNormal = relativeVel - (relativeNorm *Dot(relativeVel, relativeNorm));
	if (CMP(MagnitudeSq(fNormal), 0.0f)) {
		return;
	}
	Normalize(fNormal);

	// Calculate friction strength
	numerator = -Dot(relativeVel, fNormal);
	d1 = invMassSum;

#ifdef USE_ANGULAR_VELOCITY
	d2 = Cross(MultiplyVector(Cross(r1, fNormal), i1), r1);
	d3 = Cross(MultiplyVector(Cross(r2, fNormal), i2), r2);
	denominator = d1 + Dot(fNormal, d2 + d3);
#else
	denominator = d1;
#endif

	float fMagnitude = numerator / denominator;
	if (M.contacts.size() > 0.0f && fMagnitude != 0.0f) {
		fMagnitude /= (float)M.contacts.size();
	}
	if (CMP(fMagnitude, 0.0f)) {
		return;
	}

	// Clamp between -j * friction coef and +j * friction coef
	float friction = sqrtf(A.friction * B.friction);
	if (fMagnitude > j* friction) fMagnitude = j * friction;
	else if (fMagnitude < -j * friction) fMagnitude = -j * friction;

	// Apply tangential impulse (friction) to bodies
	vec3 tangentImpuse = fNormal * fMagnitude;
	A.velocity = A.velocity - tangentImpuse * invMass1;
	B.velocity = B.velocity + tangentImpuse * invMass2;

#ifdef USE_ANGULAR_VELOCITY
	A.angVel = A.angVel - MultiplyVector(Cross(r1, tangentImpuse), i1);
	B.angVel = B.angVel + MultiplyVector(Cross(r2, tangentImpuse), i2);
#endif

}

void StopJitter(vec3& velocity) {
	// Stop jitter
	if (fabsf(velocity.x) < 0.001f) {
		velocity.x = 0.0f;
	}
	if (fabsf(velocity.y) < 0.001f) {
		velocity.y = 0.0f;
	}
	if (fabsf(velocity.z) < 0.001f) {
		velocity.z = 0.0f;
	}
}