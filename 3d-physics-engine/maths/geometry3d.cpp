#include "geometry3d.h"

#include <cfloat>
#include <list>
#include <cmath>

#define CMP(x, y) \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

float Length(const Line& line) { return Magnitude(line.start - line.end); }
float LengthSq(const Line& line) { return MagnitudeSq(line.start - line.end); }

Ray FromPoints(const Point& from, const Point& to) {
  return Ray(from, AsNormal(to - from));
}

vec3 GetMin(const AABB& aabb) {
  vec3 p1 = aabb.origin + aabb.size;
  vec3 p2 = aabb.origin - aabb.size;
  return vec3(fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z));
}

vec3 GetMax(const AABB& aabb) {
  vec3 p1 = aabb.origin + aabb.size;
  vec3 p2 = aabb.origin - aabb.size;
  return vec3(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z));
}

AABB FromMinMax(const vec3& min, const vec3& max) {
  return AABB((min + max) * 0.5f, (max - min) * 0.5f);
}

float PlaneEquation(const Point& point, const Plane& plane) {
  return Dot(point, plane.normal) - plane.distance;
}

// Point Tests
// ----------------------------------------------
// ----------------------------------------------

// SPHERE
bool IsPointInSphere(const Point& point, const Sphere& sphere) {
  float magSq = MagnitudeSq(point - sphere.position);
  float radSq = sphere.radius * sphere.radius;
  return magSq < radSq;
}

Point GetClosestPoint(const Sphere& sphere, const Point& point) {
  // Find a normalized vector from the center of the sphere to the test point
  vec3 sphereToPoint = point - sphere.position;
  Normalize(sphereToPoint);
  // Resize the normalized vector to the size of the radius
  sphereToPoint = sphereToPoint * sphere.radius;
  // Return the resized vector offset by the position of the sphere
  // World space
  return sphereToPoint + sphere.position;
}

// AABB
bool IsPointInAABB(const Point& point, const AABB& aabb) {
  Point min = GetMin(aabb);
  Point max = GetMax(aabb);
  if (point.x < min.x || point.y < min.y || point.z < min.z) {
    return false;
  }
  if (point.x > max.x || point.y > max.y || point.z > max.z) {
    return false;
  }
  return true;
}

Point GetClosestPoint(const AABB& aabb, const Point& point) {
  Point result = point;
  Point min = GetMin(aabb);
  Point max = GetMax(aabb);
  // Clamp the closest point to the min point of the AABB
  result.x = (result.x < min.x) ? min.x : result.x;
  result.y = (result.y < min.x) ? min.y : result.y;
  result.z = (result.z < min.x) ? min.z : result.z;
  // Clamp the closest point to the max point of the AABB
  result.x = (result.x > max.x) ? max.x : result.x;
  result.y = (result.y > max.x) ? max.y : result.y;
  result.z = (result.z > max.x) ? max.z : result.z;
  // Return a point in world space.
  return result;
}

// A more efficient solution is to project the point onto each axis of the OBB,
// then compare the projected point to the length of the OBB on each axis.
bool IsPointInOBB(const Point& point, const OBB& obb) {
  // Move point to pos relative to OBB
  // Get Direction Vector from OBB centre to point
  vec3 dir = point - obb.position;
  for (int i = 0; i < 3; ++i) {
    // Get axis ( X = 0, Y = 1, Z = 2)
    const float* orientation = &obb.orientation.asArray[i * 3];
    vec3 axis(orientation[0], orientation[1], orientation[2]);
    // Project point onto axis
    float distance = Dot(dir, axis);
    // Check if inside OOB axis extension
    if (distance > obb.size.asArray[i]) {
      return false;
    }
    if (distance < -obb.size.asArray[i]) {
      return false;
    }
  }
  return true;
}

// Same as testing but CLAMP and return point
Point GetClosestPoint(const OBB& obb, const Point& point) {
  Point result = obb.position;
  vec3 dir = point - obb.position;
  for (int i = 0; i < 3; ++i) {
    const float* orientation = &obb.orientation.asArray[i * 3];
    vec3 axis(orientation[0], orientation[1], orientation[2]);
    float distance = Dot(dir, axis);
    if (distance > obb.size.asArray[i]) {
      distance = obb.size.asArray[i];
    }
    if (distance < -obb.size.asArray[i]) {
      distance = -obb.size.asArray[i];
    }
    result = result + (axis * distance);
  }
  return result;
}

// Plane
bool IsPointOnPlane(const Point& point, const Plane& plane) {
  return CMP(PlaneEquation(point, plane), 0.0f);
}

Point GetClosestPoint(const Plane& plane, const Point& point) {
  float proj = Dot(plane.normal, point);
  float pointToPlaneDistance = proj - plane.distance;
  return point - plane.normal * pointToPlaneDistance;
}

// Line
bool IsPointOnLine(const Point& point, const Line& line) {
  Point closest = GetClosestPoint(line, point);
  float distanceSq = MagnitudeSq(closest - point);
  return CMP(distanceSq, 0.0f);
}

Point GetClosestPoint(const Line& line, const Point& point) {
  vec3 lVec = line.end - line.start;  // Line Vector
  float t = Dot(point - line.start, lVec) / Dot(lVec, lVec);
  t = fmaxf(t, 0.0f);  // Clamp to 0
  t = fminf(t, 1.0f);  // Clamp to 1
  return line.start + lVec * t;
}

// Ray (Directed line from point to direction. Not a full line)
bool IsPointOnRay(const Point& point, const Ray& ray) {
  if (point == ray.origin) {
    return true;
  }

  vec3 norm = point - ray.origin;
  Normalize(norm);
  float diff = Dot(norm, ray.direction);
  return CMP(diff, 1.0f);
}

Point GetClosestPoint(const Ray& ray, const Point& point) {
  // if direction is not a normal
  // Normalise(ray.direction)
  float t = Dot(point - ray.origin, ray.direction);
  // We assume the direction of the ray is normalized
  // If for some reason the direction is not normalized
  // the below division is needed. So long as the ray
  // direction is normalized, we don't need this divide
  // t /= Dot(ray.direction, ray.direction);
  t = fmaxf(t, 0.0f);
  return Point(ray.origin + ray.direction * t);
}

// Sphere Collision tests
// -----------------------
bool SphereSphere(const Sphere& s1, const Sphere& s2) {
  float radiiSum = s1.radius + s2.radius;
  float sqDistance = MagnitudeSq(s1.position - s2.position);
  return sqDistance < radiiSum * radiiSum;
}

bool SphereAABB(const Sphere& sphere, const AABB& aabb) {
  Point closestPoint = GetClosestPoint(aabb, sphere.position);
  float distSq = MagnitudeSq(sphere.position - closestPoint);
  float radiusSq = sphere.radius * sphere.radius;
  return distSq < radiusSq;
}

bool SphereOBB(const Sphere& sphere, const OBB& obb) {
  Point closestPoint = GetClosestPoint(obb, sphere.position);
  float distSq = MagnitudeSq(sphere.position - closestPoint);
  float radiusSq = sphere.radius * sphere.radius;
  return distSq < radiusSq;
}

bool SpherePlane(const Sphere& s, const Plane& p) {
  Point closestPoint = GetClosestPoint(p, s.position);
  float distSq = MagnitudeSq(s.position - closestPoint);
  float radiusSq = s.radius * s.radius;
  return distSq < radiusSq;
}

// Axis Aligned Bounding Box Collision test
// ----------------------------------------
bool AABBAABB(const AABB& aabb1, const AABB& aabb2) {
  Point aMin = GetMin(aabb1);
  Point aMax = GetMax(aabb1);
  Point bMin = GetMin(aabb2);
  Point bMax = GetMax(aabb2);
  return (aMin.x <= bMax.x && aMax.x >= bMin.x) &&
         (aMin.y <= bMax.y && aMax.y >= bMin.y) &&
         (aMin.z <= bMax.z && aMax.z >= bMin.z);
}
// Intervals to project Bounding boxes onto axis
// ----------------------------------------------

Interval GetIntervalProjection(const AABB& aabb, const vec3& axis) {
  // Get min max vertices
  vec3 i = GetMin(aabb);
  vec3 a = GetMax(aabb);
  // Get all vertices from them
  vec3 vertex[8] = {vec3(i.x, a.y, a.z), vec3(i.x, a.y, i.z),
                    vec3(i.x, i.y, a.z), vec3(i.x, i.y, i.z),
                    vec3(a.x, a.y, a.z), vec3(a.x, a.y, i.z),
                    vec3(a.x, i.y, a.z), vec3(a.x, i.y, i.z)};
  // Project vertices onto axis and get interval
  Interval result;
  result.min = result.max = Dot(axis, vertex[0]);
  for (int i = 1; i < 8; ++i) {
    float projection = Dot(axis, vertex[i]);
    result.min = (projection < result.min) ? projection : result.min;
    result.max = (projection > result.max) ? projection : result.max;
  }
  return result;
}

Interval GetIntervalProjection(const OBB& obb, const vec3& axis) {
  vec3 vertex[8];

  vec3 centre = obb.position;  // OBB Center
  float xExtent = obb.size.x;  // OBB Extents
  float yExtent = obb.size.y;
  float zExtent = obb.size.z;

  // Orientation 3x3 matrix as array
  const float* oMat = obb.orientation.asArray;

  // Get all 3 local axes
  vec3 xAxis = vec3(oMat[0], oMat[1], oMat[2]);
  vec3 yAxis = vec3(oMat[3], oMat[4], oMat[5]);
  vec3 zAxis = vec3(oMat[6], oMat[7], oMat[8]);

  // Use the center, extents, and local axes to find the actual vertices
  vertex[0] = centre + xAxis * xExtent + yAxis * yExtent + zAxis * zExtent;
  vertex[1] = centre - xAxis * xExtent + yAxis * yExtent + zAxis * zExtent;
  vertex[2] = centre + xAxis * xExtent - yAxis * yExtent + zAxis * zExtent;
  vertex[3] = centre + xAxis * xExtent + yAxis * yExtent - zAxis * zExtent;
  vertex[4] = centre - xAxis * xExtent - yAxis * yExtent - zAxis * zExtent;
  vertex[5] = centre + xAxis * xExtent - yAxis * yExtent - zAxis * zExtent;
  vertex[6] = centre - xAxis * xExtent + yAxis * yExtent - zAxis * zExtent;
  vertex[7] = centre - xAxis * xExtent - yAxis * yExtent + zAxis * zExtent;

  // Project vertices onto axis to obtain interval
  Interval result;
  result.min = result.max = Dot(axis, vertex[0]);
  for (int i = 1; i < 8; ++i) {
    float projection = Dot(axis, vertex[i]);
    result.min = (projection < result.min) ? projection : result.min;
    result.max = (projection > result.max) ? projection : result.max;
  }
  return result;
}

bool AreOverlapingOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis) {
  Interval a = GetIntervalProjection(aabb, axis);
  Interval b = GetIntervalProjection(obb, axis);
  return ((b.min <= a.max) && (a.min <= b.max));
}

bool AreOverlapingOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis) {
  Interval a = GetIntervalProjection(obb1, axis);
  Interval b = GetIntervalProjection(obb1, axis);
  return ((b.min <= a.max) && (a.min <= b.max));
}

// AABB with OBB Collision test by SAT
// -----------------------------------
bool AABBOBB(const AABB& aabb, const OBB& obb) {
  const float* o = obb.orientation.asArray;
  vec3 testAxis[15] = {
      vec3(1, 0, 0),           // AABB axis 1
      vec3(0, 1, 0),           // AABB axis 2
      vec3(0, 0, 1),           // AABB axis 3
      vec3(o[0], o[1], o[2]),  // OBB axis 1
      vec3(o[3], o[4], o[5]),  // OBB axis 2
      vec3(o[6], o[7], o[8])   // OBB axis 3
  };

  // The next 9 axes are the X-products of the rotation frames of the shapes
  for (int i = 0; i < 3; ++i) {  // Fill out rest of axes
    testAxis[6 + i * 3 + 0] = Cross(testAxis[i], testAxis[0]);
    testAxis[6 + i * 3 + 1] = Cross(testAxis[i], testAxis[1]);
    testAxis[6 + i * 3 + 2] = Cross(testAxis[i], testAxis[2]);
  }

  for (int i = 0; i < 15; ++i) {
    if (!AreOverlapingOnAxis(aabb, obb, testAxis[i])) {
      return false;  // Seperating axis found
    }
  }
  return true;  // Seperating axis not found
}

// AABB with Plane Collision test
// -------------------------------
bool AABBPlane(const AABB& aabb, const Plane& plane) {
  // Project the half extents of the AABB onto the plane normal
  float halfExtentProjectionLenght = aabb.size.x * fabsf(plane.normal.x) +
                                     aabb.size.y * fabsf(plane.normal.y) +
                                     aabb.size.z * fabsf(plane.normal.z);

  // Find the distance from the center of the AABB to the plane
  float distFromPlaneOrigin = Dot(plane.normal, aabb.origin) - plane.distance;

  // Intersection occurs if the distance falls within the projected side
  return fabsf(distFromPlaneOrigin) <= halfExtentProjectionLenght;
}

// OBB with OBB Collision test by SAT
// -----------------------------------
bool OBBOBB(const OBB& obb1, const OBB& obb2) {
  const float* o1 = obb1.orientation.asArray;
  const float* o2 = obb2.orientation.asArray;

  vec3 testAxis[15] = {
      vec3(o1[0], o1[1], o1[2]),  // OBB1 Axis 1
      vec3(o1[3], o1[4], o1[5]),  // OBB1 Axis 2
      vec3(o1[6], o1[7], o1[8]),  // OBB1 Axis 3
      vec3(o2[0], o2[1], o2[2]),  // OBB2 Axis 1
      vec3(o2[3], o2[4], o2[5]),  // OBB2 Axis 2
      vec3(o2[6], o2[7], o2[8])   // OBB2 Axis 3
  };

  // The next 9 axes are the X-products of the rotation frames of the shapes
  for (int i = 0; i < 3; ++i) {
    testAxis[6 + i * 3 + 0] = Cross(testAxis[i], testAxis[0]);
    testAxis[6 + i * 3 + 1] = Cross(testAxis[i], testAxis[1]);
    testAxis[6 + i * 3 + 2] = Cross(testAxis[i], testAxis[2]);
  }
  for (int i = 0; i < 15; ++i) {
    if (!AreOverlapingOnAxis(obb1, obb2, testAxis[i])) {
      return false;  // Seperating axis found
    }
  }
  return true;  // Seperating axis not found
}

// OBB with Plane Collision test
// ------------------------------
bool OBBPlane(const OBB& obb, const Plane& plane) {
  // Local variables for readability only
  const float* o = obb.orientation.asArray;

  vec3 xAxis = vec3(o[0], o[1], o[2]);
  vec3 yAxis = vec3(o[3], o[4], o[5]);
  vec3 zAxis = vec3(o[6], o[7], o[8]);

  vec3 normal = plane.normal;

  // Similar as AABB to Plane but we use dot products between
  // the plane normal and the three axes of the OBB
  float halfExtentProjectionLenght = obb.size.x * fabsf(Dot(normal, xAxis)) +
                                     obb.size.y * fabsf(Dot(normal, yAxis)) +
                                     obb.size.z * fabsf(Dot(normal, zAxis));

  float distFromPlaneOrigin = Dot(plane.normal, obb.position) - plane.distance;

  return fabsf(distFromPlaneOrigin) <= halfExtentProjectionLenght;
}

// PLane with Plane Intersection test
// -----------------------------------
bool PlanePlane(const Plane& plane1, const Plane& plane2) {
  vec3 d = Cross(plane1.normal, plane2.normal);  // <0,0,0> if parallel
  return !CMP(Dot(d, d), 0.0f);
}


// Raycasting
// -----------------------------------
// helper func


void ResetRaycastResult(RaycastResult* outResult) {
	if (outResult != nullptr) {
		outResult->t = -1.0f;
		outResult->hit = false;
		outResult->normal = vec3(0.0f, 0.0f, 1.0f);
		outResult->point = vec3(0.0f, 0.0f, 0.0f);
	}
}

inline float noZeroDirection(float direction) { return CMP(direction, 0.0f) ? 0.00001f : direction;  }

bool Raycast(const Sphere& sphere, const Ray& ray, RaycastResult* storeResult)
{
	// Reset result (just in case)
	ResetRaycastResult(storeResult);

	vec3 e = sphere.position - ray.origin;

	// Store squared radiuses and maginitudes
	float rSq = sphere.radius * sphere.radius;
	float eSq = MagnitudeSq(e);

	// ray.direction is assumed to be normalized
	float a = Dot(e, ray.direction);

	// Construct sides of a triangle (work with squared units)
	float bSq = eSq - (a * a);
	float f = sqrt(rSq - bSq);

	// Start by assuming an intersection
	float t = a - f;

	// No collision has happened
	if (rSq - (eSq - (a * a)) < 0.0f) {
		return false; // -1 is invalid.
	}
	// Ray starts inside the sphere
	else if (eSq < rSq) {
		t = a + f; // Just reverse direction
	}

	// If there was an intersection calculate result
	if (storeResult != nullptr) {
		storeResult->t = t;
		storeResult->hit = true;
		storeResult->point = ray.origin + ray.direction * t;
		storeResult->normal = AsNormal(storeResult->point - sphere.position);
	}
	return true;
}

bool Raycast(const AABB& aabb, const Ray& ray, RaycastResult* storeResult)
{
	// Reset result (just in case)
	ResetRaycastResult(storeResult);

	vec3 min = GetMin(aabb);
	vec3 max = GetMax(aabb);

	// Find the both intersections of the ray against each of the three slabs which make up a bounding box
	// if a direction is too small it might be represented by 0! check for this and give it 0.00001f
	float t1 = (min.x - ray.origin.x) / noZeroDirection(ray.direction.x);
	float t2 = (max.x - ray.origin.x) / noZeroDirection(ray.direction.x);
	float t3 = (min.y - ray.origin.y) / noZeroDirection(ray.direction.y);
	float t4 = (max.y - ray.origin.y) / noZeroDirection(ray.direction.y);
	float t5 = (min.z - ray.origin.z) / noZeroDirection(ray.direction.z);
	float t6 = (max.z - ray.origin.z) / noZeroDirection(ray.direction.z);

	// To find the point of entry, we need to find the largest minimum value.
	// To find the point of exit, we need to find the smallest minimum value.

	// Find largest min value
	float tmin = fmaxf(fmaxf(fminf(t1, t2),fminf(t3, t4)),fminf(t5, t6));

	// Find smallex max val
	float tmax = fminf(fminf(fmaxf(t1, t2),fmaxf(t3, t4)),fmaxf(t5, t6));

	// Check if intersects from behind
	if (tmax < 0) {
		return false;
	}

	// Check for general intersection
	if (tmin > tmax) {
		return false;
	}

	// Check if ray origin is inside of the box
	// tmin < zero => yes and tmax is valid collision point
	float t = tmin < 0.0f ? tmax : tmin;

	// Calculate Raycast result
	if (storeResult != nullptr) {
		storeResult->hit = true;
		storeResult->t = t;
		storeResult->point = ray.origin + ray.direction * t;

		vec3 normals[] = {
			vec3(-1, 0, 0),
			vec3(1, 0, 0),
			vec3(0, -1, 0),
			vec3(0, 1, 0),
			vec3(0, 0, -1),
			vec3(0, 0, 1)
		};

		if (CMP(t, t1)) storeResult->normal = AsNormal(normals[0]);
		if (CMP(t, t2)) storeResult->normal = AsNormal(normals[1]);
		if (CMP(t, t3)) storeResult->normal = AsNormal(normals[2]);
		if (CMP(t, t4)) storeResult->normal = AsNormal(normals[3]);
		if (CMP(t, t5)) storeResult->normal = AsNormal(normals[4]);
		if (CMP(t, t6)) storeResult->normal = AsNormal(normals[5]);

	}

	return true;
}

bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* storeResult)
{
	// Reset result (just in case)
	ResetRaycastResult(storeResult);

	const float* o = obb.orientation.asArray;
	const float* size = obb.size.asArray;
	// X, Y and Z axis of OBB
	vec3 X(o[0], o[1], o[2]);
	vec3 Y(o[3], o[4], o[5]);
	vec3 Z(o[6], o[7], o[8]);

	// Find a vector pointing from origin of ray to OBB
	vec3 rayAsVector = obb.position - ray.origin;

	// Project the direction onto each of the axis
	vec3 dirAxisProj(
		Dot(X, ray.direction),
		Dot(Y, ray.direction),
		Dot(Z, ray.direction)
	);

	// Project vector P onto each axis of rotation
	vec3 rayAxisProj(
		Dot(X, rayAsVector),
		Dot(Y, rayAsVector),
		Dot(Z, rayAsVector)
	);

	// Calcualte all max and mins line in AABB
	if (CMP(dirAxisProj.x, 0)) {
		if (-rayAxisProj.x - obb.size.x > 0 || -rayAxisProj.x + obb.size.x < 0) {
			return false;
		}
		dirAxisProj.x = 0.00001f; // Avoid div by 0!
	}
	else if (CMP(dirAxisProj.y, 0)) {
		if (-rayAxisProj.y - obb.size.y > 0 || -rayAxisProj.y + obb.size.y < 0) {
			return false;
		}
		dirAxisProj.y = 0.00001f; // Avoid div by 0!
	}
	else if (CMP(dirAxisProj.z, 0)) {
		if (-rayAxisProj.z - obb.size.z > 0 || -rayAxisProj.z + obb.size.z < 0) {
			return false;
		}
		dirAxisProj.z = 0.00001f; // Avoid div by 0!
	}

	float t1 = (rayAxisProj.x + obb.size.x) / dirAxisProj.x;
	float t2 = (rayAxisProj.x - obb.size.x) / dirAxisProj.x;
	float t3 = (rayAxisProj.y + obb.size.y) / dirAxisProj.y;
	float t4 = (rayAxisProj.y - obb.size.y) / dirAxisProj.y;
	float t5 = (rayAxisProj.z + obb.size.z) / dirAxisProj.z;
	float t6 = (rayAxisProj.z - obb.size.z) / dirAxisProj.z;

	float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
	float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

	// tmax < 0 => ray origin from behind (doesn't count)
	if (tmax < 0) {
		return false;
	}

	// tmin > tmax => ray does not intersect
	if (tmin > tmax) {
		return false;
	}

	// tmin < 0 => origin of ray is inside OBB (tmax is closer)
	float t = tmin < 0.0f ? tmax : tmin;
	
	// Calculate Raycast result
	if (storeResult != nullptr) {
		storeResult->hit = true;
		storeResult->t = t;
		storeResult->point = ray.origin + ray.direction * t;

		vec3 normals[] = {
			X,			// +x
			X * -1.0f,	// -x
			Y,			// +y
			Y * -1.0f,	// -y
			Z,			// +z
			Z * -1.0f	// -z
		};

		if (CMP(t, t1)) storeResult->normal = AsNormal(normals[0]);
		if (CMP(t, t2)) storeResult->normal = AsNormal(normals[1]);
		if (CMP(t, t3)) storeResult->normal = AsNormal(normals[2]);
		if (CMP(t, t4)) storeResult->normal = AsNormal(normals[3]);
		if (CMP(t, t5)) storeResult->normal = AsNormal(normals[4]);
		if (CMP(t, t6)) storeResult->normal = AsNormal(normals[5]);

	}

	return true;
}

bool Raycast(const Plane& plane, const Ray& ray, RaycastResult* storeResult)
{
	// Reset result (just in case)
	ResetRaycastResult(storeResult);

	float rd_pn = Dot(ray.direction, plane.normal);
	float ro_pn = Dot(ray.origin, plane.normal);

	// nd has to be < 0 for ray to intersect
	// else normal of plane and ray point in same direction
	if (rd_pn >= 0.0f) { return false; }

	// obtain time
	float t = (plane.distance - ro_pn) / rd_pn;

	// check if ray is hit plane from behind (i.e no hit)
	if (t >= 0.0f) {

		if (storeResult != nullptr) {
			storeResult->t = t;
			storeResult->hit = true;
			storeResult->point = ray.origin + ray.direction * t;
			storeResult->normal = AsNormal(plane.normal);
		}
		return true;
	}
	return false;
}

bool Raycast(const Triangle& triangle, const Ray& ray, RaycastResult* storeResult)
{
	// Reset result (just in case)
	ResetRaycastResult(storeResult);

	// Create a plane and raycast to plane if no hit => no hit on triangle
	Plane plane = FromTriangle(triangle);
	RaycastResult planeRaycastResult;
	if (!Raycast(plane, ray, &planeRaycastResult)) return false;

	// Get a point along plane where ray hit
	float t = planeRaycastResult.t;

	// Afterwards check for barycentric coords beacause
	// barycentric gives if a point is inside the "volume" of a triangle

	// Find point on plane that was hit
	Point result = ray.origin + ray.direction * t;

	// Find barycentric coords of the point on the triangle
	vec3 barycentric = Barycentric(result, triangle);

	// If point is within triangle ray hit it
	if (barycentric.x >= 0.0f && barycentric.x <= 1.0f &&
		barycentric.y >= 0.0f && barycentric.y <= 1.0f &&
		barycentric.z >= 0.0f && barycentric.z <= 1.0f) {
		if (storeResult != nullptr) {
			storeResult->t = t;
			storeResult->hit = true;
			storeResult->point = ray.origin + ray.direction * t;
			storeResult->normal = plane.normal;
		}
		return true;
	}

	return false;
}

vec3 Barycentric(const Point& p, const Triangle& t)
{
	// Find vectors from test _p to points of _t
	vec3 ap = p - t.a;
	vec3 bp = p - t.b;
	vec3 cp = p - t.c;

	// Get edges of _t as vectors
	vec3 ab = t.b - t.a;
	vec3 ac = t.c - t.a;
	vec3 bc = t.c - t.b;
	vec3 cb = t.b - t.c;
	vec3 ca = t.a - t.c;

	// Vec _v will be perpendicular to _edge. _p is projected onto this perp vector
	// if value is 0 => proj_p is on _edge, if 1 => is on the third point of triangle
	vec3 v = ab - Project(ab, cb);
	float a = 1.0f - (Dot(v, ap) / Dot(v, ab)); // 1 => proj_p at C

	v = bc - Project(bc, ac);
	float b = 1.0f - (Dot(v, bp) / Dot(v, bc)); // 1 => proj_p at A

	v = ca - Project(ca, ab);
	float c = 1.0f - (Dot(v, cp) / Dot(v, ca)); // 1 => proj_p at B

	// return projection
	return vec3(a, b, c);
}


// Linetests
// -----------------------------------
bool Linetest(const Sphere& sphere, const Line& line)
{
	// Get closes point to sphere
	Point closest = GetClosestPoint(line, sphere.position);
	// find the squared distance between the closest point and the centre of the sphere
	float distSq = MagnitudeSq(sphere.position - closest);
	//compare that squared distance to the squared magnitude of the
	//sphere.If the distance is less than the magnitude, we have an intersection
	return distSq <= (sphere.radius * sphere.radius);
}

bool Linetest(const AABB& aabb, const Line& line)
{
	// Create ray
	Ray ray;
	ray.origin = line.start;
	ray.direction = AsNormal(line.end - line.start);
	// Raycast
	float t = Raycast(aabb, ray);

	// Check if there is intersection and if distance is less than the lines maginitude
	return t >= 0 && t * t <= LengthSq(line);
}

bool Linetest(const OBB& obb, const Line& line)
{
	// Create ray from line origin
	Ray ray;
	ray.origin = line.start;

	// Raycast in direction of other line point
	ray.direction = AsNormal(line.end - line.start);
	float t = Raycast(obb, ray);
	// Check for intersection and if length from origin is in line bounds
	return t >= 0 && t * t <= LengthSq(line);
}

bool Linetest(const Plane& plane, const Line& line)
{
	// Line vector
	vec3 ab = line.end - line.start;

	// Project start point onto plane normal
	float nA = Dot(plane.normal, line.start);
	// Project end point onto plane normal
	float nAB = Dot(plane.normal, ab);

	// check if plane and line are parallel
	if (nAB = 0.0f) return false;

	// Get distance
	float t = (plane.distance - nA) / nAB;

	// Check to see for intersection
	return t >= 0.0f && t <= 1.0f;
}

bool Linetest(const Triangle& triangle, const Line& line)
{
	// Create ray
	Ray ray;
	ray.origin = line.start;
	ray.direction = AsNormal(line.end - line.start);

	// Raycast
	RaycastResult raycast;

	if (!Raycast(triangle, ray, &raycast)) {
		return false;
	}

	float t = raycast.t;

	// Check if raycast is onto line
	return t >= 0 && t * t <= LengthSq(line);
}



// Triangle collisions
// -----------------------------------
bool PointInTriangle(const Point& p, const Triangle& t)
{
	// temp triangle in the point local system
	vec3 a = t.a - p;
	vec3 b = t.b - p;
	vec3 c = t.c - p;

	// Create sides of pyramid from P and ABC triangle then store sides' normals
	vec3 normPBC = Cross(b, c); // Normal of PBC (u)
	vec3 normPCA = Cross(c, a); // Normal of PCA (v)
	vec3 normPAB = Cross(a, b); // Normal of PAB (w)


	// If the faces of the pyramid do not have the same normal, the point is not contained within the triangle
	if (Dot(normPBC, normPCA) < 0.0f) {
		return false;
	}
	else if (Dot(normPBC, normPAB) < 0.0f) {
		return false;
	}
	// Same normals => flat pyramid (no volume) => point in triangle
	return true;
}

Plane FromTriangle(const Triangle& t)
{
	// Create a plane from a triangle
	Plane result;
	result.normal = AsNormal(Cross(t.b - t.a, t.c - t.a));

	// project A/B/C point to the normal of the plane to 
	// get the distane between plane and origin
	result.distance = Dot(result.normal, t.a);
	return result;
}

Point GetClosestPoint(const Triangle& t, const Point& p)
{
	// Obtain plane from triangle
	Plane plane = FromTriangle(t);

	// Get closes point of P to the plane
	Point closest = GetClosestPoint(plane, p);

	// Check to see if closest point is in triangle
	if (PointInTriangle(closest, t)) {
		return closest;
	}
	// Else: Construct one line for each side of the triangle and test for point
	Point c1 = GetClosestPoint(Line(t.a, t.b), p); // Line AB
	Point c2 = GetClosestPoint(Line(t.b, t.c), p); // Line BC
	Point c3 = GetClosestPoint(Line(t.c, t.a), p); // Line CA

	// Measure how far each is from the test point
	float magSq1 = MagnitudeSq(p - c1);
	float magSq2 = MagnitudeSq(p - c2);
	float magSq3 = MagnitudeSq(p - c3);

	// return closest
	if (magSq1 < magSq2 && magSq1 < magSq3) {
		return c1;
	}
	else if (magSq2 < magSq1 && magSq2 < magSq3) {
		return c2;
	}
	return c3;
}

bool TriangleSphere(const Triangle& t, const Sphere& s)
{
	// get closest point of triangle to sphere's position
	Point closest = GetClosestPoint(t, s.position);

	// Get distance between closest point and sphere position
	float magSq = MagnitudeSq(closest - s.position);

	// Check if distance is longer than radius
	return magSq <= s.radius * s.radius;
}

Interval GetIntervalProjection(const Triangle& triangle, const vec3& axis)
{
	Interval result;

	// Project the first point of the triangle onto the axis
	result.min = Dot(axis, triangle.points[0]);
	result.max = result.min;

	// Project other 2 points on the axis. Store them accordingly on min/max value
	for (int i = 1; i < 3; ++i) {
		float value = Dot(axis, triangle.points[i]);
		result.min = fminf(result.min, value);
		result.max = fmaxf(result.max, value);
	}

	return result;
}

bool AreOverlapingOnAxis(const AABB& aabb, const Triangle& triangle, const vec3& axis)
{
	// Get interval of _aabb onto _axiss
	Interval a = GetIntervalProjection(aabb, axis);
	// Get interval of _triangle onto _axis
	Interval b = GetIntervalProjection(triangle, axis);
	// Compare for intersection
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool TriangleAABB(const Triangle& t, const AABB& a)
{
	// Find edge vectors of _triangle
	vec3 f0 = t.b - t.a;
	vec3 f1 = t.c - t.b;
	vec3 f2 = t.a - t.c;

	// Find face normals of _AABB
	vec3 u0(1.0f, 0.0f, 0.0f);
	vec3 u1(0.0f, 1.0f, 0.0f);
	vec3 u2(0.0f, 0.0f, 1.0f);

	// Declare test axes
	vec3 test[13] = {
		u0, // AABB Axis 1
		u1, // AABB Axis 2
		u2, // AABB Axis 3
		Cross(f0, f1), // Normal of triangle => then cross products of _AABB normals and _triangle eges
		Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
		Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
		Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)
	};

	// Test for overlaping axes to find separating AXIS
	for (int i = 0; i < 13; ++i) {
		if (!AreOverlapingOnAxis(a, t, test[i])) {
				return false; // Separating axis found
		}
	}

	return true; // Separating axis not found
}

bool AreOverlapingOnAxis(const OBB& obb, const Triangle& triangle, const vec3& axis)
{
	// Get interval of _aabb onto _axis
	Interval a = GetIntervalProjection(obb, axis);
	// Get interval of _triangle onto _axis
	Interval b = GetIntervalProjection(triangle, axis);
	// Compare for intersection
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool TriangleOBB(const Triangle& t, const OBB& o)
{
	// Find edge vectors of _triangle
	vec3 f0 = t.b - t.a;
	vec3 f1 = t.c - t.b;
	vec3 f2 = t.a - t.c;

	// Get face normals of OBB
	const float* orientation = o.orientation.asArray;
	vec3 u0(orientation[0], orientation[1], orientation[2]);
	vec3 u1(orientation[3], orientation[4], orientation[5]);
	vec3 u2(orientation[6], orientation[7], orientation[8]);

	// Declare test axes
	vec3 test[13] = {
		u0, // _OBB Axis 1
		u1, // _OBB Axis 2
		u2, // _OBB Axis 3
		Cross(f0, f1), // Normal of triangle => then cross products of _AABB normals and _triangle eges
		Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
		Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
		Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)
	};

	// Test for overlaping axes to find separating AXIS
	for (int i = 0; i < 13; ++i) {
		if (!AreOverlapingOnAxis(o, t, test[i])) {
			return false; // Separating axis found
		}
	}

	return true;
}

bool TrianglePlane(const Triangle& t, const Plane& p)
{
	// Check _triangle points position to sides of plane
	float side1 = PlaneEquation(t.a, p);
	float side2 = PlaneEquation(t.b, p);
	float side3 = PlaneEquation(t.c, p);

	// If all on plane => triangle is on the plane (coplanar)
	if (CMP(side1, 0) && CMP(side2, 0) && CMP(side3, 0)) {
		return true;
	}

	// If all points are infront => no intersection
	if (side1 > 0 && side2 > 0 && side3 > 0) {
		return false;
	}

	// If all points are behind => no intersection
	if (side1 < 0 && side2 < 0 && side3 < 0) {
		return false;
	}

	// Else intersects somewhere
	return true;
}

bool AreOverlapingOnAxis(const Triangle& t1, const Triangle& t2, const vec3& axis)
{
	// Get interval of _triangle onto _axis
	Interval a = GetIntervalProjection(t1, axis);
	// Get interval of _aabb onto _axis
	Interval b = GetIntervalProjection(t2, axis);
	// Compare for intersection
	return ((b.min <= a.max) && (a.min <= b.max));
	return false;
}

bool TriangleTriangle(const Triangle& t1, const Triangle& t2)
{
	// Get T1 edges
	vec3 t1_f0 = t1.b - t1.a;
	vec3 t1_f1 = t1.c - t1.b;
	vec3 t1_f2 = t1.a - t1.c;

	// Get T2 edges
	vec3 t2_f0 = t2.b - t2.a; 
	vec3 t2_f1 = t2.c - t2.b; 
	vec3 t2_f2 = t2.a - t2.c;

	// Get potentially separating axis
	vec3 axisToTest[] = {
		Cross(t1_f0, t1_f1), // t1 normal
		Cross(t2_f0, t2_f1), // t2 normal
		// Cross product of each edge of t1 to each edge of t2
		Cross(t2_f0, t1_f0), Cross(t2_f0, t1_f1),
		Cross(t2_f0, t1_f2), Cross(t2_f1, t1_f0),
		Cross(t2_f1, t1_f1), Cross(t2_f1, t1_f2),
		Cross(t2_f2, t1_f0), Cross(t2_f2, t1_f1),
		Cross(t2_f2, t1_f2)
	};

	// Find separating axis
	for (int i = 0; i < 11; ++i) {
		if (!AreOverlapingOnAxis(t1, t2, axisToTest[i])) {
			return false; // Seperating axis found
		}
	}

	return true; // no separating axis found
}

vec3 SatCrossEdge(const vec3& a, const vec3& b, const vec3& c, const vec3& d)
{
	// Create default sides and find their cross product
	vec3 ab = a - b;
	vec3 cd = c - d;
	vec3 result = Cross(ab, cd);

	// Check if cross product is not 0 => they are not parallel => can use the axis
	if (!CMP(MagnitudeSq(result), 0)) {
		return result; // Not parallel!
	}
	else {
		// Construct a temp axis perp to side of one triangle
		vec3 axis = Cross(ab, c - a);
		result = Cross(ab, axis);

		// Check for parallelism again
		if (!CMP(MagnitudeSq(result), 0)) {
			return result; // Not parallel
		}
	}
	// If both test axis are tested parallel then 
	// no way to get a proper cross product between them
	return vec3();
}

bool TriangleTriangleRobust(const Triangle& t1, const Triangle& t2)
{
	vec3 axisToTest[] = {
		SatCrossEdge(t1.a, t1.b, t1.b, t1.c),
		SatCrossEdge(t2.a, t2.b, t2.b, t2.c),
		SatCrossEdge(t2.a, t2.b, t1.a, t1.b),
		SatCrossEdge(t2.a, t2.b, t1.b, t1.c),
		SatCrossEdge(t2.a, t2.b, t1.c, t1.a),
		SatCrossEdge(t2.b, t2.c, t1.a, t1.b),
		SatCrossEdge(t2.b, t2.c, t1.b, t1.c),
		SatCrossEdge(t2.b, t2.c, t1.c, t1.a),
		SatCrossEdge(t2.c, t2.a, t1.a, t1.b),
		SatCrossEdge(t2.c, t2.a, t1.b, t1.c),
		SatCrossEdge(t2.c, t2.a, t1.c, t1.a)
	};

	// Find separating axis
	for (int i = 0; i < 11; ++i) {
		if (!AreOverlapingOnAxis(t1, t2, axisToTest[i])) {
			return false; // Seperating axis found
		}
	}

	return true; // no separating axis found
}

#define OPTIMAL_MESH_SPLIT_DEPTH 3
// 8 = 2^3 split each dimension by 2
#define BHV_MESH_CHILD_SPLIT_COUNT 8

void AccelerateMesh(Mesh& mesh)
{
	if (mesh.accelerator != 0)	return;

	//Find the minimum and maximum points of the mesh
	vec3 min = mesh.vertices[0];
	vec3 max = mesh.vertices[0];
	for (int i = 1; i < mesh.numTriangles * 3; ++i) {
		min.x = fminf(mesh.vertices[i].x, min.x);
		min.y = fminf(mesh.vertices[i].y, min.y);
		min.z = fminf(mesh.vertices[i].z, min.z);
		max.x = fmaxf(mesh.vertices[i].x, max.x);
		max.y = fmaxf(mesh.vertices[i].y, max.y);
		max.z = fmaxf(mesh.vertices[i].z, max.z);
	}

	// Create a new accelerator structure within the mesh
	// then set the AABB bounds to the min and max points of the mash
	mesh.accelerator = new BVHNode();
	mesh.accelerator->bounds = FromMinMax(min, max);
	mesh.accelerator->numTriangles = mesh.numTriangles;

	// Allocate memory for indices of mesh triangles 
	// Then store to prevent duplication
	mesh.accelerator->triangles =
		new int[mesh.numTriangles];

	// Store indices in the accelerator 
	for (int i = 0; i < mesh.numTriangles; ++i) {
		mesh.accelerator->triangles[i] = i;
	}

	// Recursively split the tree
	SplitBVHNode(mesh.accelerator, mesh, OPTIMAL_MESH_SPLIT_DEPTH);
}

void SplitBVHNode(BVHNode* node, const Mesh& model, int depth)
{
	// Stop recursion if reached given depth
	if (depth-- == 0) {
		return;
	}

	// if node is a leaf and contains triangles => split it into 8 child nodes
	if (node->children == 0) {
		if (node->numTriangles > 0) {
			node->children = new BVHNode[BHV_MESH_CHILD_SPLIT_COUNT];
			vec3 centre = node->bounds.origin;
			vec3 split_offset = node->bounds.size * 0.5f;
			node->children[0].bounds =
				AABB(centre + vec3(-split_offset.x, +split_offset.y, -split_offset.z), split_offset);
			node->children[1].bounds =
				AABB(centre + vec3(+split_offset.x, +split_offset.y, -split_offset.z), split_offset);
			node->children[2].bounds =
				AABB(centre + vec3(-split_offset.x, +split_offset.y, +split_offset.z), split_offset);
			node->children[3].bounds =
				AABB(centre + vec3(+split_offset.x, +split_offset.y, +split_offset.z), split_offset);
			node->children[4].bounds =
				AABB(centre + vec3(-split_offset.x, -split_offset.y, -split_offset.z), split_offset);
			node->children[5].bounds =
				AABB(centre + vec3(+split_offset.x, -split_offset.y, -split_offset.z), split_offset);
			node->children[6].bounds =
				AABB(centre + vec3(-split_offset.x, -split_offset.y, +split_offset.z), split_offset);
			node->children[7].bounds =
				AABB(centre + vec3(+split_offset.x, -split_offset.y, +split_offset.z), split_offset);
		}
	}

	// If node was just split
	// Assign each child node its triangles
	if (node->children != 0 && node->numTriangles > 0) {
		for (int i = 0; i < 8; ++i) { // For each child

			// Count triangles a child contains
			node->children[i].numTriangles = 0;
			for (int j = 0; j < node->numTriangles; ++j) {
				Triangle t = model.triangles[node->triangles[j]];

				// For every intersection increase triangle count for node
				if (TriangleAABB(t, node->children[i].bounds))
					node->children[i].numTriangles += 1;
			}

			// If there are no triangles in the child node => do nothing
			if (node->children[i].numTriangles == 0) continue;

			// Else allocate new memory for the child node
			node->children[i].triangles = new int[node->children[i].numTriangles];

			// any triangle which intersects the child node being created,
			// add it's index to the list of triangle indices
			int index = 0;
			for (int j = 0; j < node->numTriangles; ++j) {
				Triangle t = model.triangles[node->triangles[j]];

				if (TriangleAABB(t, node->children[i].bounds)) {
					node->children[i].triangles[index++] = node->triangles[j];
				}
			}
		}

		// Now this node does not contain triangles
		// just childred on nodes
		node->numTriangles = 0;
		delete[] node->triangles;
		node->triangles = 0;

		// Recursively call on children
		for (int i = 0; i < 8; ++i) {
			SplitBVHNode(&node->children[i], model, depth);
		}
	}
}

void FreeBVHNode(BVHNode* node)
{
	// Recursively free the BVH node
	if (node->children != 0) {

		// Free children
		for (int i = 0; i < 8; ++i) FreeBVHNode(&node->children[i]);

		// Remove children
		delete[] node->children;
		node->children = 0;

		// If any triangle indices are present remove the array holding them
		if (node->numTriangles != 0 || node->triangles != 0) {
			delete[] node->triangles;
			node->triangles = 0;
			node->numTriangles = 0;
		}
	}
}

// Mesh tests
bool Linetest(const Mesh& mesh, const Line& line)
{
	// if no accelerator -> go through all triangles
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (Linetest(mesh.triangles[i], line)) {
				return true;
			}
		}
	}
	else {

		// Start from root
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			// if a leaf node -> process triangles
			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					if (Linetest(mesh.triangles[iterator->triangles[i]], line)) {
						return true;
					}
				}
			}

			// if node is not a leaf -> raycast over children
			// if children node is hit -> add it to process
			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (Linetest(iterator->children[i].bounds, line)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshSphere(const Mesh& mesh, const Sphere& sphere)
{
	// if no accelerator -> go through all triangles
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TriangleSphere(mesh.triangles[i], sphere)) {
				return true;
			}
		}
	}
	else {
		// Start from root
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			// if a leaf node -> process triangles
			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					if (TriangleSphere(mesh.triangles[iterator->triangles[i]], sphere)) {
						return true;
					}
				}
			}

			// if node is not a leaf -> raycast over children
			// if children node is hit -> add it to process
			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (SphereAABB(sphere, iterator->children[i].bounds)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshAABB(const Mesh& mesh, const AABB& aabb)
{
	// if no accelerator -> go through all triangles
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TriangleAABB(mesh.triangles[i], aabb)) {
				return true;
			}
		}
	}
	else {
		// process root node
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// recursively proccess it and its children
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			// if a leaf node -> process triangles
			if (iterator->numTriangles >= 0) {
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// The TirangleAABB test here would change
					// if we where testing a shape other than AABB
					if (TriangleAABB( mesh.triangles[iterator->triangles[i]], aabb)) {
						return true;
					}
				}
			}

			// if node is not a leaf -> raycast over children
			// if children node is hit -> add it to process
			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// The AABBAABB test here would change
					// if we where testing a shape other than AABB
					if (AABBAABB(iterator->children[i].bounds,aabb)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	// if he hit no triangle
	return false;
}

bool MeshOBB(const Mesh& mesh, const OBB& obb)
{
	// if no accelerator -> go through all triangles
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TriangleOBB(mesh.triangles[i], obb)) {
				return true;
			}
		}
	}
	else {
		// Start from root
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			// if a leaf node -> process triangles
			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					if (TriangleOBB(mesh.triangles[iterator->triangles[i]], obb)) {
						return true;
					}
				}
			}

			// if node is not a leaf -> raycast over children
			// if children node is hit -> add it to process
			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (AABBOBB(iterator->children[i].bounds, obb)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false; 
}

bool MeshPlane(const Mesh& mesh, const Plane& plane)
{
	// if no accelerator -> go through all triangles
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TrianglePlane(mesh.triangles[i], plane)) {
				return true;
			}
		}
	}
	else {
		// Start from root
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			// if a leaf node -> process triangles
			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					if (TrianglePlane(mesh.triangles[iterator->triangles[i]], plane)) {
						return true;
					}
				}
			}

			// if node is not a leaf -> raycast over children
			// if children node is hit -> add it to process
			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (AABBPlane(iterator->children[i].bounds, plane)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshTriangle(const Mesh& mesh, const Triangle& triangle)
{
	// if no accelerator -> go through all triangles
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			if (TriangleTriangle(mesh.triangles[i], triangle)) {
				return true;
			}
		}
	}
	else {
		// Start from root
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			// if a leaf node -> process triangles
			if (iterator->numTriangles >= 0) {
				// Iterate trough all triangles of the node
				for (int i = 0; i < iterator->numTriangles; ++i) {
					if (TriangleTriangle(mesh.triangles[iterator->triangles[i]], triangle)) {
						return true;
					}
				}
			}

			// if node is not a leaf -> raycast over children
			// if children node is hit -> add it to process
			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					if (TriangleAABB(triangle, iterator->children[i].bounds)) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

float MeshRay(const Mesh& mesh, const Ray& ray)
{
	// if no accelerator struct => check all triangles
	if (mesh.accelerator == 0) {
		for (int i = 0; i < mesh.numTriangles; ++i) {
			RaycastResult raycast;
			Raycast(mesh.triangles[i], ray, &raycast);
			float result = raycast.t;
			if (result >= 0) {
				return result;
			}
		}
	}
	else {
		// walk through the BVH tree
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		// Recursivley walk the BVH tree
		while (!toProcess.empty()) {

			// Get the current node
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			// if node has triangles => iterate over them
			if (iterator->numTriangles >= 0) {
				for (int i = 0; i < iterator->numTriangles; ++i) {
					// Do a raycast against the triangle
					RaycastResult raycast;
					Raycast(mesh.triangles[iterator->triangles[i]], ray, &raycast);
					float r = raycast.t;
					if (r >= 0.0f) { return r;}
				}
			}
			// if node is not a leaf perform a raycast agains bounds of each child
			// if it hits a child add the node to Process list
			if (iterator->children != 0) {
				for (int i = 8 - 1; i >= 0; --i) {
					// Only push children whos bounds intersect the test geometry
					RaycastResult raycast;
					Raycast(iterator->children[i].bounds, ray, &raycast);
					if (raycast.t >= 0) {
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return -1;
}

// MODEL class functionality
// ------------------------------
void Model::SetContent(Mesh* mesh)
{
	content = mesh;
	if (content != 0) {
		// calculate AABB of mesh
		vec3 min = mesh->vertices[0];
		vec3 max = mesh->vertices[0];

		for (int i = 1; i < mesh->numTriangles * 3; ++i) {
			min.x = fminf(mesh->vertices[i].x, min.x);
			min.y = fminf(mesh->vertices[i].y, min.y);
			min.z = fminf(mesh->vertices[i].z, min.z);
			max.x = fmaxf(mesh->vertices[i].x, max.x);
			max.y = fmaxf(mesh->vertices[i].y, max.y);
			max.z = fmaxf(mesh->vertices[i].z, max.z);
		}

		bounds = FromMinMax(min, max);
	}
}

mat4 GetWorldMatrix(const Model& model)
{
	mat4 translation = Translation(model.position);
	mat4 rotation = Rotation(
		model.rotation.x,
		model.rotation.y,
		model.rotation.z
	);

	// mult by scale if scale added
	mat4 localMat = rotation * translation;

	mat4 parentMat;
	if (model.parent != 0) {
		parentMat = GetWorldMatrix(*model.parent);
	}
	return localMat * parentMat;

}

OBB GetOBB(const Model& model)
{
	// obtain the world matrix
	mat4 world = GetWorldMatrix(model);
	// get the AABB bounds
	AABB aabb = model.GetBounds();
	// apply world transform to OBB
	OBB obb;
	obb.size = aabb.size;
	obb.position = MultiplyPoint(aabb.origin, world);
	obb.orientation = Cut(world, 3, 3);

	return obb;
}

// Model tests
float ModelRay(const Model& model, const Ray& ray)
{
	// Get inverse of model's world matrix
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	// Create ray and transform it by the matrix to get to model local space
	Ray local;
	local.origin = MultiplyPoint(ray.origin, inv);
	local.direction = MultiplyVector(ray.origin, inv);
	local.NormalizeDirection();

	// Test between the mesh and Ray in local space
	if (model.GetMesh() != 0) {
		return MeshRay(*(model.GetMesh()), local);
	}
	return -1;
}

bool Linetest(const Model& model, const Line& line)
{
	// Get inverse of model's world matrix
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	// Create line and transform it by the matrix to get to model local space
	Line local;
	local.start = MultiplyPoint(line.start, inv);
	local.end = MultiplyPoint(line.end, inv);

	// Test between the mesh and Line in local space
	if (model.GetMesh() != 0) {
		return Linetest(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelSphere(const Model& model, const Sphere& sphere)
{
	// Get inverse of model's world matrix
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	// Create Sphere and transform it by the matrix to get to model local space
	Sphere local;
	local.position = MultiplyPoint(sphere.position, inv);

	// Test between the mesh and Sphere in local space
	if (model.GetMesh() != 0) {
		return MeshSphere(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelAABB(const Model& model, const AABB& aabb)
{
	// Get inverse of model's world matrix
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	// Create AABB and transform it by the matrix to get to model local space
	// Because the inverse transform can have a rotation, the AABB will turn into an OBB
	OBB local;
	local.size = aabb.size;
	local.position = MultiplyPoint(aabb.origin, inv);
	local.orientation = Cut(inv, 3, 3);

	// Test between the mesh and AABB in local space
	if (model.GetMesh() != 0) {
		return MeshOBB(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelOBB(const Model& model, const OBB& obb)
{
	// Get inverse of model's world matrix
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	// Create OBB and transform it by the matrix to get to model local space
	OBB local;
	local.size = obb.size;
	local.position = MultiplyPoint(obb.position, inv);
	local.orientation = obb.orientation * Cut(inv, 3, 3);

	// Test between the mesh and OBB in local space
	if (model.GetMesh() != 0) {
		return MeshOBB(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelPlane(const Model& model, const Plane& plane)
{
	// Get inverse of model's world matrix
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	// Create Plane and transform it by the matrix to get to model local space
	Plane local;
	local.normal = MultiplyVector(plane.normal, inv);
	local.distance = plane.distance;

	// Test between the mesh and Plane in local space
	if (model.GetMesh() != 0) {
		return MeshPlane(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelTriangle(const Model& model, const Triangle& triangle)
{
	// Get inverse of model's world matrix
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	// Create Triangle and transform it by the matrix to get to model local space
	Triangle local;
	local.a = MultiplyPoint(triangle.a, inv);
	local.b = MultiplyPoint(triangle.b, inv);
	local.c = MultiplyPoint(triangle.c, inv);

	// Test between the mesh and Triangle in local space
	if (model.GetMesh() != 0) {
		return MeshTriangle(*(model.GetMesh()), local);
	}
	return false;
}



// FRUSTUM 
// ------------------------------------------
Point Intersection(Plane p1, Plane p2, Plane p3) {

	// Following "Cramer's Rule" to solve intersection of 3 planes
	/* http://www.purplemath.com/modules/cramers.htm. */


	// Create coefficient matrix composed of known quantities
	// for the system of equations for _p1, _p2, _p3	
	mat3 D(
		p1.normal.x, p2.normal.x, p3.normal.x,
		p1.normal.y, p2.normal.y, p3.normal.y,
		p1.normal.z, p2.normal.z, p3.normal.z
	);

	// Create a row matrix with the solution to each system
	vec3 A(-p1.distance, -p2.distance, -p3.distance);


	// create matrix which has one row replaced by the answer row (row 1)
	mat3 Dx = D;
	Dx._11 = A.x; Dx._12 = A.y; Dx._13 = A.z;

	// create matrix which has one row replaced by the answer row (row 2)
	mat3 Dy = D;
	Dy._21 = A.x; Dy._22 = A.y; Dy._23 = A.z;

	// create matrix which has one row replaced by the answer row (row 3)
	mat3 Dz = D;
	Dz._31 = A.x; Dz._32 = A.y; Dz._33 = A.z;


	// Find determinant of original matrix
	float detD = Determinant(D);
	if (CMP(detD, 0)) {
		return Point();
	}

	// Find determinant of answer matrices
	float detDx = Determinant(Dx);
	float detDy = Determinant(Dy);
	float detDz = Determinant(Dz);

	// Return the intersection point
	return Point(detDx / detD, detDy / detD, detDz / detD);
}

void GetCorners(const Frustum& f, vec3* outCorners) {
	outCorners[0] = Intersection(f._near, f.top, f.left);
	outCorners[1] = Intersection(f._near, f.top, f.right);
	outCorners[2] = Intersection(f._near, f.bottom, f.left);
	outCorners[3] = Intersection(f._near, f.bottom, f.right);
	outCorners[4] = Intersection(f._far, f.top, f.left);
	outCorners[5] = Intersection(f._far, f.top, f.right);
	outCorners[6] = Intersection(f._far, f.bottom, f.left);
	outCorners[7] = Intersection(f._far, f.bottom, f.right);
}

bool Intersects(const Frustum& f, const Point& p)
{
	// For each plane in frustum
	for (int i = 0; i < 6; ++i) {

		// Get point's position with respect to the plane
		vec3 normal = f.planes[i].normal;
		float dist = f.planes[i].distance;
		float side = Dot(p, normal) + dist;

		// If the point is behind any of the planes, there is no intersection
		// i.e. point is not inside the camera frustum
		// points on the planes are considered not in the frustum
		if (side < 0.0f) {
			return false;
		}
	}
	return true;
}

bool Intersects(const Frustum& f, const Sphere& s)
{
	for (int i = 0; i < 6; ++i) {

		// Get sphere's origin with respect to the plane
		vec3 normal = f.planes[i].normal;
		float dist = f.planes[i].distance;
		float side = Dot(s.position, normal) + dist;

		// Check if the sphere is outside of frustum 
		// Spheres touching the planes are not considered inside
		if (side + s.radius < 0.0f) {
			return false;
		}
	}
	return true;
}

float Classify(const AABB& aabb, const Plane& plane)
{
	// Find positive extends of the AABB projected onto plane's normal
	// Similar to DOT product but returns positive number always
	float r = fabsf(aabb.size.x * plane.normal.x)
		+ fabsf(aabb.size.y * plane.normal.y)
		+ fabsf(aabb.size.z * plane.normal.z);

	// Find distance between origin of AABB and the plane
	float d = Dot(plane.normal, aabb.origin) + plane.distance;

	// if the AABB's extends are longer than the distance of the origin to the plane
	// => it intersects the frustum
	if (fabsf(d) < r) {
		return 0.0f;
	}
	
	// Return positive number if AABB is infront of a plane
	else if (d < 0.0f) {
		return d + r;
	}
	// Return negative number if AABB is behind the plane
	return d - r;
}

float Classify(const OBB& obb, const Plane& plane)
{

	// First transform plane normal to OBB local space -> becomes a plane to AABB
	vec3 normal = MultiplyVector(plane.normal, obb.orientation);

	// Find positive extends of the OBB projected onto plane's normal
	// Similar to DOT product but returns positive number always
	float r = fabsf(obb.size.x * plane.normal.x)
		+ fabsf(obb.size.y * plane.normal.y)
		+ fabsf(obb.size.z * plane.normal.z);

	// Find distance between origin of OBB and the plane
	float d = Dot(plane.normal, obb.position) + plane.distance;

	// if the AABB's extends are longer than the distance of the origin to the plane
	// => it intersects the frustum
	if (fabsf(d) < r) {
		return 0.0f;
	}

	// Return positive number if OBB is infront of a plane
	else if (d < 0.0f) {
		return d + r;
	}
	// Return negative number if OBB is behind the plane
	return d - r;
}

bool Intersects(const Frustum& f, const AABB& aabb)
{
	// Check the AABB against each plane of the frustum
	for (int i = 0; i < 6; ++i) 
		if (Classify(aabb, f.planes[i]) < 0) return false;

	return true;
}

bool Intersects(const Frustum& f, const OBB& obb)
{
	// Check the AABB against each plane of the frustum
	for (int i = 0; i < 6; ++i)
		if (Classify(obb, f.planes[i]) < 0) return false;

	return true;
}

vec3 Unproject(const vec3& viewportPoint, 
	const vec2& viewportOrigin, 
	const vec2& viewportSize, 
	const mat4& view, 
	const mat4& projection)
{

	// Normalize input vector to Viewport
	float normalized[4] = {
		(viewportPoint.x - viewportOrigin.x) / viewportSize.x,
		(viewportPoint.y - viewportOrigin.y) / viewportSize.y,
		viewportPoint.z,
		1.0f
	};

	// translate vector into NDC space
	float ndcSpace[4] = { normalized[0], normalized[1], normalized[2], normalized[3]};

	// input is from {0 to 1} -> NDC { -1 to 1}
	ndcSpace[0] = ndcSpace[0] * 2.0f - 1.0f; // X range
	ndcSpace[1] = 1.0f - ndcSpace[1] * 2.0f; // Y range

	// For Direct3D clamp Z: { 0 to 1}
	if (ndcSpace[2] < 0.0f) { // for OpenGL can use the above method instead
		ndcSpace[2] = 0.0f;
	}
	if (ndcSpace[2] > 1.0f) {
		ndcSpace[2] = 1.0f;
	}

	// Transform NDC vector into eye space
	mat4 invProjection = Inverse(projection);
	
	// This will transform the NDC space to eye space
	// eyeSpace = MultiplyPoint(ndcSpace, invProjection);
	float eyeSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	Multiply(eyeSpace, ndcSpace, 1, 4, invProjection.asArray, 4, 4);

	mat4 invView = Inverse(view);
	// Translate from eye space to world space
	// worldSpace = MultiplyPoint(eyeSpace, invView);
	float worldSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	Multiply(worldSpace, eyeSpace, 1, 4, invView.asArray, 4, 4);

	// Undo the prespective divide (fourth component of world space vector)
	if (!CMP(worldSpace[3], 0.0f)) {
		worldSpace[0] /= worldSpace[3];
		worldSpace[1] /= worldSpace[3];
		worldSpace[2] /= worldSpace[3];
	}
	// Can add worldSpace[3] /= worldspace [3]; and return a vec4
	return vec3(worldSpace[0], worldSpace[1],worldSpace[2]);
}

Ray GetPickRay(const vec2& viewportPoint, 
	const vec2& viewportOrigin, 
	const vec2& viewportSize, 
	const mat4& view, 
	const mat4& projection)
{

	// Construct near and far point
	vec3 nearPoint(viewportPoint.x, viewportPoint.y, 0.0f); // OpenGL should use -1.0f instead of 0.0f
	vec3 farPoint(viewportPoint.x, viewportPoint.y, 1.0f);

	// Unproject near and far points to create the ray components
	vec3 pNear = Unproject(nearPoint, viewportOrigin, viewportSize, view, projection);
	vec3 pFar = Unproject(farPoint, viewportOrigin, viewportSize, view, projection);

	// Create ray
	vec3 normal = AsNormal(pFar - pNear);
	vec3 origin = pNear;

	// ray can be used to raycast into a scene from the perspective of the viewer
	return Ray(origin, normal);

	 // Picking use case 
	/*
		vec2 screenOrigin = vec2(0.0f, 0.0f);
		vec3 screenSize = vec2(GetWidth(), GetHeight());
		mat4 view = camera.GetViewMatrix();
		mat4 projection = camera.GetProjectionMatrix();
		Ray ray = GetPickRay(mousePosition, screenOrigin, screenSize, view, projection);
		std::vector<Model*> visible = scene->Cull(camera.GetFrustum());
		Model* selectedModel = scene->Raycast(ray);
		for (int i = 0; i<visible.size(); ++i) {
			if (visible[i] == selectedModel) {
				// TODO: Indicate that current model is selected
			}
			Render(visible[i]);
		}
	*/
}


// COLISION MANIFOLDS 

// Helper function to get items from OBB
std::vector<Point> GetVertices(const OBB& obb)
{
	std::vector<vec3> v;
	v.resize(8); // A box has 8 vertices

	vec3 center = obb.position;
	vec3 extents = obb.size;
	const float* o = obb.orientation.asArray;
	vec3 axes[] = {
		vec3(o[0], o[1], o[2]),
		vec3(o[3], o[4], o[5]),
		vec3(o[6], o[7], o[8]),
	};

	v[0] = center + axes[0] * extents[0] + axes[1] * extents[1] + axes[2] * extents[2];
	v[1] = center - axes[0] * extents[0] + axes[1] * extents[1] + axes[2] * extents[2];
	v[2] = center + axes[0] * extents[0] - axes[1] * extents[1] + axes[2] * extents[2];
	v[3] = center + axes[0] * extents[0] + axes[1] * extents[1] - axes[2] * extents[2];
	v[4] = center - axes[0] * extents[0] - axes[1] * extents[1] - axes[2] * extents[2];
	v[5] = center + axes[0] * extents[0] - axes[1] * extents[1] - axes[2] * extents[2];
	v[6] = center - axes[0] * extents[0] + axes[1] * extents[1] - axes[2] * extents[2];
	v[7] = center - axes[0] * extents[0] - axes[1] * extents[1] + axes[2] * extents[2];

	return v;
}
std::vector<Line> GetEdges(const OBB& obb)
{
	std::vector<Line> result;
	result.reserve(12);				// Box has 12 edges

	// We can create vertices from edges
	std::vector<Point> vertices = GetVertices(obb);

	int index_of_edges[][2] = { // Indices of edges
		{ 6, 1 },{ 6, 3 },{ 6, 4 },{ 2, 7 },{ 2, 5 },{ 2, 0 },
		{ 0, 1 },{ 0, 3 },{ 7, 1 },{ 7, 4 },{ 4, 5 },{ 5, 3 }
	};

	for (int j = 0; j < 12; ++j) {
		result.push_back(Line(vertices[index_of_edges[j][0]], vertices[index_of_edges[j][1]]));
	}

	return result;
}
std::vector<Plane> GetPlanes(const OBB& obb)
{
	vec3 center = obb.position;	// OBB Center
	vec3 extents = obb.size;		// OBB Extents
	const float* o = obb.orientation.asArray;
	vec3 axes[] = {			// OBB Axis
		vec3(o[0], o[1], o[2]),
		vec3(o[3], o[4], o[5]),
		vec3(o[6], o[7], o[8]),
	};

	std::vector<Plane> result;
	result.resize(6);

	result[0] = Plane(axes[0], Dot(axes[0], (center + axes[0] * extents.x)));
	result[1] = Plane(axes[0] * -1.0f, -Dot(axes[0], (center - axes[0] * extents.x)));
	result[2] = Plane(axes[1], Dot(axes[1], (center + axes[1] * extents.y)));
	result[3] = Plane(axes[1] * -1.0f, -Dot(axes[1], (center - axes[1] * extents.y)));
	result[4] = Plane(axes[2], Dot(axes[2], (center + axes[2] * extents.z)));
	result[5] = Plane(axes[2] * -1.0f, -Dot(axes[2], (center - axes[2] * extents.z)));

	return result;
}

bool ClipToPlane(const Plane& plane, const Line& line, Point* outPoint) {

	vec3 line_dir = line.end - line.start;

	// Check if plane intersects
	float nA = Dot(plane.normal, line.start);
	float nAB = Dot(plane.normal, line_dir);
	if (CMP(nAB, 0.0f)) {
		return false;	// they dont so cant clip
	}

	// time(position) along the line which it intersects
	float t = (plane.distance - nA) / nAB;

	// return point of intersection 
	if (t >= 0.0f && t <= 1.0f) {
		if (outPoint != nullptr) *outPoint = line.start + line_dir * t;

		return true;
	}

	return false;
}
std::vector<Point> ClipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb) {
	std::vector<Point> result;
	result.reserve(edges.size() * 3);
	Point intersection;

	std::vector<Plane> planes = GetPlanes(obb);

	// Go through planes and edges to try to clip current edge ot current plane
	for (int i = 0; i < planes.size(); ++i) {
		for (int j = 0; j < edges.size(); ++j) {
			if (ClipToPlane(planes[i], edges[j], &intersection)) {
				// if intersection record the point
				if (IsPointInOBB(intersection, obb)) {
					result.push_back(intersection);
				}
			}
		}
	}

	return result;
}
float PenetrationDepth(const OBB& o1, const OBB& o2, const vec3& axis, bool* flipNormalFlag) {
	// Get both intervals and check for intersections
	Interval i1 = GetIntervalProjection(o1, AsNormal(axis));
	Interval i2 = GetIntervalProjection(o2, AsNormal(axis));

	if (!((i2.min <= i1.max) && (i1.min <= i2.max))) {
		return 0.0f; // No overlap = no penetration
	}

	float len1 = i1.max - i1.min;
	float len2 = i2.max - i2.min;
	float min = fminf(i1.min, i2.min);
	float max = fmaxf(i1.max, i2.max);
	float length = max - min;

	// if second OBB is in front of first one we need to flip the collision normal
	if (flipNormalFlag != nullptr) *flipNormalFlag = (i2.min < i1.min);

	return (len1 + len2) - length;
}
void ResetCollisionManifold(CollisionManifold* result) {
	if (result != 0) {
		result->colliding = false;
		result->normal = vec3(0, 0, 1);
		result->depth = FLT_MAX;
		result->contacts.clear();
	}
}

// Create collision manifolds
CollisionManifold FindCollisionFeatures(const Sphere& A, const Sphere& B)
{
	CollisionManifold result;
	ResetCollisionManifold(&result);

	float radius_combined = A.radius + B.radius;
	vec3 distance = B.position - A.position;

	// If the squared distance is less than the squared sum radius, the spheres do not intersect
	if (MagnitudeSq(distance) - radius_combined * radius_combined > 0 || MagnitudeSq(distance) == 0.0f) {
		return result;
	}

	// Get direction vector by normalizing
	vec3 direction = distance;
	Normalize(direction);

	// fill data
	result.colliding = true;
	result.normal = direction;
	result.depth = fabsf(Magnitude(direction) - radius_combined) * 0.5f;
	
	float dist_to_point = A.radius - result.depth; // Distance to intersection point
	Point contact = A.position + direction * dist_to_point;
	result.contacts.push_back(contact);
	return result;
}
CollisionManifold FindCollisionFeatures(const OBB& A, const Sphere& B) {
	CollisionManifold result;
	ResetCollisionManifold(&result);
	// Get closest point of sphere onto OBB
	Point closestPoint = GetClosestPoint(A, B.position);

	// Check for intersection
	float distanceSq = MagnitudeSq(
		closestPoint - B.position);
	if (distanceSq > B.radius* B.radius) {
		return result;
	}

	// If the closest point = center of sphere, we can't easily build a collision normal.
	// try to find a new closest point

	vec3 normal;
	if (CMP(distanceSq, 0.0f)) {
		float mSq = MagnitudeSq(closestPoint - A.position);
		if (CMP(mSq, 0.0f)) return result;
		// Closest point is at the center of the sphere
		normal = AsNormal(closestPoint - A.position);
	}
	else {
		normal = AsNormal(B.position - closestPoint);
	}

	// fill data of manifold
	Point outsidePoint = B.position - normal * B.radius;
	float distance = Magnitude(closestPoint - outsidePoint);
	result.colliding = true;
	result.contacts.push_back(closestPoint + (outsidePoint - closestPoint) * 0.5f);
	result.normal = normal;
	result.depth = distance * 0.5f;
	return result;
}
CollisionManifold FindCollisionFeatures(const OBB& A, const OBB& B)
{
	CollisionManifold result;
	ResetCollisionManifold(&result);

	// Check if there is a posibility to have a collision
	// by creating spheres and checking if they collide
	Sphere s1(A.position, Magnitude(A.size));
	Sphere s2(B.position, Magnitude(B.size));

	if (!SphereSphere(s1, s2)) {
		return result;
	}

	// Store orientations and create axis to test for collision
	const float* o1 = A.orientation.asArray;
	const float* o2 = B.orientation.asArray;

	vec3 test_axes[15] = {
		// first box axes
		vec3(o1[0], o1[1], o1[2]),
		vec3(o1[3], o1[4], o1[5]),
		vec3(o1[6], o1[7], o1[8]),
		// second box axes
		vec3(o2[0], o2[1], o2[2]),
		vec3(o2[3], o2[4], o2[5]),
		vec3(o2[6], o2[7], o2[8])
	};
	for (int i = 0; i < 3; ++i) { // Fill out rest of axis (cross products of all possible pairs)
		test_axes[6 + i * 3 + 0] = Cross(test_axes[i], test_axes[0]);
		test_axes[6 + i * 3 + 1] = Cross(test_axes[i], test_axes[1]);
		test_axes[6 + i * 3 + 2] = Cross(test_axes[i], test_axes[2]);
	}

	// Vars to retrieve data
	vec3* hitNormal = nullptr;
	bool shouldFlip;

	// Test axes
	for (int i = 0; i < 15; ++i) {
		if (test_axes[i].x < 0.000001f) test_axes[i].x = 0.0f;
		if (test_axes[i].y < 0.000001f) test_axes[i].y = 0.0f;
		if (test_axes[i].z < 0.000001f) test_axes[i].z = 0.0f;
		if (MagnitudeSq(test_axes[i]) < 0.001f) {
			continue;
		}

		// Get penetration depth of the OBBs on the separating axis:
		float depth = PenetrationDepth(A, B, test_axes[i], &shouldFlip);
		if (depth <= 0.0f) return result;	// test for intersection 

		if (depth < result.depth) {
			// flip if needed
			if (shouldFlip) test_axes[i] = test_axes[i] * (-1.0f);
			result.depth = depth;
			hitNormal = &test_axes[i];
		}
	}

	// If no collision on any axes
	if (hitNormal == nullptr) return result;

	// else clip boxes and obtain intersection points
	vec3 axis = AsNormal(*hitNormal); // make sure collision normal is a normal

	std::vector<Point> c1 = ClipEdgesToOBB(GetEdges(B), A);
	std::vector<Point> c2 = ClipEdgesToOBB(GetEdges(A), B);
	result.contacts.reserve(c1.size() + c2.size());
	result.contacts.insert(result.contacts.end(), c1.begin(), c1.end());
	result.contacts.insert(result.contacts.end(), c2.begin(), c2.end());

	// project the result of the clipped points onto a shared plane
	Interval i = GetIntervalProjection(A, axis);
	float distance = (i.max - i.min) * 0.5f - result.depth * 0.5f;
	vec3 pointOnPlane = A.position + axis * distance;

	for (int i = result.contacts.size() - 1; i >= 0; --i) {
		vec3 contact = result.contacts[i];
		// Store result of projection
		result.contacts[i] = contact + (axis * Dot(axis, pointOnPlane - contact));

		// Remove not needed contact points (i.e. duplicates)
		for (int j = result.contacts.size() - 1; j > i; --j) {
			if (MagnitudeSq(result.contacts[j] - result.contacts[i]) < 0.0001f) {
				result.contacts.erase(result.contacts.begin() + j);
				break;
			}
		}
	}

	// Return the manifold
	result.colliding = true;
	result.normal = axis;
	return result;
}


