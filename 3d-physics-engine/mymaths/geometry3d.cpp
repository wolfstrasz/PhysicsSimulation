#include "geometry3d.h"

#include <cfloat>
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
float Raycast(const Sphere& sphere, const Ray& ray)
{
	vec3 e = sphere.position - ray.origin;

	// Store squared radiuses and maginitudes
	float rSq = sphere.radius * sphere.radius;
	float eSq = MagnitudeSq(e);

	// ray.direction is assumed to be normalized
	float a = Dot(e, ray.direction);

	// Construct sides of a triangle (work with squared units)
	float bSq = eSq - (a * a);
	float f = sqrt(rSq - bSq);

	// No collision has happened
	if (rSq - (eSq - (a * a)) < 0.0f) {
		return -1; // -1 is invalid.
	}
	// Ray starts inside the sphere
	else if (eSq < rSq) {
		return a + f; // Just reverse direction
	}
	// else Normal intersection
	return a - f;
}

float Raycast(const AABB& aabb, const Ray& ray)
{
	vec3 min = GetMin(aabb);
	vec3 max = GetMax(aabb);

	// Find the both intersections of the ray against each of the three slabs which make up a bounding box
	float t1 = (min.x - ray.origin.x) / ray.direction.x;
	float t2 = (max.x - ray.origin.x) / ray.direction.x;
	float t3 = (min.y - ray.origin.y) / ray.direction.y;
	float t4 = (max.y - ray.origin.y) / ray.direction.y;
	float t5 = (min.z - ray.origin.z) / ray.direction.z;
	float t6 = (max.z - ray.origin.z) / ray.direction.z;

	// To find the point of entry, we need to find the largest minimum value.
	// To find the point of exit, we need to find the smallest minimum value.

	// Find largest min value
	float tmin = fmaxf(
		fmaxf(
			fminf(t1, t2),
			fminf(t3, t4)
		),
		fminf(t5, t6)
	);

	// Find smallex max val
	float tmax = fminf(
		fminf(
			fmaxf(t1, t2),
			fmaxf(t3, t4)
		),
		fmaxf(t5, t6)
	);

	// Check if intersects from behind
	if (tmax < 0) {
		return -1;
	}

	// Check for general intersection
	if (tmin > tmax) {
		return -1;
	}

	// Check if ray origin is inside of the box
	// tmin < zero => yes and tmax is valid collision point
	if (tmin < 0.0f) {
		return tmax;
	}
	return tmin;
}

float Raycast(const OBB& obb, const Ray& ray)
{
	const float* o = obb.orientation.asArray;
	const float* size = obb.size.asArray;
	// X, Y and Z axis of OBB
	vec3 X(o[0], o[1], o[2]);
	vec3 Y(o[3], o[4], o[5]);
	vec3 Z(o[6], o[7], o[8]);

	// Find a vector pointing from origin of ray to OBB
	vec3 p = obb.position - ray.origin;

	// Project the direction onto each of the axis
	vec3 f(
		Dot(X, ray.direction),
		Dot(Y, ray.direction),
		Dot(Z, ray.direction)
	);

	// Project vector P onto each axis of rotation
	vec3 e(
		Dot(X, p),
		Dot(Y, p),
		Dot(Z, p)
	);

	// Calcualte all max and mins line in AABB
	float t[6] = { 0, 0, 0, 0, 0, 0 };
	for (int i = 0; i < 3; ++i) {
		if (CMP(f[i], 0)) {
			// If the ray is parallel to the slab being tested, and the origin of the ray is not inside the
			// slab we have no hit
			if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0) {
				return -1;
			}
			f[i] = 0.00001f; // Avoid div by 0!
		}
		t[i * 2 + 0] = (e[i] + size[i]) / f[i]; // min
		t[i * 2 + 1] = (e[i] - size[i]) / f[i]; // max
	}

	// Find min and max values
	float tmin = fmaxf(
		fmaxf(
			fminf(t[0], t[1]),
			fminf(t[2], t[3])),
		fminf(t[4], t[5])
	);
	float tmax = fminf(
		fminf(
			fmaxf(t[0], t[1]),
			fmaxf(t[2], t[3])),
		fmaxf(t[4], t[5])
	);

	// tmax < 0 => ray origin from behind (doesn't count)
	if (tmax < 0) {
		return -1.0f;
	}

	// tmin > tmax => ray does not intersect
	if (tmin > tmax) {
		return -1.0f;
	}

	// tmin < 0 => origin of ray is inside OBB
	if (tmin < 0.0f) {
		return tmax;
	}

	// Else return closes point
	return tmin;
}

float Raycast(const Plane& plane, const Ray& ray)
{
	float nd = Dot(ray.direction, plane.normal);
	float pn = Dot(ray.origin, plane.normal);

	// nd has to be < 0 for ray to intersect
	// else normal of plane and ray point in same direction
	if (nd >= 0.0f) {
		return -1;
	}

	// obtain time
	float t = (plane.distance - pn) / nd;

	// check if ray is hit plane from behind (i.e no hit)
	if (t >= 0.0f) {
		return t;
	}
	return -1;
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

float Raycast(const Triangle& triangle, const Ray& ray)
{
	// Create a plane and raycast to plane if no hit => no hit on triangle
	// Afterwards check for barycentric coords beacause
	// barycentric gives if a point is inside the "volume" of a triangle
	Plane plane = FromTriangle(triangle);
	float t = Raycast(plane, ray);
	if (t < 0.0f) {
		return t;
	}

	// Find point on plane that was hit
	Point result = ray.origin + ray.direction * t;

	// Find barycentric coords of the point on the triangle
	vec3 barycentric = Barycentric(result, triangle);

	// If point is within triangle ray hit it
	if (barycentric.x >= 0.0f && barycentric.x <= 1.0f &&
		barycentric.y >= 0.0f && barycentric.y <= 1.0f &&
		barycentric.z >= 0.0f && barycentric.z <= 1.0f) {
		return t;
	}

	return -1.0f;
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
	float t = Raycast(triangle, ray);

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
		if (!AreOverlapingOnAxis(a, t, test[i])) {
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
