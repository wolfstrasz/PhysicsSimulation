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
bool DoPlanesIntersect(const Plane& plane1, const Plane& plane2) {
  vec3 d = Cross(plane1.normal, plane2.normal);  // <0,0,0> if parallel
  return !CMP(Dot(d, d), 0.0f);
}
