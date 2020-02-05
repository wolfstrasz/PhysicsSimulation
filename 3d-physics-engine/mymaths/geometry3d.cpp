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

Point ClosestPoint(const Ray& ray, const Point& point) {
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
