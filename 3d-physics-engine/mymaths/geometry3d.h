#ifndef _H_MYMATHS_GEOMETRY_3D
#define _H_MYMATHS_GEOMETRY_3D

#include "matrices.h"
#include "vectors.h"

typedef vec3 Point;

typedef struct Line {
  Point start;
  Point end;
  inline Line() {}
  inline Line(const Point& s, const Point& e) : start(s), end(e) {}
} Line;

// Line func
float Length(const Line& line);
float LengthSq(const Line& line);

// RAYS
typedef struct Ray {
  Point origin;
  vec3 direction;
  inline Ray() : direction(0.0f, 0.0f, 1.0f) {}
  inline Ray(const Point& o, const vec3& d) : origin(o), direction(d) {
    NormalizeDirection();
  }
  inline void NormalizeDirection() { Normalize(direction); }
} Ray;

// Ray func
Ray FromPoints(const Point& from, const Point& to);

// Spheres
typedef struct Sphere {
  Point position;
  float radius;
  inline Sphere() : radius(1.0f) {}
  inline Sphere(const Point& p, float r) : position(p), radius(r) {}
} Sphere;

// Axis Aligned Bounding Box (AABB)

typedef struct AABB {
  Point origin;
  vec3 size;
  inline AABB() : size(1, 1, 1) {}
  inline AABB(const Point& o, const vec3& s) : origin(o), size(s) {}
} AABB;

vec3 GetMin(const AABB& aabb);
vec3 GetMax(const AABB& aabb);
AABB FromMinMax(const vec3& min, const vec3& max);

// Oriented Bounding Box
typedef struct OBB {
  Point position;
  vec3 size;
  mat3 orientation;
  inline OBB() : size(1, 1, 1) {}
  inline OBB(const Point& p, const vec3& s) : position(p), size(s) {}
  inline OBB(const Point& p, const vec3& s, const mat3& o)
      : position(p), size(s), orientation(o) {}
} OBB;

// Plane
typedef struct Plane {
  vec3 normal;
  float distance;
  inline Plane() : normal(1, 0, 0), distance(0.0f) {}
  inline Plane(const vec3& n) : normal(n), distance(0.0f) {}
  inline Plane(const vec3& n, float d) : normal(n), distance(d) {}
} Plane;

float PlaneEquation(const Point& point, const Plane& plane);

// Triangle
typedef struct Triangle {
  union {
    struct {
      Point a;
      Point b;
      Point c;
    };
    Point points[3];
    float values[9];
  };
  inline Triangle() : a(Point()), b(Point()), c(Point()) {}
  inline Triangle(const Point& p1, const Point& p2, const Point& p3)
      : a(p1), b(p2), c(p3) {}
} Triangle;

// Point Tests
// ----------------------------------------------
// ----------------------------------------------

// Sphere
bool IsPointInSphere(const Point& point, const Sphere& sphere);
Point GetClosestPoint(const Sphere& sphere, const Point& point);

// AABB
bool IsPointInAABB(const Point& point, const AABB& aabb);
Point GetClosestPoint(const AABB& aabb, const Point& point);

// OBB
bool IsPointInOBB(const Point& point, const OBB& obb);
Point GetClosestPoint(const OBB& obb, const Point& point);

// Plane
bool IsPointOnPlane(const Point& point, const Plane& plane);
Point GetClosestPoint(const Plane& plane, const Point& point);

// Line
bool IsPointOnLine(const Point& point, const Line& line);
Point GetClosestPoint(const Line& line, const Point& point);

// Ray
bool IsPointOnRay(const Point& point, const Ray& ray);
Point GetClosestPoint(const Ray& ray, const Point& point);

// Sphere Collision tests
// -----------------------
bool SphereSphere(const Sphere& s1, const Sphere& s2);
bool SphereAABB(const Sphere& sphere, const AABB& aabb);
#define AABBSphere(aabb, sphere) SphereAABB(Sphere, AABB)
bool SphereOBB(const Sphere& sphere, const OBB& obb);
#define OBBSphere(obb, sphere) SphereOBB(sphere, obb)
bool SpherePlane(const Sphere& sphere, const Plane& plane);
#define PlaneSphere(plane, sphere) SpherePlane(sphere, plane)

// Axis Aligned Bounding Box Collision test
// ----------------------------------------
bool AABBAABB(const AABB& aabb1, const AABB& aabb2);

// Intervals to project Bounding boxes onto axis
// ----------------------------------------------
typedef struct Interval {
  float min;
  float max;
} Interval;

Interval GetIntervalProjection(const AABB& rect, const vec3& axis);
Interval GetIntervalProjection(const OBB& rect, const vec3& axis);

bool AreOverlapingOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis);
bool AreOverlapingOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis);

// AABB with OBB Collision test by SAT
// -----------------------------------
bool AABBOBB(const AABB& aabb, const OBB& obb);
#define OBBAABB(obb, aabb) AABBOBB(aabb, obb)

// AABB with Plane Collision test
// -------------------------------
bool AABBPlane(const AABB& aabb, const Plane& plane);
#define PlaneAABB(plane, aabb) AABBPlane(aabb, plane)

// OBB with OBB Collision test by SAT
// -----------------------------------
bool OBBOBB(const OBB& obb1, const OBB& obb2);

// OBB with Plane Collision test
// ------------------------------
bool OBBPlane(const OBB& obb, const Plane& plane);
#define PlaneOBB(plane, obb) OBBPlane(obb, plane)

// PLane with Plane Intersection test
// -----------------------------------
bool DoPlanesIntersect(const Plane& plane1, const Plane& plane2);

#endif  // !_H_MYMATHS_GEOMETRY_3D
