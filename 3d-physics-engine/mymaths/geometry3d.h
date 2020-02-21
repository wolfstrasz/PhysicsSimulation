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
bool PlanePlane(const Plane& plane1, const Plane& plane2);

// Raycasting - care if it intersects and how much time it takes
// -----------------------------------
float Raycast(const Sphere& sphere, const Ray& ray);
float Raycast(const AABB& aabb, const Ray& ray);
float Raycast(const OBB& obb, const Ray& ray);
float Raycast(const Plane& plane, const Ray& ray);
vec3 Barycentric(const Point& p, const Triangle& t);
float Raycast(const Triangle& triangle, const Ray& ray);

// Linetest - only care if it intersects (bool)
// -----------------------------------
bool Linetest(const Sphere& sphere, const Line& line);
bool Linetest(const AABB& aabb, const Line& line);
bool Linetest(const OBB& obb, const Line& line);
bool Linetest(const Plane& plane, const Line& line);
bool Linetest(const Triangle& triangle, const Line& line);

// Collision cases for Triangle
// -----------------------------------
// Helper functions for tringle
bool PointInTriangle(const Point& p, const Triangle& t);
Plane FromTriangle(const Triangle& t);
Point GetClosestPoint(const Triangle& t, const Point& p);

// Triangle with Sphere Collision test
bool TriangleSphere(const Triangle& t, const Sphere& s);
#define SphereTriangle(s, t) TriangleSphere(t, s)

// Triangle with AABB Collision test
Interval GetIntervalProjection(const Triangle& triangle, vec3& axis);
bool AreOverlapingOnAxis(const AABB& aabb, const Triangle& triangle, const vec3& axis);
bool TriangleAABB(const Triangle& t, const AABB& a);
#define AABBTriangle(a, t) TriangleAABB(t, a)

// Triangle with OBB Collision test
bool AreOverlapingOnAxis(const OBB& obb, const Triangle& triangle, const vec3& axis);
bool TriangleOBB(const Triangle& t, const OBB& o);
#define OBBTriangle(o, t) TriangleOBB(t, o)

// Triangle with Plane Collision test
bool TrianglePlane(const Triangle& t, const Plane& p);
#define PlaneTriangle(p, t) TrianglePlane(t, p)

// Triangle with Triangle Collision test
bool AreOverlapingOnAxis(const Triangle& t1, const Triangle& t2, const vec3& axis);
bool TriangleTriangle(const Triangle& t1, const Triangle& t2);

// SAT cross edge for normals going to 0
vec3 SatCrossEdge(const vec3& t1_p1, const vec3& t1_p2, const vec3& t2_p1, const vec3& t2_p2);
// Robust collision test
// TODO: for other collision tests
bool TriangleTriangleRobust(const Triangle& t1,const Triangle& t2);



// MESHES
// ------------------------------------------
typedef struct Mesh {
	int numTriangles;
	union {
		Triangle* triangles;//size = numTriangles
		Point* vertices; //size = numTriangles * 3
		float* values; //size = numTriangles * 3 * 3
	};
	BVHNode* accelerator;
	Mesh() : numTriangles(0), values(nullptr), accelerator(nullptr) {}

} Mesh;

// Bounding Volume Hierarchy
typedef struct BVHNode {
	AABB bounds;
	BVHNode* children;
	int numTriangles;
	int* triangles;
	BVHNode() : children(nullptr), numTriangles(0), triangles(nullptr) {}
} BVHNode;

void AccelerateMesh(Mesh& mesh);
void SplitBVHNode(BVHNode* node, const Mesh& model,
	int depth);
void FreeBVHNode(BVHNode* node);

// Intersection tests
bool Linetest(const Mesh& mesh, const Line& line);
bool MeshSphere(const Mesh& mesh, const Sphere& sphere);
bool MeshAABB(const Mesh& mesh, const AABB& aabb); 
bool MeshOBB(const Mesh& mesh, const OBB& obb);
bool MeshPlane(const Mesh& mesh, const Plane& plane);
bool MeshTriangle(const Mesh& mesh, const Triangle& triangle); 
float MeshRay(const Mesh& mesh, const Ray& ray);


// Model class: contains Mesh, translation and rotation (in full engine scale)

class Model {
protected:
	Mesh* content;
	AABB bounds;
public:
	vec3 position;
	vec3 rotation;
	Model* parent;

	inline Model() : parent(0), content(0) { }
	inline Mesh* GetMesh() const {
		return content;
	}
	inline AABB GetBounds() const {
		return bounds;
	}
	void SetContent(Mesh* mesh);
	// TODO: add list of children for easier search
};

mat4 GetWorldMatrix(const Model& model);
OBB GetOBB(const Model& model);

// Intersection tests for model
float ModelRay(const Model& model, const Ray& ray);
bool Linetest(const Model& model, const Line& line);
bool ModelSphere(const Model& model, const Sphere& sphere);
bool ModelAABB(const Model& model, const AABB& aabb);
bool ModelOBB(const Model& model, const OBB& obb);
bool ModelPlane(const Model& model, const Plane& plane);
bool ModelTriangle(const Model& model, const Triangle& triangle);

// FRUSTUM OBJ
// --------------------------------
typedef struct Frustum {
	union {
		struct {
			Plane top;
			Plane bottom;
			Plane left;
			Plane right;
			Plane near;
			Plane far;
		};
		Plane planes[6];
	};
	inline Frustum() { }
} Frustum;

// Frustum intersection functions

Point Intersection(Plane p1, Plane p2, Plane p3);
void GetCorners(const Frustum& f, vec3* outCorners);

bool Intersects(const Frustum& f, const Point& p);
bool Intersects(const Frustum& f, const Sphere& s);

/*
Classifying functions for OBB/AABB agains a plane
If the box is behind the plane, the negative distance is returned
If the box is in front of the plane, the positive distance is returned
If the box intersects the plane, zero is returned
*/

float Classify(const AABB& aabb, const Plane& plane);
float Classify(const OBB& obb, const Plane& plane);

bool Intersects(const Frustum& f, const AABB& aabb);
bool Intersects(const Frustum& f, const OBB& obb);

#endif  // !_H_MYMATHS_GEOMETRY_3D
