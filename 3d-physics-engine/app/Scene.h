#ifndef _H_SCENE_
#define _H_SCENE_
#include "../maths/geometry3d.h"
#include <vector>

#define SPLIT_SCENE_CONSTANT 5
// 5 was a proposed optimal constant.

// OcreeNode to split the scene similar to BVH for the model
typedef struct OctreeNode {
	AABB bounds;
	OctreeNode* children = nullptr;
	std::vector<Model*> models;
	inline OctreeNode() : children(nullptr) { }
	inline ~OctreeNode() {
		if (children != nullptr) {
			delete[] children;
		}
	}

} OctreeNode;

class Scene {
protected:
	std::vector<Model*> objects;
	OctreeNode* octree;

private:
	Scene(const Scene&);
	Scene& operator=(const Scene&);

public: 
	inline Scene() : octree(nullptr) { }
	inline ~Scene() { // Octree is optional
		if (octree != nullptr) {
			delete octree;
		}
	}

	void AddModel(Model* model);
	void RemoveModel(Model* model);
	void UpdateModel(Model* model);
	// Create an OCT-tree scene
	bool Accelerate(const vec3& position, float size);

public:
	std::vector<Model*>FindChildren(const Model* model);
	// Raycast to get the closes model in the scene to the origin of the ray
	// This way we handle broad-phase collision
	Model* Raycast(const Ray& ray);

	// Query returns set of objects occupying a region specified by a sphere or AABB
	std::vector<Model*> Query(const Sphere& sphere);
	std::vector<Model*> Query(const AABB& aabb);


	/* 
		If a scene is spatially divided with an Octree, this method of culling should increase render
		time by eliminating non visible objects from being rendered. Otherwise we just linearly test 
		all objects against the frustum and make performance worse.
	*/
	std::vector<Model*>Cull(const Frustum& f);
	
};



void SplitTree(OctreeNode* node, int depth);

// Model handling support functions for the tree
void Insert(OctreeNode* node, Model* model);
void Remove(OctreeNode* node, Model* model);
void Update(OctreeNode* node, Model* model);

// Add raycast and Query support to OCT-tree
Model* FindClosest(const std::vector<Model*>& set,const Ray& ray);
Model* Raycast(OctreeNode* node, const Ray& ray); 
std::vector<Model*> Query(OctreeNode* node, const Sphere& sphere);
std::vector<Model*> Query(OctreeNode* node, const AABB& aabb);

#endif // _H_SCENE