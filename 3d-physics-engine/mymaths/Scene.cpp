#include "Scene.h"
#include <algorithm>
#include <stack>
#include <list>

void Scene::AddModel(Model* model) {

	if (std::find(objects.begin(), objects.end(), model) != objects.end()) {
		// Duplicate object, don't add
		return;
	}
	objects.push_back(model);

}

void Scene::RemoveModel(Model* model) {
	objects.erase(std::remove(objects.begin(), objects.end(), model), objects.end());
}

void Scene::UpdateModel(Model* model) {
	// Placeholder
}

bool Scene::Accelerate(const vec3& position, float size)
{
	// If already an OCT-tree => do nothing
	if (octree != nullptr) {
		return false;
	}

	// Min-max for bounding box
	vec3 min(position.x - size,
		position.y - size,
		position.z - size);
	vec3 max(position.x + size,
		position.y + size,
		position.z + size);

	// Create root node and add all models
	octree = new OctreeNode();
	octree->bounds = FromMinMax(min, max);
	octree->children = nullptr;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		octree->models.push_back(objects[i]);
	}

	SplitTree(octree, SPLIT_SCENE_CONSTANT);
	return true;
}

std::vector<Model*> Scene::FindChildren(const Model* model)
{
	std::vector<Model*> result;

	// go through all objects to find children of model
	for (int i = 0, size = objects.size(); i < size; ++i) {

		// avoid nullptr and ignore root model of search
		if (objects[i] == nullptr || objects[i] == model) {
			continue;
		}

		// If any object above the current model is the argument to this function,
		// that model is a child of the argument
		Model* test_model_iterator = objects[i]->parent;
		if (test_model_iterator != nullptr) {
			if (test_model_iterator == model) {
				result.push_back(objects[i]);
				continue;
			}
			test_model_iterator = test_model_iterator->parent;
		}
	}
	return result;
}

Model* Scene::Raycast(const Ray& ray)
{
	// Check if and octree exists
	if (octree != 0) {
		return ::Raycast(octree, ray);
	}
	Model* result = nullptr;
	float result_distance = -1;

	// Find closest object 
	for (int i = 0, size = objects.size(); i < size; ++i) {
		float distance = ModelRay(*objects[i], ray);
		if (result == nullptr && distance >= 0) {
			result = objects[i];
			result_distance = distance;
		}
		else if (result != nullptr && distance < result_distance) {
			result = objects[i];
			result_distance = distance;
		}
	}
	return result;
}

std::vector<Model*> Scene::Query(const Sphere& sphere)
{
	// Check if and octree exists
	if (octree != 0) {
		return ::Query(octree, sphere);
	}
	std::vector<Model*> result;

	// Go through obejects in scene and find intersections with the sphere
	// A model counts as one even if it is split into a tree of smaller objects
	for (int i = 0, size = objects.size(); i < size; ++i) {
		OBB bounds = GetOBB(*objects[i]);
		if (SphereOBB(sphere, bounds)) {
			result.push_back(objects[i]);
		}
	}
	return result;
}

std::vector<Model*> Scene::Query(const AABB& aabb)
{
	// Check if and octree exists
	if (octree != 0) {
		return ::Query(octree, aabb);
	}

	std::vector<Model*> result;

	// Go through obejects in scene and find intersections with the AABB
	// A model counts as one even if it is split into a tree of smaller objects
	for (int i = 0, size = objects.size(); i < size; ++i) {
		OBB bounds = GetOBB(*objects[i]);
		if (AABBOBB(aabb, bounds)) {
			result.push_back(objects[i]);
		}
	}
	return result;
}

std::vector<Model*> Scene::Cull(const Frustum& f)
{
	std::vector<Model*> result;

	if (octree == 0) {
		for (int i = 0; i < objects.size(); ++i) {
			OBB bounds = GetOBB(*(objects[i]));
			if (Intersects(f, bounds))
				result.push_back(objects[i]);
		}
	}

	else {
		std::list<OctreeNode*> nodes;
		nodes.push_back(octree);

		// Loop depth first through scene
		while (nodes.size() > 0) {
			OctreeNode* current_node = *nodes.begin();
			nodes.pop_front();

			// Check if not a leaf -> go through children of node
			if (current_node->children != nullptr) {
				for (int i = 0; i < 8; ++i) {
					// if bounds of children intersect the frustum consider for culling
					AABB bounds = current_node->children[i].bounds;
					if (Intersects(f, bounds))
						nodes.push_back(&current_node->children[i]);
				}
			}
			// If a leaf go through models in node
			else {
				for (int i = 0; i < current_node->models.size(); ++i) {
					OBB bounds = GetOBB(*(current_node->models[i]));
					if (Intersects(f, bounds))
						result.push_back(current_node->models[i]);
				}

			}
		}
	}
	return result;
}

void SplitTree(OctreeNode* node, int depth)
{
	if (depth-- <= 0) { // Decrements depth
		return;
	}
	
	// Split every node that doesnt have children
	if (node->children == nullptr) {
		node->children = new OctreeNode[8];

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

	// If node contains models and has children, send models to children
	if (node->children != 0 && node->models.size() > 0) {
		for (int i = 0; i < 8; ++i) { // For each child
			for (int j = 0, size = node->models.size();
				j < size; ++j) {

				// Add models which OBB intersects the child node OBB
				OBB bounds = GetOBB(*node->models[j]);
				if (AABBOBB(node->children[i].bounds,
					bounds)) {
					node->children[i].models.push_back(
						node->models[j]
					);
				}
			}
		}
		// remove models from split node
		node->models.clear();
		
		// Go recursively and apply splitting to child nodes
		for (int i = 0; i < 8; ++i) { // Recurse
			SplitTree(&(node->children[i]), depth);
		}
	}
}

void Insert(OctreeNode* node, Model* model)
{
	// Get model bounds
	OBB bounds = GetOBB(*model);
	// Check if model occupies the node space
	if (AABBOBB(node->bounds, bounds)) {
		// only insert in leaf nodes
		if (node->children == nullptr) {
			node->models.push_back(model);
		}
		else { // if not leaf node go to children
			for (int i = 0; i < 8; ++i) {
				Insert(&(node->children[i]), model);
			}
		}
	}
}

void Remove(OctreeNode* node, Model* model)
{
	// Remove a model if it occupies a leaf node
	if (node->children == nullptr) {
		std::vector<Model*>::iterator it =
			std::find(node->models.begin(),
				node->models.end(), model
			);
		if (it != node->models.end()) {
			node->models.erase(it);
		}
	}
	else { // if not a leaf node go to children
		for (int i = 0; i < 8; ++i) {
			Remove(&(node->children[i]), model);
		}
	}

}

void Update(OctreeNode* node, Model* model)
{
	// Note: if we have lots of dynamic objects in scene its
	//		 better to use LIST and not VECTOR
	//		 Increases speed of Remove/Insert but decreases speed of iteration
	// To update first remove it from its current space
	Remove(node, model);
	// Then add it again to the right one
	Insert(node, model);
}

Model* FindClosest(const std::vector<Model*>& set, const Ray& ray)
{
	if (set.size() == 0) {
		return 0;
	}

	// Variables to store closes model
	Model* closest = nullptr;
	float closest_t = -1.0f;

	// Raycast over every model
	for (int i = 0, size = set.size(); i < size; ++i) {
		float this_t = ModelRay(*set[i], ray);

		// If Raycast didnt hit => skip it
		if (this_t < 0) {
			continue;
		}

		// Store closest hit
		if (closest_t < 0 || this_t < closest_t) {
			closest_t = this_t;
			closest = set[i];
		}
	}
	return closest;
}

Model* Raycast(OctreeNode* node, const Ray& ray)
{
	float t = Raycast(node->bounds, ray);
	
	// Check for a raycast hit to the node
	if (t >= 0) {
		// return lagest object if node is a leaf
		if (node->children == 0) {
			return FindClosest(node->models, ray);
		}
		else { // raycast on children recursively
			std::vector<Model*> results;
			for (int i = 0; i < 8; ++i) {
				Model* result = Raycast(&(node->children[i]), ray);
				if (result != 0) results.push_back(result);
			}

			// return closest model hit
			return FindClosest(results, ray);
		}
	}
	// if we did not find any model
	return nullptr;
}

std::vector<Model*> Query(OctreeNode* node, const Sphere& sphere)
{
	std::vector<Model*> result;
	// Check if sphere intersects the node bounds
	if (SphereAABB(sphere, node->bounds)) {
		if (node->children == nullptr) { // If a leaf node

			for (int i = 0, size = node->models.size(); i < size; ++i) {
				OBB bounds = GetOBB(*(node->models[i]));
				// Add models which OBB intersects with the sphere
				if (SphereOBB(sphere, bounds)) {
					result.push_back(node->models[i]);
				}
			}
		}
		else { // not a leaf node -> recursively go to children

			for (int i = 0; i < 8; ++i) {
				std::vector<Model*> child = Query(&(node->children[i]), sphere);
				if (child.size() > 0) {
					result.insert(result.end(), child.begin(), child.end());
				}
			}
		}
	}
	return result;
}

std::vector<Model*> Query(OctreeNode* node, const AABB& aabb)
{
	std::vector<Model*> result;

	// Check if sphere intersects the node bounds
	if (AABBAABB(aabb, node->bounds)) {
		if (node->children == nullptr) { // If a leaf node
			for (int i = 0, size = node->models.size(); i < size; ++i) {

				OBB bounds = GetOBB(*(node->models[i]));
				// Add models which OBB intersects with the AABB
				if (AABBOBB(aabb, bounds)) {
					result.push_back(node->models[i]);
				}
			}
		}
		else { // not a leaf node -> recursively go to children
			for (int i = 0; i < 8; ++i) { 
				std::vector<Model*> child = Query(&(node->children[i]), aabb);
				if (child.size() > 0) {
					result.insert(result.end(),child.begin(), child.end());
				}
			}
		}
	}
	return result;
}
