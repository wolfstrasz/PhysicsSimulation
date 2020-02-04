#include "QuadTree.h"

#include <queue>

// init static vars
int QuadTreeNode::maxDepth = 5;
unsigned int QuadTreeNode::maxObjectsPerNode = 15;

//
bool QuadTreeNode::IsLeaf() { return children.size() == 0; }

// Get number of objects in node without recursion
int QuadTreeNode::NumObjects() {
  Reset();
  int objectCount = contents.size();
  for (int i = 0, size = contents.size(); i < size; ++i) {
    contents[i]->flag = true;
  }
  // Make a queue of nodes to be processed and push the initial node into this
  // queue
  std::queue<QuadTreeNode*> process;
  process.push(this);

  // Loop through the process queue
  while (process.size() > 0) {
    QuadTreeNode* processing = process.back();

    // If the node we are looking at is not a leaf, add its children to the
    // process list
    if (!processing->IsLeaf()) {
      for (int i = 0, size = processing->children.size(); i < size; ++i) {
        process.push(&processing->children[i]);
      }

      // Otherwise, count the child object
      // If object is counted, flip flag to signify that it has been counted
    } else {
      for (int i = 0, size = processing->contents.size(); i < size; ++i) {
        if (!processing->contents[i]->flag) {
          objectCount += 1;
          processing->contents[i]->flag = true;
        }
      }
    }
    process.pop();
  }
  Reset();
  return objectCount;
}

// Insert an object
void QuadTreeNode::Insert(QuadTreeData& data) {
  if (!RectangleRectangle(data.bounds, nodeBounds)) {
    return;  // The object does not fit into this node
  }
  // If the node is a leaf and can be split further
  if (IsLeaf() && (contents.size() + 1 > maxObjectsPerNode)) {
    Split();  // Try splitting!
  }
  if (IsLeaf()) {
    contents.push_back(&data);
  } else {
    // If the node is not a leaf, try to insert the content into all children of
    // the node
    for (int i = 0, size = children.size(); i < size; ++i) {
      children[i].Insert(data);
    }
  }
}

// Remove
void QuadTreeNode::Remove(QuadTreeData& data) {
  if (IsLeaf()) {
    int removeIndex = -1;
    // If we are dealing with a leaf node, look for an object to remove
    for (int i = 0, size = contents.size(); i < size; ++i) {
      if (contents[i]->object == data.object) {
        removeIndex = i;
        break;
      }
    }
    // If an object to be removed is found, actually remove it
    if (removeIndex != -1) {
      contents.erase(contents.begin() + 1);
    }
    // If the node is not a leaf, call the Remove function recursively
  } else {
    for (int i = 0, size = children.size(); i < size; ++i) {
      children[i].Remove(data);
    }
  }
  Shake();
}

// Update
void QuadTreeNode::Update(QuadTreeData& data) {
  Remove(data);
  Insert(data);
}

// Reset
void QuadTreeNode::Reset() {
  if (IsLeaf()) {
    for (int i = 0, size = contents.size(); i < size; ++i) {
      contents[i]->flag = false;
    }
  } else {
    for (int i = 0, size = children.size(); i < size; ++i) {
      children[i].Reset();
    }
  }
}

// Shape
void QuadTreeNode::Shake() {
  if (!IsLeaf()) {
    unsigned int numObjects = NumObjects();
    if (numObjects == 0) {
      children.clear();
    }
    // If this node contains less than the maximum number of objects, we can
    // collapse all of the child nodes into this node

    else if (numObjects < maxObjectsPerNode) {
      std::queue<QuadTreeNode*> process;
      process.push(this);
      while (process.size() > 0) {
        QuadTreeNode* processing = process.back();
        if (!processing->IsLeaf()) {
          for (int i = 0, size = processing->children.size(); i < size; ++i) {
            process.push(&processing->children[i]);
          }
        } else {
          contents.insert(contents.end(), processing->contents.begin(),
                          processing->contents.end());
        }
        process.pop();
      }
      children.clear();
    }
  }
}

// SPLIT
void QuadTreeNode::Split() {
  if (currentDepth + 1 >= maxDepth) {
    return;
  }
  vec2 min = GetMin(nodeBounds);
  vec2 max = GetMax(nodeBounds);
  vec2 center = min + ((max - min) * 0.5f);

  // Use the min, max, and center variables to divide the node being processed
  // into four smaller nodes
  Rectangle2D childAreas[] = {
      Rectangle2D(FromMinMax(vec2(min.x, min.y), vec2(center.x, center.y))),
      Rectangle2D(FromMinMax(vec2(center.x, min.y), vec2(max.x, center.y))),
      Rectangle2D(FromMinMax(vec2(center.x, center.y), vec2(max.x, max.y))),
      Rectangle2D(FromMinMax(vec2(min.x, center.y), vec2(center.x, max.y))),
  };
  // Distribute the objects held in this node into its children
  for (int i = 0; i < 4; ++i) {
    children.push_back(QuadTreeNode(childAreas[i]));
    children[i].currentDepth = currentDepth + 1;
  }
  for (int i = 0, size = contents.size(); i < size; ++i) {
    children[i].Insert(*contents[i]);
  }
  contents.clear();
}

// Query

std::vector<QuadTreeData*> QuadTreeNode::Query(const Rectangle2D& area) {
  std::vector<QuadTreeData*> result;
  if (!RectangleRectangle(area, nodeBounds)) {
    return result;
  }
  // If we are looking at a leaf node, query the elements within this node
  if (IsLeaf()) {
    for (int i = 0, size = contents.size(); i < size; ++i) {
      if (RectangleRectangle(contents[i]->bounds, area)) {
        result.push_back(contents[i]);
      }
    }
  }
  // If the node we are searching is not a leaf node, recursively query all
  // child nodes

  else {
    for (int i = 0, size = children.size(); i < size; ++i) {
      vector<QuadTreeData*> recurse = children[i].Query(area);
      if (recurse.size() > 0) {
        result.insert(result.end(), recurse.begin(), recurse.end());
      }
    }
  }
  return result;
}
