#pragma once
#include "../maths/geometry3d.h"
#include "tiny_obj_loader.h"
bool LoadMesh(const char* szFilePath, Mesh* mesh);
void FreeMesh(Mesh* mesh);
