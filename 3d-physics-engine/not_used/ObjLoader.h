#pragma once
#include "../maths/geometry3d.h"

bool LoadMesh(const char* szFilePath, Mesh* mesh);
void FreeMesh(Mesh* mesh);
