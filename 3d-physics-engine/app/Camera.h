#pragma once
#include "../maths/matrices.h"
#include "../maths/geometry3d.h"

class Camera {
protected:
	float m_Fov;
	float m_Aspect;
	float m_Near;
	float m_Far;
	float m_Width;
	float m_Height;

	mat4 m_World;
	mat4 m_Proj;

	int m_ProjectionMode; // ^ 0 - Perspective, 1 - Ortho, 2 - Use

public:
	Camera();
	inline virtual ~Camera() { }

	mat4 GetViewMatrix(); // Inverse of world!
	mat4 GetWorldMatrix();
	mat4 GetProjectionMatrix();

	void SetWorld(const mat4& view);
	void SetProjection(const mat4& projection);

	// Helper functions
	float GetAspect();

	bool IsOrthographic();
	bool IsPerspective();

	bool IsOrthoNormal();
	void OrthoNormalize();

	void Resize(int width, int height);

	// user defined cameras
	void Perspective(float fov, float aspect, float zNear, float zFar);
	void Orthographic(float width, float height, float zNear, float zFar);

	// Frustum helper functions
	Frustum GetFrustum();

};