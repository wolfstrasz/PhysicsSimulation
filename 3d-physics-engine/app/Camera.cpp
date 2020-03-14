#include "Camera.h"

#include <math.h>
#include <cfloat>


Camera::Camera() {
	m_Fov = 60.0f;
	m_Aspect = 1.3f;
	m_Near = 0.01f;
	m_Far = 1000.0f;
	m_Width = 1.0;
	m_Height = 1.0f;
	m_World = mat4();
	m_Proj = Projection(m_Fov, m_Aspect, m_Near, m_Far);
	m_ProjectionMode = 0;
}

mat4 Camera::GetWorldMatrix() {
	return m_World;
}

mat4 Camera::GetViewMatrix() {
	if (!IsOrthoNormal()) {
		OrthoNormalize();
	}

	// When the matrix is orthonormal we can use
	//  M * (M^T) = I 
	// To build the inverse we need to find the transposed matrix

	// Build inverse matrix (i.e view matrix)
	mat4 inverse = Transpose(m_World);
	inverse._41 = inverse._14 = 0.0f;
	inverse._42 = inverse._24 = 0.0f;
	inverse._43 = inverse._34 = 0.0f;

	vec3 right = vec3(m_World._11, m_World._12, m_World._13);
	vec3 up = vec3(m_World._21, m_World._22, m_World._23);
	vec3 forward = vec3(m_World._31, m_World._32, m_World._33);
	vec3 position = vec3(m_World._41, m_World._42, m_World._43);

	inverse._41 = -Dot(right, position);
	inverse._42 = -Dot(up, position);
	inverse._43 = -Dot(forward, position);

	return inverse;
}

mat4 Camera::GetProjectionMatrix() {
	return m_Proj;
}

float Camera::GetAspect() {
	return m_Aspect;
}

bool Camera::IsOrthographic() {
	return m_ProjectionMode == 1;
}

bool Camera::IsPerspective() {
	return m_ProjectionMode == 0;
}

bool Camera::IsOrthoNormal() {

	vec3 right = vec3(m_World._11, m_World._12, m_World._13);
	vec3 up = vec3(m_World._21, m_World._22, m_World._23);
	vec3 forward = vec3(m_World._31, m_World._32, m_World._33);

	// Check if axes are normal
	if (!CMP(Dot(right, right), 1.0f) ||
		!CMP(Dot(up, up), 1.0f) ||
		!CMP(Dot(forward, forward), 1.0f)) {
		return false; // Axis are not normal length
	}

	// Check if axes are orthogonal
	if (!CMP(Dot(forward, up), 0.0f) ||
		!CMP(Dot(forward, right), 0.0f) ||
		!CMP(Dot(right, up), 0.0f)) {
		return false; // Axis are not perpendicular
	}
	return true;
}

void Camera::OrthoNormalize() {
	vec3 right = vec3(m_World._11, m_World._12, m_World._13);
	vec3 up = vec3(m_World._21, m_World._22, m_World._23);
	vec3 forward = vec3(m_World._31, m_World._32, m_World._33);


	vec3 f = AsNormal(forward);
	vec3 r = AsNormal(Cross(up, f));
	vec3 u = Cross(f, r);

	m_World = mat4(
		r.x, r.y, r.z, 0.0f,
		u.x, u.y, u.z, 0.0f,
		f.x, f.y, f.z, 0.0f,
		m_World._41, m_World._42, m_World._43, 1.0f
	);
}

void Camera::Resize(int width, int height) {
	m_Aspect = (float)width / (float)height;

	// If perspective camera -> build perspective proj matrix
	if (m_ProjectionMode == 0) // Perspective
		m_Proj = Projection(m_Fov, m_Aspect, m_Near, m_Far);

	else if (m_ProjectionMode == 1) { // Ortho
		m_Width = (float)width;
		m_Height = (float)height;
		float halfW = m_Width * 0.5f;
		float halfH = m_Height * 0.5f;
		m_Proj = Ortho(-halfW, halfW, halfH, -halfH, m_Near, m_Far);
	}

}

void Camera::Perspective(float fov, float aspect, float zNear, float zFar) {
	m_Fov = fov;
	m_Aspect = aspect;
	m_Near = zNear;
	m_Far = zFar;
	m_Proj = Projection(fov, aspect, zNear, zFar);
	m_ProjectionMode = 0;
}

void Camera::Orthographic(float width, float height, float zNear, float zFar) {
	m_Width = width;
	m_Height = height;
	m_Near = zNear;
	m_Far = zFar;
	float halfW = width * 0.5f;
	float halfH = height * 0.5f;
	m_Proj = Ortho(-halfW, halfW, halfH, -halfH, zNear, zFar);

	m_ProjectionMode = 1;
}

Frustum Camera::GetFrustum()
{
	Frustum result;
	// View projection matrix (VP)
	mat4 vp = GetViewMatrix() * GetProjectionMatrix();
	
	vec3 col1(vp._11, vp._21, vp._31);//, vp._41
	vec3 col2(vp._12, vp._22, vp._32);//, vp._42
	vec3 col3(vp._13, vp._23, vp._33);//, vp._43
	vec3 col4(vp._14, vp._24, vp._34);//, vp._44

	// calculate the direction vector for every plane
	result.left.normal = col4 + col1;
	result.right.normal = col4 - col1;
	result.bottom.normal = col4 + col2;
	result.top.normal = col4 - col2;

	// Direct3D style for near: Range of space x E (-1,1), y E (-1,1), z E(0,1) 
	result._near.normal = col3;
	// OpenGL style for near: Range of space x E (-1,1), y E (-1,1), z E(-1,1) 
	// result.near.normal = col3 + col4; 
	result._far.normal = col4 - col3;

	// calculate the distance from origin for each plane
	result.left.distance = vp._44 + vp._41;
	result.right.distance = vp._44 - vp._41;
	result.bottom.distance = vp._44 + vp._42;
	result.top.distance = vp._44 - vp._42;
	result._near.distance = vp._43;
	result._far.distance = vp._44 - vp._43;

	// normalize planes and return frustum object
	for (int i = 0; i < 6; ++i) {
		float mag = 1.0f / Magnitude(result.planes[i].normal);
		result.planes[i].normal = result.planes[i].normal * mag;
		result.planes[i].distance *= mag;
	}
	return result;
}


void Camera::SetProjection(const mat4& projection) {
	m_Proj = projection;
	m_ProjectionMode = 2;
}

void Camera::SetWorld(const mat4& view) {
	m_World = view;
}