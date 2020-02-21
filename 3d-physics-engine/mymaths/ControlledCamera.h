#pragma once
#include "Camera.h"


class OrbitCamera : public Camera {
protected:

	vec3 target;
	vec2 panSpeed;

	// zoom variables
	float zoomDistance;
	vec2 zoomDistanceLimit; // x = min, y = max;
	float zoomSpeed;

	// rotation vars
	vec2 rotationSpeed;
	vec2 yRotationLimit; // x = min, y = max
	vec2 currentRotation;

	// Helper function so angle stays within 360 and -360 range
	float ClampAngle(float angle, float min, float max);

public:
	OrbitCamera();
	inline virtual ~OrbitCamera() { }

	// Control functions
	void Rotate(const vec2& deltaRot, float deltaTime);
	void Zoom(float deltaZoom, float deltaTime);
	void Pan(const vec2& delataPan, float deltaTime);

	// Refresh at delta time
	void Update(float dt);

};
