#pragma once
#include "Camera.h"


class ControlledCamera : public Camera {
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

	

public:
	ControlledCamera();
	inline virtual ~ControlledCamera() { }

	// Control functions
	void Rotate(const vec2& deltaRot, float deltaTime);
	void Zoom(float deltaZoom, float deltaTime);
	void Pan(const vec2& delataPan, float deltaTime);

	// Refresh at delta time
	void Update(float dt);

	// Helper function so angle stays within 360 and -360 range
	float ClampAngle(float angle, float min, float max);

	// Setters
	void SetTarget(const vec3& newTarget);
	void SetZoom(float zoom);
	void SetRotation(const vec2& rotation);

};
