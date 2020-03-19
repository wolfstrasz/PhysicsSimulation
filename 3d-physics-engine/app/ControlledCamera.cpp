#include "ControlledCamera.h"

ControlledCamera::ControlledCamera() {
	target = vec3(0, 0, 0);
	zoomDistance = 10.0f;
	zoomSpeed = 200.0f;
	rotationSpeed = vec2(250.0f, 120.0f);
	yRotationLimit = vec2(-20.0f, 80.0f);
	zoomDistanceLimit = vec2(3.0f, 15.0f);
	currentRotation = vec2(0, 0);
	panSpeed = vec2(180.0f, 180.0f);
}

void ControlledCamera::Rotate(const vec2& deltaRot, float deltaTime) {
	currentRotation.x += deltaRot.x * rotationSpeed.x * zoomDistance * deltaTime;
	currentRotation.y += deltaRot.y * rotationSpeed.y * zoomDistance * deltaTime;

	currentRotation.x = ClampAngle(currentRotation.x, -360, 360);
	currentRotation.y = ClampAngle(currentRotation.y, yRotationLimit.x, yRotationLimit.y);
}

void ControlledCamera::Zoom(float deltaZoom, float deltaTime) {
	zoomDistance = zoomDistance + deltaZoom * zoomSpeed * deltaTime;

	// Clamp zoom distance (if disabled then camera can free move forward)
	if (zoomDistance < zoomDistanceLimit.x) {
		zoomDistance = zoomDistanceLimit.x;
	}
	if (zoomDistance > zoomDistanceLimit.y) {
		zoomDistance = zoomDistanceLimit.y;
	}
}

void ControlledCamera::Pan(const vec2& delataPan, float deltaTime) {

	vec3 right(m_World._11, m_World._12, m_World._13);

	// Allow camera to move left and right
	float xPanMag = delataPan.x * panSpeed.x * deltaTime;
	target = target - (right * xPanMag);

	// Allow camera to move up and down
	float yPanMag = delataPan.y * panSpeed.y * deltaTime;
	target = target + (vec3(0, 1, 0) * yPanMag);
}

void ControlledCamera::Update(float dt) {
	vec3 rotation = vec3(currentRotation.y, currentRotation.x, 0);
	mat3 orient = Rotation3x3(rotation.x, rotation.y, rotation.z);
	vec3 direction = MultiplyVector( vec3(0.0, 0.0, -zoomDistance), orient);
	vec3 position = direction + target;

	// TODO: apply fast method as LookAt is orthogonal (transform first to orthonormal)
	m_World = FastInverse(LookAt(position, target, vec3(0, 1, 0)));
}

float ControlledCamera::ClampAngle(float angle, float min, float max) {
	while (angle < -360) {
		angle += 360;
	}
	while (angle > 360) {
		angle -= 360;
	}
	if (angle < min) {
		angle = min;
	}
	if (angle > max) {
		angle = max;
	}
	return angle;
}

void ControlledCamera::SetTarget(const vec3& newTarget) {
	target = newTarget;
}

void ControlledCamera::SetZoom(float zoom) {
	zoomDistance = zoom;
}

void ControlledCamera::SetRotation(const vec2& rotation) {
	currentRotation = rotation;
}

