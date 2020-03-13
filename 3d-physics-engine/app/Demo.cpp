#include "Demo.h"
#include "FixedFunctionPrimitives.h"

Demo::Demo() {
	m_windowSize = vec2(0.0f, 0.0f);
}

void Demo::Initialize(int width, int height) {
	Resize(width, height);
}

void Demo::Resize(int width, int height) {
	camera.Perspective(60.0f, (float)width / (float)height, 0.01f, 1000.0f);
	m_windowSize = vec2(width, height);
}

void Demo::Update(float dt) {

	// Update the camera position
	//camera.Update(dt);
}

void Demo::Render() {
	FixedFunctionOrigin();
}