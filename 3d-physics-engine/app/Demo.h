#pragma once


#include "ControlledCamera.h"


class Demo {

private: // Disable
	Demo(const Demo&);
	Demo& operator=(const Demo);

	vec2 m_windowSize;
public:
	ControlledCamera camera;

public:
	Demo();
	inline virtual ~Demo() {}

	virtual void Initialize(int width, int height);
	virtual void Resize(int width, int height);
	virtual void Render();
	virtual void Update(float dt);
	inline virtual void Shutdown() { }
};