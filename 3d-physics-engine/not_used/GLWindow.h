#pragma once

#include "Window.h"
#include "../maths/matrices.h"


class GLWindow : public Window {
protected:

	vec2 m_vecMousePos;
	unsigned int mouseButtonState;
	bool keyboardState[256];

public:
	GLWindow(int width, int height);
	virtual ~GLWindow() { }
	void SetClearColor(float r, float g, float b);

	virtual void OnInitialize();
	virtual void OnRender();
	virtual void OnResize(int width, int height);

	virtual void OnMouseMove(int x, int y);
	virtual void OnMouseDown(int mouseCode);
	virtual void OnMouseUp(int mouseCode);

	virtual void OnKeyDown(int keyCode);
	virtual void OnKeyUp(int keyCode);

	vec2 GetMousePosition();
	bool MouseButonDown(int button);
	bool KeyDown(int key);

	void SetGLProjection(float* projectionArray);
	void SetGLModelView(float* mvArray);
};