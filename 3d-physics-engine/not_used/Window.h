#pragma once

class Window {

protected:
	bool m_exitFlag;
	bool m_isFullscreen;
	bool m_isVisible;

	int  m_width;
	int  m_height;

	int m_targedFPS;
	int m_fixedFPS;

public:

	Window();
	Window(int width, int height);

	virtual ~Window() {};

	virtual void OnInitialize() { }
	virtual void OnUpdate(float deltaTime) { }
	virtual void OnFixedUpdate(float fixedDeltaTime) { }
	virtual void OnRender() { }
	virtual void OnShutdown() { }

	virtual void OnResize(int width, int height) { }

	virtual void OnMouseMove(int x, int y) { }
	virtual void OnMouseDown(int mouseCode) { }
	virtual void OnMouseUp(int mouseCode) { }

	virtual void OnKeyDown(int keyCode) { }
	virtual void OnKeyUp(int keyCode) { }

	void SetFullScreen(bool value);
	bool GetFullScreen();

	void Resize(int width, int height);
	int GetWidth();
	int GetHeight();

	void Close();
	bool GetExitFlag();

	void SetTargetFPS(int target);
	int GetTargetFPS();
	int GetFixedFPS();

	void MarkAsShown();
	bool WasWindowShown();

	int KeyIndex(int keyCode);
};
