#define UNICODE
#define _UNICODE
#include <stdlib.h>
#include <crtdbg.h>
#include <float.h>

#include <iostream>

#include <windows.h>
#include <windowsx.h>

#include "include/glad/glad.h"
//#include "include/glfw3.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <conio.h>

#include "app/BoxDemo.h"
#include "app/BowlingDemo.h"
#include "app/ClothDemo.h"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR szCmdLine, int iCmdShow);
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
void OpenGLUnbindContext(HWND hwnd, HDC hdc, HGLRC hglrc);
HGLRC OpenGLBindContext(HDC hdc);
double GetMilliseconds();

//void framebuffer_size_callback(GLFWwindow* window, int width, int height);
//void processInput(GLFWwindow* window);

#define WWIDTH 800
#define WHEIGHT 600

RECT windowRect;
RECT clientRect;
RECT borderRect;

int main()
{
	WinMain(GetModuleHandle(NULL), NULL, GetCommandLineA(), SW_SHOWDEFAULT);

	return 0;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR szCmdLine, int iCmdShow)
{
	//std::cout << "Initialised\n";

	//// Init GLFW
	//glfwInit();
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	//glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	//// GLFW window
	//GLFWwindow* window = glfwCreateWindow(WWIDTH, WHEIGHT, "LearnOpenGL", NULL, NULL);
	//if (window == NULL)
	//{
	//	std::cout << "Failed to create GLFW window" << std::endl;
	//	glfwTerminate();
	//	return -1;
	//}

	//// Some additional input modes for GLFW
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	//// Set as context
	//glfwMakeContextCurrent(window);
	HINSTANCE hinstance = hInstance;
	WNDCLASSEX wndclass;

	ZeroMemory(&wndclass, sizeof(WNDCLASSEX));
	wndclass.cbSize = sizeof(WNDCLASSEX);
	wndclass.style = CS_HREDRAW | CS_VREDRAW;
	wndclass.lpfnWndProc = WindowProc;
	wndclass.hInstance = hInstance;
	wndclass.lpszClassName = L" ";
	RegisterClassEx(&wndclass);

	SetRect(&windowRect, (GetSystemMetrics(SM_CXSCREEN) / 2) - (WWIDTH / 2),
			(GetSystemMetrics(SM_CYSCREEN) / 2) - (WHEIGHT / 2),
			(GetSystemMetrics(SM_CXSCREEN) / 2) + (WWIDTH / 2),
			(GetSystemMetrics(SM_CYSCREEN) / 2) + (WHEIGHT / 2));
	AdjustWindowRectEx(&windowRect, WS_VISIBLE, FALSE, 0);

	HWND hwnd = CreateWindowEx(0, L" ", L"", WS_VISIBLE,
							   windowRect.left, windowRect.top, windowRect.right - windowRect.left, windowRect.bottom - windowRect.top,
							   NULL, NULL, hInstance, NULL);

	HDC hdc = GetDC(hwnd);
	HGLRC hglrc = OpenGLBindContext(hdc);

	// Init GLAD POINTERS
	//if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	if (!gladLoadGL())
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	std::cout << "OpenGL Context: " << GLVersion.major << ", " << GLVersion.minor << "\n";
	std::cout << "OpenGL version: " << glGetString(GL_VERSION) << "\n";
	std::cout << "GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << "\n";

	// WINDOW pWindowInstance->OnInitialize()
	glClearColor(0.5f, 0.6f, 0.7f, 1.0f);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	// Set-up viewport
	glViewport(0, 0, WWIDTH, WHEIGHT);

	// Set-up resizing callback
	//glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	ShowWindow(hwnd, SW_SHOW);
	UpdateWindow(hwnd);
	GetClientRect(hwnd, &clientRect);
	GetWindowRect(hwnd, &windowRect);
	GetWindowRect(hwnd, &borderRect);

	// Setup FPS and timers
	// ------------------------------------------------------------------------

#define FIXED_FPS 30.0
#define TARGET_FPS 60.0
	auto next_game_tick = GetTickCount();
	int sleep_time = 0;

	//	double lastTime = glfwGetTime();
	double lastTime = GetMilliseconds();
	double deltaTime = 0.0;
	double nowTime = 0.0;
	double fixed_millis = FIXED_FPS / 1000.0;
	double fixed_ellapsed = 0.0;

	Demo *currentDemo = nullptr;

	// Run loop
	//while (!glfwWindowShouldClose(window))
	bool stop = false;
	while (!stop)
	{
		//processInput(window);

		// ------------------------------------------------------------------------
		// Measure Time
		//	nowTime = glfwGetTime();
		nowTime = GetMilliseconds();
		deltaTime = (nowTime - lastTime) * 0.001f;
		lastTime = nowTime;

		// ----------------------------------------------------------------
		// ON UPDATE of window
		if (currentDemo == nullptr)
		{
			//currentDemo = new BoxDemo();
			//currentDemo = new BowlingDemo();
			currentDemo = new ClothDemo();
			currentDemo->Initialize(WWIDTH, WHEIGHT);

			//Apply DEMO CAMERA
			if (currentDemo != nullptr)
			{
				mat4 projection = currentDemo->camera.GetProjectionMatrix();
				mat4 view = currentDemo->camera.GetViewMatrix();

				//SetGLProjection(projection.asArray);
				glMatrixMode(GL_PROJECTION);
				glLoadMatrixf(projection.asArray);

				//SetGLModelView(view.asArray);
				glMatrixMode(GL_MODELVIEW);
				glLoadMatrixf(view.asArray);
				//std::cout << "applying camera\n";
			}
		}

		// ------------------------------------------------------
		// - Only update at 60 frames / s
		fixed_ellapsed += deltaTime;
		//std::cout << fixed_ellapsed << " elapsed / millies " << fixed_millis << std::endl;
		while (fixed_ellapsed > fixed_millis)
		{
			// On fixed Update
			if (currentDemo != nullptr)
			{
				currentDemo->Update(fixed_millis);
			}
			fixed_ellapsed -= fixed_millis;
		}

		// DEMO WINDOW ON RENDER
		// ----------------------------------------------------------------
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		if (currentDemo != nullptr)
		{
			mat4 view = currentDemo->camera.GetViewMatrix();
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(view.asArray);

			currentDemo->Render();
		}

		// -----------------------------------------------------------
		//glfwSwapBuffers(window);
		//glfwPollEvents();
		SwapBuffers(hdc);

		// -----------------------------------------------------------

		// Regulate FPS
		int SKIP_TICKS = 1000 / TARGET_FPS;
		next_game_tick += SKIP_TICKS;
		sleep_time = next_game_tick - GetTickCount();
		if (sleep_time >= 0)
		{
			Sleep(sleep_time);
		}
	}

	//terminate
	//	glfwTerminate();
	return 0;
}

//void processInput(GLFWwindow* window)
//{
//	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//		glfwSetWindowShouldClose(window, true);
//}

//void framebuffer_size_callback(GLFWwindow* window, int width, int height)
//{
//	glViewport(0, 0, width, height);
//}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;

	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hwnd, &ps);

		FillRect(hdc, &ps.rcPaint, (HBRUSH)(COLOR_WINDOW + 1));

		EndPaint(hwnd, &ps);
	}
		return 0;
	}
	return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

HGLRC OpenGLBindContext(HDC hdc)
{
	PIXELFORMATDESCRIPTOR pfd;
	ZeroMemory(&pfd, sizeof(PIXELFORMATDESCRIPTOR));

	pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pfd.nVersion = 1;
	pfd.dwFlags = PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 24;
	pfd.cDepthBits = 32;
	pfd.cStencilBits = 8;
	pfd.iLayerType = PFD_MAIN_PLANE;

	int pixelFormat = ChoosePixelFormat(hdc, &pfd);
	SetPixelFormat(hdc, pixelFormat, &pfd);

	HGLRC context = wglCreateContext(hdc);
	wglMakeCurrent(hdc, context);
	return context;
}

void OpenGLUnbindContext(HWND hwnd, HDC hdc, HGLRC hglrc)
{
	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hglrc);
	ReleaseDC(hwnd, hdc);
}

double GetMilliseconds()
{
	static LARGE_INTEGER s_frequency;
	static BOOL s_use_qpc = QueryPerformanceFrequency(&s_frequency);
	if (s_use_qpc)
	{
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		return (double)((1000LL * now.QuadPart) / s_frequency.QuadPart);
	}
	else
	{
		return GetTickCount();
	}
}