#include <iostream>

#include <windows.h>

#include "include/glad/glad.h"
#include "include/glfw3.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <conio.h>

#include "app/ControlledCamera.h"

#include "app/BoxDemo.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

#define WWIDTH 1200
#define WHEIGHT 800
#define FIXED_FPS 30.0
#define TARGET_FPS 30.0

int main() {
	std::cout << "Initialised\n";
	
	// Init GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// GLFW window
	GLFWwindow* window = glfwCreateWindow(WWIDTH, WHEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	// Some additional input modes for GLFW
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	
	// Set as context
	glfwMakeContextCurrent(window);


	// Init GLAD POINTERS
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
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


	// Setup FPS and timers
	auto next_game_tick = GetTickCount64();
	int sleep_time = 0;

	double lastTime = glfwGetTime();
	double deltaTime = 0;
	double nowTime = 0;
	double fixed_millis = FIXED_FPS / 1000.0;
	double fixed_ellapsed = 0.0;

	Demo* currentDemo = nullptr;
	
	// Run loop
	while (!glfwWindowShouldClose(window))
	{
		//processInput(window);

		// ------------------------------------------------------------------------
		// Measure Time
		nowTime = glfwGetTime();
        deltaTime += (nowTime - lastTime) * 0.001f;
        lastTime = nowTime;
	
		// ----------------------------------------------------------------
		// ON UPDATE of window
		if (currentDemo == nullptr) {
			currentDemo = new BoxDemo();					
			currentDemo->Initialize(WWIDTH, WHEIGHT);		

			//Apply DEMO CAMERA
			if (currentDemo != nullptr) {
				mat4 projection = currentDemo->camera.GetProjectionMatrix();
				mat4 view = currentDemo->camera.GetViewMatrix();
				
				//SetGLProjection(projection.asArray);
				glMatrixMode(GL_PROJECTION);
				glLoadMatrixf(projection.asArray);

				//SetGLModelView(view.asArray);
				glMatrixMode(GL_MODELVIEW);
				glLoadMatrixf(view.asArray);
				std::cout << "applying camera\n";
			}
		}

		// ------------------------------------------------------
		// - Only update at 60 frames / s
		fixed_ellapsed += deltaTime;
		//std::cout << fixed_ellapsed << " elapsed / millies " << fixed_millis << std::endl;
		while (fixed_ellapsed >= fixed_millis) {
			// On fixed Update
			if (currentDemo != nullptr) {
				currentDemo->Update(fixed_millis);
			}
			fixed_ellapsed-= fixed_millis;
		}

		// DEMO WINDOW ON RENDER
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		if (currentDemo != nullptr) {
			mat4 view = currentDemo->camera.GetViewMatrix();
			glMatrixMode(GL_MODELVIEW);  
			glLoadMatrixf(view.asArray);
			//std::cout << "Render\n";
			currentDemo->Render();
		}

		// -----------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();

		// -----------------------------------------------------------


		// Regulate FPS
		int SKIP_TICKS = 1000 / TARGET_FPS;
		next_game_tick += SKIP_TICKS;
		sleep_time = next_game_tick - GetTickCount64();
		if (sleep_time >= 0) {
			Sleep(sleep_time);
		}
	}

	//terminate
	glfwTerminate();
	return 0;

}

void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}