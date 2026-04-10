#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <shader.h>
#include <filesystem.h>

#include "Utils.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include <limits>
#include <thread>
#include <map>
#include <vector>

void frameBufferSizeCallback(GLFWwindow* window, int width, int height);
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void processInput(GLFWwindow* window);

// settings
//#define FULLSCREEN
#ifdef FULLSCREEN
const unsigned int WIDTH = 1920;
const unsigned int HEIGHT = 1080;
#else
const unsigned int WIDTH = 1600;
const unsigned int HEIGHT = 900;
#endif

//const float WORLD_WIDTH = 226.65f;
//const float WORLD_HEIGHT = 127.5f;
const float FIX_DT = 1.0f / 60.0f;
float deltaTime = 0.0f;
float lastTime = 0.0f;

// glfw
GLFWwindow* window = nullptr;
void toggleFullscreen(GLFWwindow* window);


// controls
std::map<unsigned, bool> keyDownMap;
bool getKeyDown(GLFWwindow* window, unsigned int key);

int main() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor);
	#ifdef FULLSCREEN
	window = glfwCreateWindow(WIDTH, HEIGHT, "2D_Pinball", primaryMonitor, NULL);
	#else
	window = glfwCreateWindow(WIDTH, HEIGHT, "2D_Pinball", NULL, NULL);
	glfwSetWindowPos(window, (mode->width / 2) - (WIDTH / 2), (mode->height / 2) - (HEIGHT / 2));
	#endif
	if (window == NULL) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	glfwSetFramebufferSizeCallback(window, frameBufferSizeCallback);
	glfwSetMouseButtonCallback(window, mouseButtonCallback);

	srand(time(NULL));

	stbi_set_flip_vertically_on_load(true);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	while (!glfwWindowShouldClose(window)) {
		processInput(window);

		float currentTime = (float)glfwGetTime();
		deltaTime = currentTime - lastTime;
		lastTime = currentTime;

		// update
		//+

		// render
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		//+

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	return 0; 
}

void frameBufferSizeCallback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {

	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {

	}

	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {

	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {

	}
}

void processInput(GLFWwindow* window) {
	if (getKeyDown(window, GLFW_KEY_ESCAPE)) {
		glfwSetWindowShouldClose(window, true);
	}
}

void toggleFullscreen(GLFWwindow* window) {
	GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor);

	if (glfwGetWindowMonitor(window)) { // full screen -> windowed
		glfwSetWindowMonitor(window, NULL, (mode->width / 2) - (WIDTH / 2), (mode->height / 2) - (HEIGHT / 2), WIDTH, HEIGHT, GLFW_DONT_CARE);
	}
	else { // windowed -> full screen
		glfwSetWindowMonitor(window, primaryMonitor, 0, 0, mode->width, mode->height, mode->refreshRate);
	}
}

bool getKeyDown(GLFWwindow* window, unsigned int key) {
	// init
	if (keyDownMap.count(key) == 0) {
		keyDownMap[key] = false;
		return false;
	}

	if (glfwGetKey(window, key) == GLFW_PRESS && keyDownMap.at(key)) {
		return false;
	}

	if (glfwGetKey(window, key) == GLFW_RELEASE && keyDownMap.at(key)) {
		keyDownMap[key] = false;
		return false;
	}

	if (glfwGetKey(window, key) == GLFW_PRESS && !keyDownMap.at(key)) {
		keyDownMap[key] = true;
		return true;
	}
}