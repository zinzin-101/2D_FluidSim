#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <shader.h>
#include <filesystem.h>

#include "ComputeShader.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include <limits>
#include <thread>
#include <map>
#include <vector>

void frameBufferSizeCallback(GLFWwindow* window, int width, int height);
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void mouseCallBack(GLFWwindow* window, double xpos, double ypos);
void processInput(GLFWwindow* window);

// settings
//#define FULLSCREEN
#ifdef FULLSCREEN
const unsigned int WIDTH = 1920;
const unsigned int HEIGHT = 1080;
#else
const unsigned int WIDTH = 1000;
const unsigned int HEIGHT = 1000;
#endif

//const float WORLD_WIDTH = 226.65f;
//const float WORLD_HEIGHT = 127.5f;
const float FIX_DT = 1.0f / 60.0f;
float deltaTime = 0.0f;
float lastTime = 0.0f;

// glfw
GLFWwindow* window = nullptr;
void toggleFullscreen(GLFWwindow* window);
float mouseX = 0.0f;
float mouseY = 0.0f;
class Fluid;
class FluidGPU;
Fluid* fluidPtr = nullptr;
//FluidGPU* fluidPtr = nullptr;
bool isMouseDown = false;
bool isRightMouseDown = false;
bool showFreeSpace = false;
bool useGPU = true;

// simulation
enum Fields {
	U_FIELD,
	V_FIELD,
	S_FIELD
};

const int BORDER_SIZE = 2;
const int BORDER_OFFSET = BORDER_SIZE / 2;
const float OVERRELAXATION = 1.9f;
const float DENSITY = 1000.0f;
const float SPACING = 3.0f;
const float OBSTACLE_RADIUS = 100.0f;

bool isNear(float a, float b);

class Fluid {
protected:
	float density;
	int sizeX;
	int sizeY;
	int totalCells;
	float spacing;
	std::vector<float> uSpeed;
	std::vector<float> vSpeed;
	std::vector<float> newSpeedU;
	std::vector<float> newSpeedV;
	std::vector<float> pressure;
	std::vector<float> freeSpace;
	std::vector<float> smoke;
	std::vector<float> newSmoke;

	int frameCount;
	float obstacleRadius;
	float obstacleX;
	float obstacleY;

	virtual void integrate(float dt, float gravity) {
		int n = sizeY;
		for (int x = BORDER_OFFSET; x < sizeX; x++) {
			for (int y = BORDER_OFFSET; y < sizeY; y++) {
				// check if it's a border cell
				float current = freeSpace.at(x * n + y);
				float below = freeSpace.at(x * n + y - 1);
				if (!isNear(current, 0.0f) && !isNear(below, 0.0f)) {
					vSpeed[x * n + y] += gravity * dt;
				}
			}
		}
	}

	virtual void solveIncompressibility(float dt, int iterations) {
		int n = sizeY;
		float pressureScaling = density * spacing / dt;
		for (int i = 0; i < iterations; i++) {
			for (int x = BORDER_OFFSET; x < sizeX - BORDER_OFFSET; x++) {
				for (int y = BORDER_OFFSET; y < sizeY - BORDER_OFFSET; y++) {
					float current = freeSpace.at(x * n + y);
					if (isNear(current, 0.0f)) continue;

					float left = freeSpace.at((x - 1) * n + y);
					float right = freeSpace.at((x + 1) * n + y);;
					float top = freeSpace.at(x * n + y + 1);
					float bottom = freeSpace.at(x * n + y - 1);
					float sum = left + right + top + bottom;
					if (isNear(sum, 0.0f)) continue;

					float div = uSpeed.at((x + 1) * n + y) - uSpeed.at(x * n + y) + vSpeed.at(x * n + y + 1) - vSpeed.at(x * n + y);
					float pressureCorrection = -div / sum;
					pressureCorrection *= OVERRELAXATION;
					pressure[x * n + y] += pressureScaling * pressureCorrection;

					uSpeed[x * n + y] -= left * pressureCorrection;
					uSpeed[(x + 1) * n + y] += right * pressureCorrection;
					vSpeed[x * n + y] -= bottom * pressureCorrection;
					vSpeed[x * n + y + 1] += top * pressureCorrection;
				}
			}
		}
	}

	virtual void extrapolate() {
		int n = sizeY;
		for (int x = 0; x < sizeX; x++) {
			uSpeed[x * n + 0] = uSpeed.at(x * n + 1);
			uSpeed[x * n + sizeY - BORDER_OFFSET] = uSpeed.at(x * n + sizeY - BORDER_SIZE);
		}
		for (int y = 0; y < sizeY; y++) {
			vSpeed[0 * n + y] = vSpeed.at(1 * n + y);
			vSpeed[(sizeX - BORDER_OFFSET) * n + y] = vSpeed.at((sizeX - BORDER_SIZE) * n + y);
		}
	}

	virtual float sampleField(float x, float y, Fields field) {
		int n = sizeY;
		float spacing1 = 1.0f / spacing;
		float spacing2 = 0.5f * spacing;

		x = std::max(std::min(x, sizeX * spacing), spacing);
		y = std::max(std::min(y, sizeY * spacing), spacing);

		float dx = 0.0f;
		float dy = 0.0f;
		std::vector<float>* f = nullptr;
		switch (field) {
		case U_FIELD:
			f = &uSpeed;
			dy = spacing2;
			break;

		case V_FIELD:
			f = &vSpeed;
			dx = spacing2;
			break;

		case S_FIELD:
			f = &smoke;
			dx = spacing2;
			dy = spacing2;
			break;
		}

		int x0 = std::min((int)std::floor((x - dx) * spacing1), sizeX - BORDER_OFFSET);
		float tx = ((x - dx) - x0 * spacing) * spacing1;
		int x1 = std::min(x0 + 1, sizeX - BORDER_OFFSET);

		int y0 = std::min((int)std::floor((y - dy) * spacing1), sizeY - BORDER_OFFSET);
		float ty = ((y - dy) - y0 * spacing) * spacing1;
		int y1 = std::min(y0 + 1, sizeY - BORDER_OFFSET);

		float sx = 1.0f - tx;
		float sy = 1.0f - ty;

		if (f == nullptr) return 0.0f;

		std::vector<float>& temp = *f;
		float value =
			sx * sy * temp.at(x0 * n + y0) +
			tx * sy * temp.at(x1 * n + y0) +
			tx * ty * temp.at(x1 * n + y1) +
			sx * ty * temp.at(x0 * n + y1);

		return value;
	}

	float getAverageUSpeedAtCell(int x, int y) {
		int n = sizeY;
		float u = (uSpeed.at(x * n + y - 1) + uSpeed.at(x * n + y) +
			uSpeed.at((x + 1) * n + y - 1) + uSpeed.at((x + 1) * n + y)) * 0.25f;
		return u;
	}

	float getAverageVSpeedAtCell(int x, int y) {
		int n = sizeY;
		float v = (vSpeed.at((x - 1) * n + y) + vSpeed.at(x * n + y) +
			vSpeed.at((x - 1) * n + y + 1) + vSpeed.at(x * n + y + 1)) * 0.25f;
		return v;
	}

	virtual void advectVelocity(float dt) {
		newSpeedU = uSpeed;
		newSpeedV = vSpeed;

		int n = sizeY;
		float spacing2 = 0.5f * spacing;

		for (int i = BORDER_OFFSET; i < sizeX; i++) {
			for (int j = BORDER_OFFSET; j < sizeY; j++) {
				// u component
				if (freeSpace.at(i * n + j) != 0.0 && freeSpace.at((i - BORDER_OFFSET) * n + j) != 0.0 && j < sizeY - BORDER_OFFSET) {
					float x = i * spacing;
					float y = j * spacing + spacing2;
					float u = uSpeed.at(i * n + j);
					float v = getAverageVSpeedAtCell(i, j);
					x -= u * dt;
					y -= v * dt;
					u = sampleField(x, y, U_FIELD);
					newSpeedU[i * n + j] = u;
				}
				// v component
				if (!isNear(freeSpace.at(i * n + j), 0.0f) && !isNear(freeSpace.at(i * n + j - BORDER_OFFSET), 0.0f) && i < sizeX - BORDER_OFFSET) {
					float x = i * spacing + spacing2;
					float y = j * spacing;
					float u = getAverageUSpeedAtCell(i, j);
					float v = vSpeed.at(i * n + j);
					x -= u * dt;
					y -= v * dt;
					v = sampleField(x, y, V_FIELD);
					newSpeedV[i * n + j] = v;
				}
			}
		}

		uSpeed = newSpeedU;
		vSpeed = newSpeedV;
	}

	virtual void advectSmoke(float dt) {
		newSmoke = smoke;

		int n = sizeY;
		float spacing2 = 0.5f * spacing;

		for (int i = BORDER_OFFSET; i < sizeX - BORDER_OFFSET; i++) {
			for (int j = BORDER_OFFSET; j < sizeY - BORDER_OFFSET; j++) {

				if (!isNear(freeSpace.at(i * n + j), 0.0f)) {
					float u = (uSpeed.at(i * n + j) + uSpeed.at((i + 1) * n + j)) * 0.5f;
					float v = (vSpeed.at(i * n + j) + vSpeed.at(i * n + j + 1)) * 0.5f;
					float x = i * spacing + spacing2 - dt * u;
					float y = j * spacing + spacing2 - dt * v;

					newSmoke[i * n + j] = sampleField(x, y, S_FIELD);
				}
			}
		}

		smoke = newSmoke;
	}


public:
	Fluid(float density, int width, int height, float spacing, float obstacleRadius): 
		density(density), sizeX(width + BORDER_SIZE), sizeY(height + BORDER_SIZE), spacing(spacing), obstacleRadius(obstacleRadius) {
		totalCells = sizeX * sizeY;
		uSpeed = std::vector<float>(totalCells);
		vSpeed = std::vector<float>(totalCells);
		newSpeedU = std::vector<float>(totalCells);
		newSpeedV = std::vector<float>(totalCells);
		pressure = std::vector<float>(totalCells);
		freeSpace = std::vector<float>(totalCells);
		smoke = std::vector<float>(totalCells);
		newSmoke = std::vector<float>(totalCells);

		std::fill(smoke.begin(), smoke.end(), 1.0f);
		std::fill(freeSpace.begin(), freeSpace.end(), 1.0f);

		frameCount = 0;
		obstacleX = 0.0f;
		obstacleY = 0.0f;
	}

	virtual void update(float dt, float gravity, int iterations) {
		integrate(dt, gravity);

		std::fill(pressure.begin(), pressure.end(), 0.0f);
		solveIncompressibility(dt, iterations);

		extrapolate();
		advectVelocity(dt);
		advectSmoke(dt);

		frameCount++;
	}

	virtual void setObstacle(float dt, float x, float y, bool reset) {
		float vx = 0.0f;
		float vy = 0.0f;
		if (!reset) {
			vx = (x - obstacleX) / dt;
			vy = (y - obstacleY) / dt;
		}

		obstacleX = x;
		obstacleY = y;

		float r = obstacleRadius;
		int n = sizeY;
		float cellDiagonalLength = std::sqrt(2.0f) * spacing;

		for (int i = BORDER_OFFSET; i < sizeX - BORDER_SIZE; i++) {
			for (int j = BORDER_OFFSET; j < sizeY - BORDER_SIZE; j++) {
				freeSpace[i * n + j] = 1.0f;

				float dx = (i + 0.5f) * spacing - x;
				float dy = (j + 0.5f) * spacing - y;

				if (dx * dx + dy * dy < r * r) {
					freeSpace[i * n + j] = 0.0f;
					//smoke[i * n + j] = 0.5f + 0.5f * std::sin(0.1f * (float)frameCount);
					smoke[i * n + j] = 0.0f;
					uSpeed[i * n + j] = vx;
					uSpeed[(i + 1) * n + j] = vx;
					vSpeed[i * n + j] = vy;
					vSpeed[i * n + j + 1] = vy;
				}

			}
		}
	}

	int getSizeX() const {
		return sizeX;
	}

	int getSizeY() const {
		return sizeY;
	}

	float* getMaterialData() {
		return smoke.data();
	}

	float* getPressureData() {
		return pressure.data();
	}

	float getSpacing() const {
		return spacing;
	}
};

class FluidGPU : public Fluid {
private:
	ComputeShader integrateShader;
	ComputeShader incompressibilityShader;
	ComputeShader extrapolateShader;
	ComputeShader advectVelocityShader;
	ComputeShader advectSmokeShader;
	ComputeShader setObstacleShader;
	unsigned int velocityTexture; // R channel: U, G channel: V
	unsigned int freeSpaceTexture; // R channel only
	unsigned int pressureTexture; // R channel only
	unsigned int newVelocityTexture; // R channel: U, G channel: V
	unsigned int smokeTexture; // R channel only
	unsigned int newSmokeTexture; // R channel only

	virtual void integrate(float dt, float gravity) override {
		integrateShader.use();
		integrateShader.setFloat("dt", dt);
		integrateShader.setFloat("gravity", gravity);

		glBindImageTexture(0, velocityTexture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
		glBindImageTexture(1, freeSpaceTexture, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);

		glDispatchCompute((sizeX + 15) / 16, (sizeY + 15) / 16, 1);
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	}

	virtual void solveIncompressibility(float dt, int iterations) override {
		incompressibilityShader.use();

		// reset all to zero
		float zero = 0.0f;
		glClearTexImage(pressureTexture, 0, GL_RED, GL_FLOAT, &zero);

		float pressureScaling = density * spacing / dt;
		incompressibilityShader.setFloat("pressureScaling", pressureScaling);
		incompressibilityShader.setFloat("overrelaxation", 1.9f);
		incompressibilityShader.setIVec2("gridSize", sizeX, sizeY);

		glBindImageTexture(0, velocityTexture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
		glBindImageTexture(1, freeSpaceTexture, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
		glBindImageTexture(2, pressureTexture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32F);

		// using 4-color Gauss-Seidel method
		for (int i = 0; i < iterations; i++) {
			for (int j = 0; j < 4; j++) {
				incompressibilityShader.setInt("pass", j);
				glDispatchCompute((sizeX + 15) / 16, (sizeY + 15) / 16, 1);
				glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
			}
		}
	}

	virtual void extrapolate() override {
		extrapolateShader.use();
		glBindImageTexture(0, velocityTexture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
		glDispatchCompute((sizeX + 15) / 16, (sizeY + 15) / 16, 1);
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	}

	virtual void advectVelocity(float dt) override {
		advectVelocityShader.use();
		advectVelocityShader.setFloat("dt", dt);
		advectVelocityShader.setFloat("spacing", spacing);

		glBindImageTexture(0, newVelocityTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, velocityTexture);
		advectVelocityShader.setInt("velocityTexture", 2);

		glDispatchCompute((sizeX + 15) / 16, (sizeY + 15) / 16, 1);
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

		std::swap(velocityTexture, newVelocityTexture);
	}

	virtual void advectSmoke(float dt) override {
		advectSmokeShader.use();
		advectSmokeShader.setFloat("dt", dt);
		advectSmokeShader.setFloat("spacing", spacing);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, velocityTexture);
		advectSmokeShader.setInt("velocityTexture", 0);

		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, smokeTexture);
		advectSmokeShader.setInt("smokeTexture", 1);

		glBindImageTexture(0, newSmokeTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
		glBindImageTexture(1, freeSpaceTexture, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);

		glDispatchCompute((sizeX + 15) / 16, (sizeY + 15) / 16, 1);
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

		std::swap(smokeTexture, newSmokeTexture);
	}

public:
	FluidGPU(float density, int width, int height, float spacing, float obstacleRadius) :
		Fluid(density, width, height, spacing, obstacleRadius),
		integrateShader("integrate.comp"),
		incompressibilityShader("incompressibility.comp"),
		extrapolateShader("extrapolate.comp"),
		advectVelocityShader("advect_velocity.comp"),
		advectSmokeShader("advect_smoke.comp"),
		setObstacleShader("set_obstacle.comp"),
		velocityTexture{}, freeSpaceTexture{}, pressureTexture{}, newVelocityTexture{}, smokeTexture{}, newSmokeTexture{}
	{
		integrateShader.use();
		integrateShader.setIVec2("gridSize", sizeX, sizeY);

		incompressibilityShader.use();
		incompressibilityShader.setIVec2("gridSize", sizeX, sizeY);

		extrapolateShader.use();
		extrapolateShader.setIVec2("gridSize", sizeX, sizeY);

		advectVelocityShader.use();
		advectVelocityShader.setIVec2("gridSize", sizeX, sizeY);

		advectSmokeShader.use();
		advectSmokeShader.setIVec2("gridSize", sizeX, sizeY);

		setObstacleShader.use();
		setObstacleShader.setIVec2("gridSize", sizeX, sizeY);

		// velocity
		glGenTextures(1, &velocityTexture);
		glBindTexture(GL_TEXTURE_2D, velocityTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, sizeX, sizeY, 0, GL_RGBA, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glGenTextures(1, &newVelocityTexture);
		glBindTexture(GL_TEXTURE_2D, newVelocityTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, sizeX, sizeY, 0, GL_RGBA, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		// free space
		glGenTextures(1, &freeSpaceTexture);
		glBindTexture(GL_TEXTURE_2D, freeSpaceTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, sizeX, sizeY, 0, GL_RED, GL_FLOAT, freeSpace.data());
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		// pressure
		glGenTextures(1, &pressureTexture);
		glBindTexture(GL_TEXTURE_2D, pressureTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, sizeX, sizeY, 0, GL_RED, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		// smoke
		glGenTextures(1, &smokeTexture);
		glBindTexture(GL_TEXTURE_2D, smokeTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, sizeX, sizeY, 0, GL_RED, GL_FLOAT, smoke.data());
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glGenTextures(1, &newSmokeTexture);
		glBindTexture(GL_TEXTURE_2D, newSmokeTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, sizeX, sizeY, 0, GL_RED, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}

	virtual void update(float dt, float gravity, int iterations) override {
		integrate(dt, gravity);
		solveIncompressibility(dt, iterations);

		extrapolate();
		advectVelocity(dt);
		advectSmoke(dt);

		frameCount++;
	}

	virtual void setObstacle(float dt, float x, float y, bool reset) override {
		std::swap(x, y);
		float vx = 0.0f;
		float vy = 0.0f;
		if (!reset) {
			vx = (x - obstacleX) / dt;
			vy = (y - obstacleY) / dt;
		}

		obstacleX = x;
		obstacleY = y;

		setObstacleShader.use();
		setObstacleShader.setVec2("mousePos", x, y);
		setObstacleShader.setVec2("mouseVel", vx, vy);
		setObstacleShader.setFloat("radius", obstacleRadius);
		setObstacleShader.setFloat("spacing", spacing);
		setObstacleShader.setBool("isReset", reset);
		setObstacleShader.setIVec2("gridSize", sizeX, sizeY);

		//float smokeColor = 0.5f + 0.5f * sin((float)frameCount * 2.0f);
		float smokeColor = 0.0f;
		setObstacleShader.setFloat("smokeColor", smokeColor);
		
		glBindImageTexture(0, velocityTexture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
		glBindImageTexture(1, freeSpaceTexture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32F);
		glBindImageTexture(2, smokeTexture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32F);

		glDispatchCompute((sizeX + 15) / 16, (sizeY + 15) / 16, 1);
		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	}

	unsigned int getSmokeTexture() const {
		return smokeTexture;
	}

	unsigned int getVelocityTexture() const {
		return velocityTexture;
	}

	unsigned int getFreeSpaceTexture() const {
		return freeSpaceTexture;
	}
};

// rendering
unsigned int vao;
unsigned int vbo;
unsigned int ebo;
unsigned int smokeTexture;

// controls
std::map<unsigned, bool> keyDownMap;
bool getKeyDown(GLFWwindow* window, unsigned int key);

int main() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor);
	#ifdef FULLSCREEN
	window = glfwCreateWindow(WIDTH, HEIGHT, "2D_FluidSim", primaryMonitor, NULL);
	#else
	window = glfwCreateWindow(WIDTH, HEIGHT, "2D_FluidSim", NULL, NULL);
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
	glfwSetCursorPosCallback(window, mouseCallBack);

	srand(time(NULL));

	stbi_set_flip_vertically_on_load(true);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	Fluid fluid = Fluid(DENSITY, 100, 100, SPACING, OBSTACLE_RADIUS / 10.0f);
	FluidGPU fluidGPU = FluidGPU(DENSITY, 1024, 1024, SPACING, OBSTACLE_RADIUS);

	fluidPtr = &fluidGPU;

	// setup quad
	float quadVertices[] = {
		// position          // uv
		 1.0f,  1.0f, 0.0f,  1.0f, 1.0f,   // Top Right (0)
		 1.0f, -1.0f, 0.0f,  1.0f, 0.0f,   // Bottom Right (1)
		-1.0f, -1.0f, 0.0f,  0.0f, 0.0f,   // Bottom Left (2)
		-1.0f,  1.0f, 0.0f,  0.0f, 1.0f    // Top Left (3)
	};

	unsigned int quadIndices[] = {
		0, 3, 2, 
		2, 1, 0  
	};

	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);
	glGenBuffers(1, &ebo);
	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quadIndices), quadIndices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0);

	// setup smoke texture
	glGenTextures(1, &smokeTexture);
	glBindTexture(GL_TEXTURE_2D, smokeTexture);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, fluid.getSizeX(), fluid.getSizeY(), 0, GL_RED, GL_FLOAT, NULL);

	Shader shader = Shader("smoke.vert", "smoke.frag");
	shader.use();
	shader.setInt("smokeTexture", 0);

	while (!glfwWindowShouldClose(window)) {
		processInput(window);

		float currentTime = (float)glfwGetTime();
		deltaTime = currentTime - lastTime;
		lastTime = currentTime;

		if (useGPU) {
			fluidPtr = &fluidGPU;
		}
		else {
			fluidPtr = &fluid;
		}

		// update
		if (isMouseDown) {
			fluidPtr->setObstacle(1.0f / 60.0f, mouseX, mouseY, isRightMouseDown);
		}
		fluidPtr->update(1.0f / 60.0f, -9.81f, 40);

		// render
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		//glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		//glBindTexture(GL_TEXTURE_2D, smokeTexture);
		//glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, fluid.getSizeX(), fluid.getSizeY(), GL_RED, GL_FLOAT, fluid.getMaterialData());
		switch (useGPU) {
			case true:
				glActiveTexture(GL_TEXTURE0);
				if (showFreeSpace) {
					glBindTexture(GL_TEXTURE_2D, fluidGPU.getFreeSpaceTexture());
				}
				else {
					glBindTexture(GL_TEXTURE_2D, fluidGPU.getSmokeTexture());
				}
				break;

			case false:
				glBindTexture(GL_TEXTURE_2D, smokeTexture);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, fluid.getSizeX(), fluid.getSizeY(), GL_RED, GL_FLOAT, fluid.getMaterialData());
				break;
		}

		//glBindTexture(GL_TEXTURE_2D, fluid.getVelocityTexture());

		shader.use();
		//glm::mat4 projection = glm::ortho(0.0f, (float)WIDTH, 0.0f, (float)HEIGHT);
		glm::mat4 projection(1.0f);
		shader.setMat4("projection", projection);
		glBindVertexArray(vao);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

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
		isMouseDown = true;
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		isMouseDown = false;
	}

	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		isRightMouseDown = true;
	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
		isRightMouseDown = false;
	}
}

void mouseCallBack(GLFWwindow* window, double xpos, double ypos) {
	if (fluidPtr == nullptr) return;

	int width, height;
	glfwGetWindowSize(window, &width, &height);

	float x = (float)xpos / (float)width;
	float y = ((float)height - (float)ypos) / (float)height;

	float worldWidth = fluidPtr->getSizeX() * fluidPtr->getSpacing();
	float worldHeight = fluidPtr->getSizeY() * fluidPtr->getSpacing();

	mouseX = y * worldWidth;
	mouseY = x * worldHeight;

}

void processInput(GLFWwindow* window) {
	if (getKeyDown(window, GLFW_KEY_ESCAPE)) {
		glfwSetWindowShouldClose(window, true);
	}

	if (getKeyDown(window, GLFW_KEY_SPACE)) {
		showFreeSpace = !showFreeSpace;
	}

	if (getKeyDown(window, GLFW_KEY_G)) {
		useGPU = !useGPU;
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

bool isNear(float a, float b) {
	return std::abs(a - b) < 1e-9;
}

glm::vec3 getSciColor(float val, float minVal, float maxVal) {
	val = std::min(std::max(val, minVal), maxVal - 0.0001f);
	float d = maxVal - minVal;
	val = isNear(d, 0.0f) ? 0.5f : (val - minVal) / d;
	float m = 0.25f;
	int num = std::floor(val / m);
	float s = (val - num * m) / m;
	float r, g, b;

	switch (num) {
		case 0: r = 0.0f; g = s; b = 1.0f; break;
		case 1: r = 0.0f; g = 1.0f; b = 1.0f - s; break;
		case 2: r = s; g = 1.0f; b = 0.0f; break;
		case 3: r = 1.0f; g = 1.0f - s; b = 0.0f; break;
	}

	return glm::vec3(r, g, b);
}