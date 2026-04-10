#pragma once
#include <cmath>
#include <cstdlib>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace Utils {
	constexpr float PI = 3.14159f;

	inline float deg2Rad(float deg) {
		return (deg * PI) / 180.0f;
	}

	inline float RandFloat() {
		return (float)rand() / (float)RAND_MAX;
	}

	glm::vec2 getClosestPointOnSegment(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
		glm::vec2 ab = b - a;
		float t = glm::dot(ab, ab);
		if (t == 0.0f) return a;
		t = glm::max(0.0f, glm::min(1.0f, (glm::dot(p, ab) - glm::dot(a, ab)) / t));
		glm::vec2 closest = a;
		return closest + ab * t;
	}

	inline glm::vec2 getPerpendicular(glm::vec2 v) {
		return glm::vec2(-v.y, v.x);
	}
}
