#version 330 core

layout (location = 0) in vec3 aPos;

uniform mat4 projection;
uniform mat4 view;
uniform vec3 position;
uniform float scale;

void main(){
	//gl_Position = vec4((aPos.x + position.x) * scale, (aPos.y + position.y) * scale, aPos.z, 1.0);
	//gl_Position = vec4((aPos.x * scale) + position.x, (aPos.y * scale) + position.y, aPos.z, 1.0);
	gl_Position = projection * view * vec4((aPos * scale) + position, 1.0);
}