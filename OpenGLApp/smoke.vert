#version 330 core
layout (location = 0) in vec3 aPos;   // Position (x, y, z)
layout (location = 1) in vec2 aTex;   // Texture UVs (0 to 1)

out vec2 TexCoords;

// Use a projection matrix if you want to scale the quad to your window size
uniform mat4 projection; 

void main() {
    // Pass the UVs to the fragment shader
    TexCoords = aTex;
    
    // Set the final vertex position
    gl_Position = projection * vec4(aPos, 1.0);
}