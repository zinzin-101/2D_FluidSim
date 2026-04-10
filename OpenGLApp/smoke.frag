#version 330 core
out vec4 FragColor;
in vec2 TexCoords;

uniform sampler2D smokeTexture;

void main() {
    float density = texture(smokeTexture, TexCoords).r;
    
    // Render as white smoke on a dark background
    vec3 smokeColor = vec3(1.0, 1.0, 1.0);
    FragColor = vec4(smokeColor, density); 
    //FragColor = vec4(TexCoords, 1.0, 1.0);
    //FragColor = vec4(0.0, 1.0, 0.0, 1.0);
}