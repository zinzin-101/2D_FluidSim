#version 330 core
in vec2 TexCoords;
out vec4 Color;

uniform sampler2D sprite;
uniform vec3 color;
uniform vec2 offset;
uniform vec2 frameScale;

void main()
{
    vec2 finalTexCoords = TexCoords * frameScale + offset;
    Color = vec4(color, 1.0) * texture(sprite, finalTexCoords);
}