#version 330 core
in vec2 TexCoords;
out vec4 Color;

uniform sampler2D sprite;
uniform vec3 color;

void main()
{
    Color = vec4(color, 1.0) * texture(sprite, TexCoords);
}