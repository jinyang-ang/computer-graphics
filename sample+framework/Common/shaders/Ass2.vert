#version 410 core

uniform mat4 mvpMatrixLoc;

layout(location = 0) in vec3 vPosition;

void main()
{
    gl_Position = mvpMatrixLoc * vec4(vPosition, 1.0);
}
