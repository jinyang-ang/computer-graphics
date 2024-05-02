//
// Created by pushpak on 6/1/18.
//


#include "SimpleScene_Quad.h"
#include <shader.hpp>
#include <glm/vec3.hpp>
//#include <glm/detail/type_mat.hpp>

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
void SimpleScene_Quad::SetupNanoGUI(GLFWwindow *pWindow)
{
    pWindow = nullptr;
}


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
SimpleScene_Quad::~SimpleScene_Quad()
{
    initMembers();
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
SimpleScene_Quad::SimpleScene_Quad(int windowWidth, int windowHeight) : Scene(windowWidth, windowHeight),
                                                                        programID(0), vertexbuffer(0), VertexArrayID(0),
                                                                        angleOfRotation(0.0f)
{
    initMembers();
}

//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "bugprone-unused-return-value"
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
void SimpleScene_Quad::initMembers()
{
    programID = 0;
    vertexbuffer = 0;
    VertexArrayID = 0;

    geometryBuffer.clear();
    angleOfRotation = 0.0f;
}
//#pragma clang diagnostic pop

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
void SimpleScene_Quad::CleanUp()
{
    // Cleanup VBO
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteProgram(programID);
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
void SimpleScene_Quad::SetupBuffers()
{
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    glGenBuffers(1, &vertexbuffer);

    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, geometryBuffer.size() * sizeof(GLfloat),
                 geometryBuffer.data(), GL_STATIC_DRAW);

    return;
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
int SimpleScene_Quad::Init()
{
    // Create and compile our GLSL program from the shaders
    programID = LoadShaders("../Common/shaders/QuadVertexShader.vert",
                            "../Common/shaders/QuadFragmentShader.frag");

    // Just two triangles making up a quad
    geometryBuffer = { 0.0f, 0.0f, 0.0f,
                        1.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f,
                        1.0f, 0.0f, 0.0f,
                        1.0f, 1.0f, 0.0f,
                        0.0f, 1.0f, 0.0f
    };

    SetupBuffers();

    return Scene::Init();
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
int SimpleScene_Quad::Render()
{
    //glClearColor( 0.5f, 0.5f, 0.5f, 1.0f );
    glClearColor(1.0f, 0.f, 0.f, 1.0f);

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);


    glUseProgram(programID);

    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer( 0,
                            3,
                            GL_FLOAT,
                            GL_FALSE,
                            0,
                           (void *) 0 );

    // Uniform matrix
    // Uniform transformation (vertex shader)
    GLint vTransformLoc = glGetUniformLocation(programID, "vertexTransform");

    // Draw the triangle !
    // T * R * S * Vertex
    glm::mat4 modelMat = glm::mat4(1.0f);
    glm::vec3 scaleVector = glm::vec3(1.0f);
    glm::vec3 centroid = glm::vec3( -0.5f, -0.5f, 0.0f );
    modelMat = glm::rotate(angleOfRotation, glm::vec3(0.0f, 0.0f, 1.0f)) *
            glm::scale( scaleVector ) *
            glm::translate( centroid );

    glUniformMatrix4fv(vTransformLoc, 1, GL_FALSE, &modelMat[0][0]);

    glDrawArrays(GL_TRIANGLES, 0, 6);

    glDisableVertexAttribArray(0);

    return 0;
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
int SimpleScene_Quad::postRender()
{
    angleOfRotation += 0.01f;
    return 0;
}
