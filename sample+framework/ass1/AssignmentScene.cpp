/*!
@file    AssignmentScene.cpp
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    20/05/2023

This file implements functionality useful and necessary for assignment 1,
implementing the various primitive classes and their respective
intersection test

*//*__________________________________________________________________________*/


/*                                                                   includes
----------------------------------------------------------------------------- */
#include "AssignmentScene.h"
#include <shader.hpp>
#include <glm/vec3.hpp>
#include <string>
#include "../Common/imgui/imgui.h"
// to integerate imgui with glfw lib
#include "../Common/imgui/imgui_impl_glfw.h"
// to render imgui UI using opengl
#include "../Common/imgui/imgui_impl_opengl3.h"
// to load from .scn
#include <fstream>
#include <sstream>
#include <iostream>
#include "Intersection.h"

# define PI 3.14159265358979323846  /* pi */

namespace Assignment1{

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::SetupNanoGUI
    @param GLFWwindow* pWindow
    @return none

    Initialize Imgui with mouse and keyboard callbacks
    */
    void AssignmentScene::SetupNanoGUI(GLFWwindow* pWindow)
    {
        // Initialize ImGui
        ImGui::CreateContext();
        ImGui_ImplGlfw_InitForOpenGL(pWindow, true);
        ImGui_ImplOpenGL3_Init("#version 460");

        // Set up ImGui click events
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

        // Register mouse button callback
        glfwSetMouseButtonCallback(pWindow, ImGui_ImplGlfw_MouseButtonCallback);
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::~AssignmentScene()
    @return none

    Destructor. Reset all core data members to default
    */
    AssignmentScene::~AssignmentScene()
    {
        initMembers();
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::AssignmentScene()
    @param int windowWidth
    @param int windowHeight
    @param GLFWwindow* pWindow
    @return none

    Initializes core data members
    */
    AssignmentScene::AssignmentScene(int windowWidth, int windowHeight, GLFWwindow* pWwindow, int Wheight, int Wwidth)
        : Scene(windowWidth, windowHeight), currentWindow(pWwindow), WindowHeight(Wheight), WindowWidth(Wwidth),
        programID(0), vertexbuffer(0), VertexArrayID(0),
        angleOfRotation(0.0f)
    {
        initMembers();
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::initMembers()
    @return none

    Initializes core data members to default
    */
    void AssignmentScene::initMembers()
    {
        programID = 0;
        vertexbuffer = 0;
        VertexArrayID = 0;

        geometryBuffer.clear();
        angleOfRotation = 0.0f;

        // initialize last mouse position to centre of screen for mouse fly
        mouse.xLast = static_cast<float> (WindowWidth / 2);
        mouse.yLast = static_cast<float> (WindowHeight / 2);
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::CleanUp()
    @return none

    Cleanup for buffers and Imgui
    */
    void AssignmentScene::CleanUp()
    {
        // Cleanup VBO
        glDeleteBuffers(1, &vertexbuffer);
        glDeleteVertexArrays(1, &VertexArrayID);
        glDeleteProgram(programID);

        // Cleanup
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::SetupBuffers()
    @return none

    Setting up buffers to be sent to vertex shader
    */
    void AssignmentScene::SetupBuffers()
    {

        glGenVertexArrays(1, &VertexArrayID);
        glBindVertexArray(VertexArrayID);

        glGenBuffers(1, &vertexbuffer);

        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, geometryBuffer.size() * sizeof(GLfloat),
                     geometryBuffer.data(), GL_STATIC_DRAW);

        return;
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::LoadMesh()
    @return bool

    Loads mesh from predefined location and in some models generate the vertices
    and stores them all into geometry buffer.
    returns false if failure to locate/open file
    */
    bool AssignmentScene::LoadMesh(Primitives modeltype)
    {

        std::ifstream file;

        switch (modeltype)
        {
            case Assignment1::AssignmentScene::Point:
                file.open("../Common/Ass1/Point.msh");
                break;
            case Assignment1::AssignmentScene::Plane:
                file.open("../Common/Ass1/Plane.msh");
                break;
            case Assignment1::AssignmentScene::Triangle:
                file.open("../Common/Ass1/Triangle.msh");
                break;
            case Assignment1::AssignmentScene::BoundingSphere:
                file.open("../Common/Ass1/BoundingSphere.msh");
                break;
            case Assignment1::AssignmentScene::AABB:
                file.open("../Common/Ass1/AABB.msh");
                break;
            case Assignment1::AssignmentScene::Ray:
                file.open("../Common/Ass1/Ray.msh");
                break;
            default:
                break;
        }

        if (!file.is_open())
        {
            std::cout << "Failed to open mesh file: " << modeltype <<  std::endl;
            return false;
        }
        std::string line;
        std::string model;
        std::getline(file, line);
        size_t vertexcount = geometryBuffer.size();
        std::istringstream iss(line);
        switch (modeltype)
        {
            case Assignment1::AssignmentScene::Point:
            {
                PointModel point;
                if (iss >> point.position.x >> point.position.y >> point.position.z)
                {
                    // Add the point's geometry to the buffer
                    geometryBuffer.push_back(point.position.x);
                    geometryBuffer.push_back(point.position.y);
                    geometryBuffer.push_back(point.position.z);
                }
                point.vertexCount = static_cast<int>(geometryBuffer.size() - vertexcount) / 3;
                pointModels.push_back(point);

            }
            break;
            case Assignment1::AssignmentScene::Plane:
            {
                PlaneModel plane;
                if (iss >> plane.P0.x >> plane.P0.y >> plane.P0.z
                    >> plane.P1.x >> plane.P1.y >> plane.P1.z
                    >> plane.P2.x >> plane.P2.y >> plane.P2.z)
                {
                    // Add the plane's geometry to the buffer
                    geometryBuffer.push_back(plane.P0.x);
                    geometryBuffer.push_back(plane.P0.y);
                    geometryBuffer.push_back(plane.P0.z);
                    geometryBuffer.push_back(plane.P1.x);
                    geometryBuffer.push_back(plane.P1.y);
                    geometryBuffer.push_back(plane.P1.z);
                    geometryBuffer.push_back(plane.P2.x);
                    geometryBuffer.push_back(plane.P2.y);
                    geometryBuffer.push_back(plane.P2.z);
                    glm::vec3 normal = glm::normalize(glm::cross((plane.P1 - plane.P0), (plane.P2 - plane.P0)));
                    plane.data = glm::vec4(normal, glm::dot(normal, plane.P0));
                    plane.positionforimgui = plane.positionforimguiold = glm::vec3(0.f);
                    plane.vertexCount = static_cast<int>(geometryBuffer.size() - vertexcount) / 3;
                    planeModels.push_back(plane);

                }
            }
            break;
            case Assignment1::AssignmentScene::Triangle:
            {
                TriangleModel triangle;
                if (iss >> triangle.v0.x >> triangle.v0.y >> triangle.v0.z
                    >> triangle.v1.x >> triangle.v1.y >> triangle.v1.z
                    >> triangle.v2.x >> triangle.v2.y >> triangle.v2.z)
                {
                    // Add the triangle's geometry to the buffer
                    geometryBuffer.push_back(triangle.v0.x);
                    geometryBuffer.push_back(triangle.v0.y);
                    geometryBuffer.push_back(triangle.v0.z);
                    geometryBuffer.push_back(triangle.v1.x);
                    geometryBuffer.push_back(triangle.v1.y);
                    geometryBuffer.push_back(triangle.v1.z);
                    geometryBuffer.push_back(triangle.v2.x);
                    geometryBuffer.push_back(triangle.v2.y);
                    geometryBuffer.push_back(triangle.v2.z);
                    triangle.positionforimgui = triangle.positionforimguiold = glm::vec3(0.f);
                    triangle.vertexCount = static_cast<int>(geometryBuffer.size() - vertexcount) / 3;
                    triangleModels.push_back(triangle);

                }
            }
            break;
            case Assignment1::AssignmentScene::BoundingSphere:
            {
                BoundingSphereModel sphere;
                if (iss >> sphere.position.x >> sphere.position.y >> sphere.position.z >> sphere.radius
                    >> sphere.sectorCount >> sphere.stackCount)
                {
                    float x, y, z, xy;

                    float sectorStep = static_cast<float>(2 * PI / sphere.sectorCount);
                    float stackStep = static_cast<float>(PI / sphere.stackCount);
                    float sectorAngle, stackAngle;

                    for (int i = 0; i <= sphere.stackCount; ++i)
                    {
                        stackAngle = static_cast<float>(PI / 2 - i * stackStep);        // starting from pi/2 to -pi/2
                        xy = sphere.radius * cosf(stackAngle); // r * cos(u)
                        z = sphere.radius * sinf(stackAngle);  // r * sin(u)

                        // add (sectorCount+1) vertices per stack
                        // first and last vertices have the same position and normal, but different tex coords
                        for (int j = 0; j <= sphere.sectorCount; ++j)
                        {
                            sectorAngle = j * sectorStep;           // starting from 0 to 2pi

                            // vertex position (x, y, z)
                            x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
                            y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)
                            geometryBuffer.push_back(x + sphere.position.x);
                            geometryBuffer.push_back(y + sphere.position.y);
                            geometryBuffer.push_back(z + sphere.position.z);
                        }
                    }

                    sphere.vertexCount = static_cast<int>(geometryBuffer.size() - vertexcount) / 3;
                    boundingSphereModels.push_back(sphere);

                }
            }
            break;
            case Assignment1::AssignmentScene::AABB:
            {
                AABBModel aabb;
                if (iss >> aabb.min.x >> aabb.min.y >> aabb.min.z >> aabb.max.x >> aabb.max.y >> aabb.max.z)
                {
                    // calculation was madness..
                    // Front face
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.min.z);

                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.min.z);

                    // Back face
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.max.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.max.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);

                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.max.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);

                    // Left face
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);

                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.max.z);

                    // Right face
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);

                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.max.z);

                    // Top face
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);

                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.max.y); geometryBuffer.push_back(aabb.max.z);

                    // Bottom face
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.max.z);

                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.min.z);
                    geometryBuffer.push_back(aabb.max.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.max.z);
                    geometryBuffer.push_back(aabb.min.x); geometryBuffer.push_back(aabb.min.y); geometryBuffer.push_back(aabb.max.z);

                    aabb.positionforimgui = aabb.positionforimguiold = glm::vec3(0.f);

                }
                aabb.vertexCount = static_cast<int>(geometryBuffer.size() - vertexcount) / 3;
                aabbModels.push_back(aabb);
            }
            break;
            case Assignment1::AssignmentScene::Ray:
            {
                RayModel ray;
                if (iss >> ray.origin.x >> ray.origin.y >> ray.origin.z
                    >> ray.direction.x >> ray.direction.y >> ray.direction.z)
                {

                    // Define the desired length of the ray segment
                    float rayLength = 30.f; // Adjust as needed

                    ray.direction = glm::normalize(ray.direction);

                    // Calculate the second point of the ray
                    glm::vec3 rayend = ray.origin + ray.direction * rayLength;

                    // Add the ray's geometry to the buffer
                    geometryBuffer.push_back(ray.origin.x);
                    geometryBuffer.push_back(ray.origin.y);
                    geometryBuffer.push_back(ray.origin.z);
                    geometryBuffer.push_back(rayend.x);
                    geometryBuffer.push_back(rayend.y);
                    geometryBuffer.push_back(rayend.z);
                }
                ray.vertexCount = static_cast<int>(geometryBuffer.size() - vertexcount) / 3;
                rayModels.push_back(ray);
            }
            break;
        default:
            break;
        }
        return true;
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::LoadScene()
    @return bool

    Mesh is loaded in prior to execution of following,
    Loads scene from predefined location and set respective model's 
    position.
    returns false if failure to locate/open file
    */
    bool AssignmentScene::LoadScene()
    {
        std::ifstream file("../Common/Ass1/ass1.scn");
        if (!file)
        {
            std::cout << "Failed to open file: " << "ass1.scn" << std::endl;
            return false;
        }

        std::string line;
        std::string modelType;
        int index;

        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            if (iss >> modelType)
            {
                if (modelType == "Point")
                {
                    LoadMesh(Point);

                    glm::vec3 position;
                    if (iss >> index >> position.x >> position.y >> position.z)
                    {
                    
                        pointModels[index].position = position;
                    }

                }
                else if (modelType == "Plane")
                {
                    LoadMesh(Plane);
                    glm::vec3 planeepos;
                    if (iss >> index >> planeepos.x >> planeepos.y >> planeepos.z)
                    {
                       // in order to calculate new vertices for intersection test
                        planeModels[index].positionforimgui = planeepos;
                        glm::vec3 translationVector = planeModels[index].positionforimgui - planeModels[index].positionforimguiold;
                        planeModels[index].positionforimguiold = planeepos;
                        planeModels[index].P0 += translationVector;
                        planeModels[index].P1 += translationVector;
                        planeModels[index].P2 += translationVector;
                    }
                }
                else if (modelType == "Triangle")
                {
                    LoadMesh(Triangle);
                    glm::vec3 trianglepos;
                    if (iss >> index >> trianglepos.x >> trianglepos.y >> trianglepos.z)
                    {
                        // in order to calculate new vertices for intersection test
                        triangleModels[index].positionforimgui = trianglepos;
                        glm::vec3 translationVector = triangleModels[index].positionforimgui - triangleModels[index].positionforimguiold;
                        triangleModels[index].positionforimguiold = trianglepos;
                        triangleModels[index].v0 += translationVector;
                        triangleModels[index].v1 += translationVector;
                        triangleModels[index].v2 += translationVector;
                    }
                }
                else if (modelType == "BoundingSphere")
                {
                    LoadMesh(BoundingSphere);
                    BoundingSphereModel sphere;
                    if (iss >> index >> sphere.position.x >> sphere.position.y >> sphere.position.z)
                    {
                        boundingSphereModels[index].position = sphere.position;                   
                    }
                }
                else if (modelType == "AABB")
                {
                    LoadMesh(AABB);
                    glm::vec3 aabbposition;
                    if (iss >> index >> aabbposition.x >> aabbposition.y >> aabbposition.z)
                    {

                        // in order to calculate new vertices for intersection test
                        aabbModels[index].positionforimgui = aabbposition;
                        glm::vec3 translationVector = aabbModels[index].positionforimgui - aabbModels[index].positionforimguiold;
                        aabbModels[index].positionforimguiold = aabbposition;
                        aabbModels[index].min += translationVector;
                        aabbModels[index].max += translationVector;

                    }
                }
                else if (modelType == "Ray")
                {
                    LoadMesh(Ray);
                    RayModel ray;
                    int index;
                    if (iss >> index >> ray.origin.x >> ray.origin.y >> ray.origin.z)
                    {
                        rayModels[index].origin = ray.origin;
                    }
                }
            }
        }

        return true;
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::Init()
    @return int

    Initialize the scene by loading shaders, geometry, Imgui and camera projection
    */
    int AssignmentScene::Init()
    {
        // Create and compile our GLSL program from the shaders
        programID = LoadShaders("../Common/shaders/Ass1.vert",
                                "../Common/shaders/Ass1.frag");

        LoadScene();

        SetupBuffers();

        SetupNanoGUI(currentWindow);

        InitCameras();

        return Scene::Init();
    }


    /*  _________________________________________________________________________ */
    /*! AssignmentScene::InitCameras()
    @return none

    Initialize 2 types of camera:
    1. Perspective camera angled at 45 degree (default)
    2. Perspective camera front view 
    */
    void AssignmentScene::InitCameras()
    {
        Camera ca1;
        // Camera parameters normal
        ca1.cameraPosition = glm::vec3(0.0f, 0.0f, 20.0f); // Camera position in world space
        ca1.cameraTarget = glm::vec3(0.0f, 0.0f, -1.0f);  // Target point the camera is looking at
        ca1.cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);  // Up vector for camera orientation

        Cameras.push_back(ca1);

        for (Camera& ca : Cameras)
        {
            // Field of view and aspect ratio
            ca.fov = 45.f;                      // Field of view (in degrees)
            ca.aspectRatio = static_cast<float>(WindowWidth) / static_cast<float>(WindowHeight);     // Aspect ratio of the window

            // Near and far clipping planes
            ca.nearPlane = 1.f;
            ca.farPlane = 100.0f;

            // Calculate the view matrix
            ca.viewMatrix = glm::lookAt(ca.cameraPosition, ca.cameraTarget, ca.cameraUp);

            // Calculate the projection matrix
            ca.projectionMatrix = glm::perspective(glm::radians(ca.fov), ca.aspectRatio, ca.nearPlane, ca.farPlane);

            // Combine the view and projection matrices for the final MVP matrix
            ca.mvpMatrix = ca.projectionMatrix * ca.viewMatrix;
        }

    }
    float fov = 45.f;
    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
    {
        fov -= (float)yoffset;

        if (fov < 1.0f)
            fov = 1.0f;
        if (fov > 45.0f)
            fov = 45.0f;
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::Render()
    @return int

    Renders the scene including Imgui and enables back-face culling
    */
    int AssignmentScene::Render()
    {

        glClearColor(0.0f, 0.f, 0.f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);

        glUseProgram(programID);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        // Uniform matrix
        GLint vTransformLoc = glGetUniformLocation(programID, "vertexTransform");
        GLint mvpMatrixLoc = glGetUniformLocation(programID, "mvpMatrixLoc");
    
        // Enable or disable wireframe rendering based on the checkbox state
        enableWireframe ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        int numdrawn = 0;
        glm::vec3 rotateVector = glm::vec3(1.0f);

        if (PointIntersectionRender || RayIntersectionRender)
        {
            //Render each triangle models
            for (const TriangleModel& triangle : triangleModels)
            {
                // Set the model matrix for the triangle
                glm::mat4 modelMatrix = glm::mat4(1.0f);
                glm::vec3 scaleVector = glm::vec3(1.f);

                modelMatrix =
                    glm::translate(modelMatrix, triangle.positionforimgui)
                    * glm::rotate(angleOfRotation, rotateVector)
                    * glm::scale(scaleVector);
                glUniformMatrix4fv(vTransformLoc, 1, GL_FALSE, &modelMatrix[0][0]);
                glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &Cameras[selectedCameraOption].mvpMatrix[0][0]);
                // Render the triangle

                glDrawArrays(GL_TRIANGLES, numdrawn, triangle.vertexCount);
                numdrawn += triangle.vertexCount;
            }
        }
        else
            numdrawn += triangleModels[0].vertexCount * triangleModels.size();

        if (PointIntersectionRender)
        {

            // Render the point models
            for (const PointModel& point : pointModels)
            {
                // Set the model matrix for each point model
                glm::mat4 modelMatrix = glm::mat4(1.0f);
                glm::vec3 scaleVector = glm::vec3(1.f);

                modelMatrix =
                    glm::translate(modelMatrix, glm::vec3(point.position.x, point.position.y, point.position.z))
                    * glm::rotate(angleOfRotation, rotateVector)
                    * glm::scale(scaleVector)
                    ;
                glUniformMatrix4fv(vTransformLoc, 1, GL_FALSE, &modelMatrix[0][0]);
                glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &Cameras[selectedCameraOption].mvpMatrix[0][0]);
                glPointSize(5.0f);
                glDrawArrays(GL_POINTS, numdrawn, point.vertexCount);
                numdrawn += point.vertexCount;
            }
        }
        else
            numdrawn += pointModels[0].vertexCount * pointModels.size();

        if (PointIntersectionRender || RayIntersectionRender || PlaneIntersectionRender)
        {
            // Render each plane model individually
            for (const auto& plane : planeModels)
            {
                // Set the model matrix for the current model
                glm::mat4 modelMatrix = glm::mat4(1.0f);
                glm::vec3 scaleVector = glm::vec3(1.f);

                modelMatrix =
                    glm::translate(modelMatrix, plane.positionforimgui)
                    * glm::rotate(angleOfRotation, rotateVector)
                    * glm::scale(scaleVector);
                glUniformMatrix4fv(vTransformLoc, 1, GL_FALSE, &modelMatrix[0][0]);
                glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &Cameras[selectedCameraOption].mvpMatrix[0][0]);

                // Draw the geometry
                glDrawArrays(GL_TRIANGLES, numdrawn, plane.vertexCount);
                numdrawn += plane.vertexCount;
            }
        }
        else
            numdrawn += planeModels[0].vertexCount * planeModels.size();

        if (RayIntersectionRender)
        {
            // Render the ray model
            for (const RayModel& ray : rayModels)
            {
                // Set the model matrix for the ray model
                glm::mat4 modelMatrix = glm::mat4(1.0f);
                glm::vec3 scaleVector = glm::vec3(1.f);

                modelMatrix = glm::scale(scaleVector) *
                    glm::rotate(angleOfRotation, rotateVector) *
                    glm::translate(modelMatrix, glm::vec3(ray.origin.x, ray.origin.y, ray.origin.z));
                glUniformMatrix4fv(vTransformLoc, 1, GL_FALSE, &modelMatrix[0][0]);
                glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &Cameras[selectedCameraOption].mvpMatrix[0][0]);

                // Draw the ray geometry
                glDrawArrays(GL_LINES, numdrawn, ray.vertexCount); // Assuming the ray is represented by a line segment
                numdrawn += ray.vertexCount;
                std::cout << ray.vertexCount;
            }
        }
        else
            numdrawn += rayModels[0].vertexCount * rayModels.size();

        if (BasicIntersectionRender || PointIntersectionRender || PlaneIntersectionRender || RayIntersectionRender)
        {

            //Render each plane sphere individually
            for (const auto& sphere : boundingSphereModels)
            {
                // Set the model matrix for the current model
                glm::mat4 modelMatrix = glm::mat4(1.0f);
                glm::vec3 scaleVector = glm::vec3(1.f);

                modelMatrix = glm::scale(scaleVector) *
                    glm::rotate(angleOfRotation, rotateVector) *
                    glm::translate(modelMatrix, sphere.position);
                glUniformMatrix4fv(vTransformLoc, 1, GL_FALSE, &modelMatrix[0][0]);
                glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &Cameras[selectedCameraOption].mvpMatrix[0][0]);

                // Draw the geometry
                glDrawArrays(GL_TRIANGLE_FAN, numdrawn, sphere.vertexCount);
                numdrawn += sphere.vertexCount;
            }

            //Render each plane aabb individually
            for (const auto& aabb : aabbModels)
            {
                // Set the model matrix for the current model
                glm::mat4 modelMatrix = glm::mat4(1.0f);
                glm::vec3 scaleVector = glm::vec3(1.f);

                modelMatrix = glm::scale(scaleVector) *
                    glm::rotate(angleOfRotation, rotateVector) *
                    glm::translate(modelMatrix, aabb.positionforimgui);
                glUniformMatrix4fv(vTransformLoc, 1, GL_FALSE, &modelMatrix[0][0]);
                glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &Cameras[selectedCameraOption].mvpMatrix[0][0]);

                // Draw the geometry, the lines across the box is actually per quad the line between them,
                // front and back quad, the line dividing both triangles
                glDrawArrays(GL_TRIANGLES, numdrawn, aabb.vertexCount);
                numdrawn += aabb.vertexCount;
            }
        }
        else
            numdrawn += boundingSphereModels[0].vertexCount * boundingSphereModels.size() 
                        + aabbModels[0].vertexCount * aabbModels.size();


        glDisableVertexAttribArray(0);

        // ImGui drag and drop options
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // all imgui code goes between newframe and renderdraw

        ImGui::Checkbox("Enable Wireframe", &enableWireframe);
        ImGui::Checkbox("Basic Intersection Test", &BasicIntersectionRender);
        ImGui::Checkbox("Point-Based Intersection Test", &PointIntersectionRender);
        ImGui::Checkbox("Ray-Based Intersection Test", &RayIntersectionRender);
        ImGui::Checkbox("Plane-Based Intersection Test", &PlaneIntersectionRender);

        // To change cameras
        ImGui::Combo("Change Camera", &selectedCameraOption, CameraNames, IM_ARRAYSIZE(CameraNames));

        // Transformation controls for each model instance
        if (PointIntersectionRender)
        for (size_t i = 0; i < pointModels.size(); ++i)
        {
            ImGui::Text("Point Transformation Controls %zu:", i);
            // Create a unique ID for each drag control to differentiate them
            // and avoid affecting both points when modified
            std::string positionID = "Point Position##" + std::to_string(i);
            ImGui::DragFloat3(positionID.c_str(), &pointModels[i].position.x);

        }

        if (PointIntersectionRender || RayIntersectionRender || PlaneIntersectionRender)
        for (size_t i = 0; i < planeModels.size(); ++i)
        {
            ImGui::Text("Plane Transformation Controls %zu:", i);
            // Create a unique ID for each drag control to differentiate them
            // and avoid affecting both points when modified
            std::string positionID = "Plane Position##" + std::to_string(i);
            if (ImGui::DragFloat3(positionID.c_str(), &planeModels[i].positionforimgui.x))
            {
                // Update the vertex positions based on the updated position
                glm::vec3 translationVector = planeModels[i].positionforimgui - planeModels[i].positionforimguiold;
                planeModels[i].positionforimguiold = planeModels[i].positionforimgui;
                planeModels[i].P0 += translationVector;
                planeModels[i].P1 += translationVector;
                planeModels[i].P2 += translationVector;
            }
        }

        if (PointIntersectionRender || RayIntersectionRender)
        for (size_t i = 0; i < triangleModels.size(); ++i)
        {
            ImGui::Text("Triangle Transformation Controls %zu:", i);
            // Create a unique ID for each drag control to differentiate them
            // and avoid affecting both points when modified
            std::string positionID = "Triangle Position##" + std::to_string(i);
            if (ImGui::DragFloat3(positionID.c_str(), &triangleModels[i].positionforimgui.x))
            {
                // Update the vertex positions based on the updated position
                glm::vec3 translationVector = triangleModels[i].positionforimgui - triangleModels[i].positionforimguiold;
                triangleModels[i].positionforimguiold = triangleModels[i].positionforimgui;
                triangleModels[i].v0 += translationVector;
                triangleModels[i].v1 += translationVector;
                triangleModels[i].v2 += translationVector;
            }
        }

        if (RayIntersectionRender)
        for (size_t i = 0; i < rayModels.size(); ++i)
        {
            ImGui::Text("Ray Transformation Controls %zu:", i);
            // Create a unique ID for each drag control to differentiate them
            // and avoid affecting both points when modified
            std::string positionID = "Ray Position##" + std::to_string(i);
            ImGui::DragFloat3(positionID.c_str(), &rayModels[i].origin.x);
        }

        if (BasicIntersectionRender || PointIntersectionRender || PlaneIntersectionRender || RayIntersectionRender)
        {
            for (size_t i = 0; i < boundingSphereModels.size(); ++i)
            {
                ImGui::Text("Sphere Transformation Controls %zu:", i);
                // Create a unique ID for each drag control to differentiate them
                // and avoid affecting both points when modified
                std::string positionID = "Sphere Position##" + std::to_string(i);
                ImGui::DragFloat3(positionID.c_str(), &boundingSphereModels[i].position.x);
            }
            for (size_t i = 0; i < aabbModels.size(); ++i)
            {

                ImGui::Text("AABB Transformation Controls %zu:", i);

                // Create a unique ID for each drag control to differentiate them
                // and avoid affecting both points when modified
                std::string positionID = "AABB##" + std::to_string(i);

                if (ImGui::DragFloat3(positionID.c_str(), &aabbModels[i].positionforimgui.x))
                {
                    // Update the vertex positions based on the updated position
                    glm::vec3 translationVector = aabbModels[i].positionforimgui - aabbModels[i].positionforimguiold;
                    aabbModels[i].positionforimguiold = aabbModels[i].positionforimgui;
                    aabbModels[i].max += translationVector;
                    aabbModels[i].min += translationVector;
                }
            }
        }

        UpdateCamera();


        // Intersection test
        if (BasicIntersectionRender)
        BasicIntersection();
        if (PointIntersectionRender)
        PointIntersection();
        if (RayIntersectionRender)
        RayIntersection();
        if (PlaneIntersectionRender)
        PlaneIntersection();

        // Render ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        return 0;
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::PointIntersection()
    @return none

    Iterates through all instances of point and their respective 
    intersection test required
    */
    void AssignmentScene::PointIntersection()
    {
        // Point with Plane
        for (size_t i = 0; i < pointModels.size(); ++i)
        {
            // Check for intersection with each plane model
            for (size_t j = 0; j < planeModels.size(); ++j)
            {
                if (Intersection::PointPlane(pointModels[i], planeModels[j]))
                {
                    // Intersection found between point and plane
                    std::string positionID = "Point##" + std::to_string(i);
                    std::string positionID1 = "Plane##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }

            // Check for intersection with each triangle model
            for (size_t j = 0; j < triangleModels.size(); ++j)
            {
                if (Intersection::pointTriangleIntersection(pointModels[i], triangleModels[j]))
                {
                    // Intersection found between point and triangle
                    std::string positionID = "Point##" + std::to_string(i);
                    std::string positionID1 = "Triangle##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }

            // Check for intersection with each sphere model
            for (size_t j = 0; j < boundingSphereModels.size(); ++j)
            {
                if (Intersection::pointSphereIntersection(pointModels[i], boundingSphereModels[j]))
                {
                    // Intersection found between point and triangle
                    std::string positionID = "Point##" + std::to_string(i);
                    std::string positionID1 = "Sphere##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }

            // Check for intersection with each aabb model
            for (size_t j = 0; j < aabbModels.size(); ++j)
            {
                if (Intersection::pointAABBIntersection(pointModels[i], aabbModels[j]))
                {
                    // Intersection found between point and triangle
                    std::string positionID = "Point##" + std::to_string(i);
                    std::string positionID1 = "AABB##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }
        }

    
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::RayIntersection()
    @return none

    Iterates through all instances of ray and their respective
    intersection test required
    */
    void AssignmentScene::RayIntersection()
    {
        // Ray with ...
        for (size_t i = 0; i < rayModels.size(); ++i)
        {
            // Check for intersection with each plane model
            for (size_t j = 0; j < planeModels.size(); ++j)
            {
                // if ray.z <= 0, the whole ray is always intersecting with the plane
                if (Intersection::RayPlaneIntersection(rayModels[i], planeModels[j]).first)
                {
                    // Intersection found between point and plane
                    std::string positionID = "Ray##" + std::to_string(i);
                    std::string positionID1 = "Plane##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }

            // Check for intersection with each triangle model
            for (size_t j = 0; j < triangleModels.size(); ++j)
            {
                // if ray.z <= 0, the whole ray is always intersecting with the plane
                if (Intersection::RayTriangleIntersection(rayModels[i], triangleModels[j]))
                {
                    // Intersection found between point and plane
                    std::string positionID = "Ray##" + std::to_string(i);
                    std::string positionID1 = "Triangle##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }

            // Check for intersection with each sphere model
            for (size_t j = 0; j < boundingSphereModels.size(); ++j)
            {
                if (Intersection::RaySphereIntersection(rayModels[i], boundingSphereModels[j]))
                {
                    // Intersection found between point and plane
                    std::string positionID = "Ray##" + std::to_string(i);
                    std::string positionID1 = "Sphere##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }

            // Check for intersection with each aabb model
            for (size_t j = 0; j < aabbModels.size(); ++j)
            {
                if (Intersection::RayAABBIntersection(rayModels[i], aabbModels[j]))
                {
                    // Intersection found between point and plane
                    std::string positionID = "Ray##" + std::to_string(i);
                    std::string positionID1 = "AABB##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }
        }
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::BasicIntersection()
    @return none

    Iterates through all instances of sphere and aabb box and their respective
    intersection test required
    */
    void AssignmentScene::BasicIntersection()
    {
        // Sphere vs 
        for (size_t i = 0; i < boundingSphereModels.size(); ++i)
        {
            // Check for intersection with each sphere model
            for (size_t j = i+1; j < boundingSphereModels.size(); ++j)
            {
            
                if (Intersection::SphereSphereIntersection(boundingSphereModels[i], boundingSphereModels[j]))
                {
                    // Intersection found between sphere and sphere
                    std::string positionID = "Sphere##" + std::to_string(i);
                    std::string positionID1 = "Sphere##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }
        
            // Check for intersection with each AABB model
            for (size_t j = 0; j < aabbModels.size(); ++j)
            {
                if (Intersection::SphereAABBIntersection(boundingSphereModels[i], aabbModels[j]))
                {
                    // Intersection found between sphere and sphere
                    std::string positionID = "Sphere##" + std::to_string(i);
                    std::string positionID1 = "AABB##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }
        }
        // AABB vs
        for (size_t i = 0; i < aabbModels.size(); ++i)
        {
            // Check for intersection with each AABB model
            for (size_t j = i + 1; j < aabbModels.size(); ++j)
            {
                if (Intersection::AABBAABBIntersection(aabbModels[i], aabbModels[j]))
                {
                    std::string positionID = "AABB##" + std::to_string(i);
                    std::string positionID1 = "AABB##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }
        }
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::PlaneIntersection()
    @return none

    Iterates through all instances plane and their respective
    intersection test required
    */
    void AssignmentScene::PlaneIntersection()
    {
        // Plane vs 
        for (size_t i = 0; i < planeModels.size(); ++i)
        {
            // Check for intersection with each sphere model
            for (size_t j = 0; j < boundingSphereModels.size(); ++j)
            {
                // if ray.z <= 0, the whole ray is always intersecting with the plane
                if (Intersection::PlaneSphereIntersection(planeModels[i], boundingSphereModels[j]))
                {
                    // Intersection found between sphere and sphere
                    std::string positionID = "Plane##" + std::to_string(i);
                    std::string positionID1 = "Sphere##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }

            // Check for intersection with each aabb model
            for (size_t j = 0; j < aabbModels.size(); ++j)
            {
                // if ray.z <= 0, the whole ray is always intersecting with the plane
                if (Intersection::PlaneAABBIntersection(planeModels[i], aabbModels[j]))
                {
                    // Intersection found between sphere and sphere
                    std::string positionID = "Plane##" + std::to_string(i);
                    std::string positionID1 = "AABB##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }
        }
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::postRender()
    @return int

    Updates can be done here
    */
    int AssignmentScene::postRender()
    {
        return 0;
    }

    /*  _________________________________________________________________________ */
    /*! AssignmentScene::MousePicking()
    @return none

    Implementation for mouse picking by ray casting and checking for
    intersection between ray and objects
    */
    void AssignmentScene::MousePicking()
    {

        // Convert cursor position to NDC, [-1,1] [-1,1] [0,1]
        double ndcX = (2.0 * mouse.xPos) / WindowWidth - 1.0;
        double ndcY = 1.0 - (2.0 * mouse.yPos) / WindowHeight;

        // Convert NDC to clip space
        glm::vec4 rayClipSpace(ndcX, ndcY, -1.0, 1.0);

        // Convert clip space to eye space
        glm::mat4 inverseProjectionMatrix = glm::inverse(Cameras[selectedCameraOption].projectionMatrix);
        glm::vec4 rayEyeSpace = inverseProjectionMatrix * rayClipSpace;
        rayEyeSpace = glm::vec4(rayEyeSpace.x, rayEyeSpace.y, -Cameras[selectedCameraOption].nearPlane, 0.0);

        // Convert eye space to world space
        glm::mat4 inverseViewMatrix = glm::inverse(Cameras[selectedCameraOption].viewMatrix);
        glm::vec4 rayWorldSpace = inverseViewMatrix * rayEyeSpace;
        glm::vec3 rayDirection = glm::normalize(glm::vec3(rayWorldSpace));

        RayModel ray;
        ray.origin = Cameras[selectedCameraOption].cameraPosition;
        ray.direction = rayDirection;



        // perform ray vs all objects..
        // Check for intersection with each plane model
        if (PointIntersectionRender || RayIntersectionRender || PlaneIntersectionRender)
        for (size_t j = 0; j < planeModels.size(); ++j)
        {
            // if ray.z <= 0, the whole ray is always intersecting with the plane
            if (Intersection::RayPlaneIntersection(ray, planeModels[j]).first)
            {
                // Intersection found between point and plane
                std::string positionID = "Mouse Picking";
                std::string positionID1 = "Plane##" + std::to_string(j);
                std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                ImGui::Text(Intersecting.c_str());
            }
        }

        // Check for intersection with each triangle model
        if (PointIntersectionRender || RayIntersectionRender)
        for (size_t j = 0; j < triangleModels.size(); ++j)
        {
            // if ray.z <= 0, the whole ray is always intersecting with the plane
            if (Intersection::RayTriangleIntersection(ray, triangleModels[j]))
            {

                // Intersection found between point and plane
                std::string positionID = "Mouse Picking";
                std::string positionID1 = "Triangle##" + std::to_string(j);
                std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                ImGui::Text(Intersecting.c_str());
            }
        }

        if (BasicIntersectionRender || PointIntersectionRender || PlaneIntersectionRender || RayIntersectionRender)
        {
            // Check for intersection with each sphere model
            for (size_t j = 0; j < boundingSphereModels.size(); ++j)
            {
                if (Intersection::RaySphereIntersection(ray, boundingSphereModels[j]))
                {
                    // Intersection found between point and plane
                    std::string positionID = "Mouse Picking";
                    std::string positionID1 = "Sphere##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }

            // Check for intersection with each aabb model
            for (size_t j = 0; j < aabbModels.size(); ++j)
            {
                if (Intersection::RayAABBIntersection(ray, aabbModels[j]))
                {
                    // Intersection found between point and plane
                    std::string positionID = "Mouse Picking";
                    std::string positionID1 = "AABB##" + std::to_string(j);
                    std::string Intersecting = positionID + std::string(" intersecting with ") + positionID1;
                    ImGui::Text(Intersecting.c_str());
                }
            }
        }

    }
    /*  _________________________________________________________________________ */
    /*! AssignmentScene::UpdateCamera()
    @return none

    Implementation for camera movement using keyboard and mouse
    */
    void AssignmentScene::UpdateCamera()
    {
        if (glfwGetMouseButton(currentWindow, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            // Mouse left button is pressed
            // Perform desired actions
            glfwGetCursorPos(currentWindow, &mouse.xPos, &mouse.yPos);
            MousePicking();

        }
        if (glfwGetMouseButton(currentWindow, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
            // Mouse left button is pressed
            // Perform desired actions
            glfwGetCursorPos(currentWindow, &mouse.xPos, &mouse.yPos);

            if (mouse.firsttime)
            {
                mouse.xLast = static_cast<float> (mouse.xPos);
                mouse.yLast = static_cast<float> (mouse.yPos);
                mouse.firsttime = false;
            }

            float xoffset = static_cast<float> (mouse.xPos) - mouse.xLast;
            float yoffset = mouse.yLast - static_cast<float> (mouse.yPos); // reversed since y-coordinates go from bottom to top
            mouse.xLast = static_cast<float> (mouse.xPos);
            mouse.yLast = static_cast<float> (mouse.yPos);

            xoffset *= mouse.mousesensitivity;
            yoffset *= mouse.mousesensitivity;

            mouse.yaw += xoffset;
            mouse.pitch += yoffset;

            glm::vec3 front;
            front.x = cos(glm::radians(mouse.yaw)) * cos(glm::radians(mouse.pitch));
            front.y = sin(glm::radians(mouse.pitch));
            front.z = sin(glm::radians(mouse.yaw)) * cos(glm::radians(mouse.pitch));
            Cameras[selectedCameraOption].cameraTarget = glm::normalize(front);

        }
        if (glfwGetMouseButton(currentWindow, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE)
            mouse.firsttime = true;

        glfwSetScrollCallback(currentWindow, scroll_callback);

        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        float cameraSpeed = mouse.keyboardspeed * deltaTime;
        if (glfwGetKey(currentWindow, GLFW_KEY_W) == GLFW_PRESS)
            Cameras[selectedCameraOption].cameraPosition += cameraSpeed * Cameras[selectedCameraOption].cameraTarget;
        if (glfwGetKey(currentWindow, GLFW_KEY_S) == GLFW_PRESS)
            Cameras[selectedCameraOption].cameraPosition -= cameraSpeed * Cameras[selectedCameraOption].cameraTarget;
        if (glfwGetKey(currentWindow, GLFW_KEY_A) == GLFW_PRESS)
            Cameras[selectedCameraOption].cameraPosition -= glm::normalize(glm::cross(Cameras[selectedCameraOption].cameraTarget, Cameras[selectedCameraOption].cameraUp)) * cameraSpeed;
        if (glfwGetKey(currentWindow, GLFW_KEY_D) == GLFW_PRESS)
            Cameras[selectedCameraOption].cameraPosition += glm::normalize(glm::cross(Cameras[selectedCameraOption].cameraTarget, Cameras[selectedCameraOption].cameraUp)) * cameraSpeed;

        // update camera 
        // Calculate the projection matrix
        // Calculate the view matrix
        Cameras[selectedCameraOption].viewMatrix = 
            glm::lookAt(Cameras[selectedCameraOption].cameraPosition, Cameras[selectedCameraOption].cameraPosition + Cameras[selectedCameraOption].cameraTarget, Cameras[selectedCameraOption].cameraUp);

        Cameras[selectedCameraOption].fov = fov;
        Cameras[selectedCameraOption].projectionMatrix = 
            glm::perspective(glm::radians(Cameras[selectedCameraOption].fov), Cameras[selectedCameraOption].aspectRatio, Cameras[selectedCameraOption].nearPlane, Cameras[selectedCameraOption].farPlane);

        // Combine the view and projection matrices for the final MVP matrix
        Cameras[selectedCameraOption].mvpMatrix = 
            Cameras[selectedCameraOption].projectionMatrix * Cameras[selectedCameraOption].viewMatrix;

    }


}