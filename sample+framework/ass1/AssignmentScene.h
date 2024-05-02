/* !
@file    AssignmentScene.h
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    20/05/2023

This file contains the declaration of namespace Assignment 1 that encapsulates
the functionality required to implement for assignment 1
*//*__________________________________________________________________________*/

#pragma once
#include "../Common/Scene.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <vector>
namespace Assignment1 {

    class AssignmentScene : public Scene
    {

    public:
        AssignmentScene() = default;
        AssignmentScene(int windowWidth, int windowHeight, GLFWwindow* pWwindow, int Wheight, int Wwidth);
        virtual ~AssignmentScene();


    public:
        int Init() override;
        void CleanUp() override;

        int Render() override;
        int postRender() override;

        struct PointModel
        {
            // x,y,z
            glm::vec3 position;
            int vertexCount;

        };

        struct PlaneModel
        {
            glm::vec3 P0;
            glm::vec3 P1;
            glm::vec3 P2;
            //n(x,y,z,P0) 
            glm::vec4 data;
            glm::vec3 positionforimgui;
            glm::vec3 positionforimguiold;
            int vertexCount;

        };

        struct TriangleModel
        {
            glm::vec3 v0;
            glm::vec3 v1;
            glm::vec3 v2;
            glm::vec3 positionforimgui;
            glm::vec3 positionforimguiold;
            int vertexCount;

        };

        struct BoundingSphereModel
        {
            glm::vec3 position;
            float radius;
            // for generating the vertices of the sphere
            int sectorCount;
            int stackCount;
            int vertexCount;
        };

        struct AABBModel
        {
            glm::vec3 min;
            glm::vec3 max;
            // for translating the position
            glm::vec3 positionforimgui;
            glm::vec3 positionforimguiold;
            int vertexCount;

        };

        struct RayModel
        {
            // P(t) = S + dt
            glm::vec3 origin;
            glm::vec3 direction;
            int vertexCount;

        };

        struct Camera
        {
            glm::vec3 cameraPosition; // Camera position in world space
            glm::vec3 cameraTarget;   // Target point the camera is looking at
            glm::vec3 cameraUp;       // Up vector for camera orientation

            // Field of view
            float fov;                  
            float aspectRatio;
            // Near and far clipping planes
            float nearPlane;
            float farPlane;

            // Calculate the view matrix
            glm::mat4 viewMatrix;

            // Calculate the projection matrix
            glm::mat4 projectionMatrix;

            // final matrix from model-view-projection
            glm::mat4 mvpMatrix;
        };

        struct Mouse
        {
            double xPos, yPos;

            // for mouse lookaround
            float xLast, yLast;
            float yaw = -90.f; // looking left and right, -90 to point towards -z axis
            float pitch = 0.f; // looking up and down
            float mousesensitivity = 0.05f;
            bool firsttime = true;
            float keyboardspeed = 2.5f;
        };

        enum Primitives
        {
            Point = 1,
            Plane = 2,
            Triangle = 3,
            BoundingSphere = 4,
            AABB = 5,
            Ray = 6

        };

    private:

        // member functions
        void initMembers();

        // This is the non-software engineered portion of the code
        // Go ahead and modularize the VAO and VBO setup into
        // BufferManagers and ShaderManagers
        void SetupBuffers();

        void SetupNanoGUI(GLFWwindow* pWwindow) override;

        bool LoadScene();
        bool LoadMesh(Primitives);

        void InitCameras();

        void PointIntersection();
        void RayIntersection();
        void BasicIntersection();
        void PlaneIntersection();
        void MousePicking();
        void UpdateCamera();

        // store instances of each model
        std::vector<PointModel> pointModels;
        std::vector<PlaneModel> planeModels;
        std::vector<TriangleModel> triangleModels;
        std::vector<BoundingSphereModel> boundingSphereModels;
        std::vector<AABBModel> aabbModels;
        std::vector<RayModel> rayModels;
        // Camera
        std::vector<Camera> Cameras;
        
        // Mouse
        Mouse mouse;

        // data members
        GLFWwindow* currentWindow;
        int WindowHeight;
        int WindowWidth;
        GLuint  programID;
        GLuint  vertexbuffer;
        GLuint  VertexArrayID;

        std::vector<GLfloat>    geometryBuffer;
        GLfloat   angleOfRotation = 0.f;

        // imgui checkbox
        bool enableWireframe = false;           // to enable wireframe through imgui
        bool BasicIntersectionRender = false;   // to enable drawing of related models only
        bool PointIntersectionRender = false;   // to enable drawing of related models only
        bool RayIntersectionRender = false;     // to enable drawing of related models only
        bool PlaneIntersectionRender = false;   // to enable drawing of related models only
        // imgui camera swap
        const char* CameraNames[1] = { "Perspective Front" };// "Perspective Front 45 Degrees", "Perspective Front", "Perspective Top"};
        int selectedCameraOption = 0;
        float deltaTime = 0.0f;	// Time between current frame and last frame
        float lastFrame = 0.0f; // Time of last frame
    };
}
