/* !
@file    AssignmentScene.h
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    10/06/2023

This file contains the declaration of namespace Assignment 2 that encapsulates
the functionality required to implement for assignment 2
*//*__________________________________________________________________________*/

#pragma once
#include "../Common/Scene.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <vector>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace Assignment2 {

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

        struct BoundingSphereModel
        {
            glm::vec3 position;
            float radius;
        };

        struct Camera
        {
            glm::vec3 cameraPosition; // Camera position in world space
            glm::vec3 cameraFront;   // Target point the camera is looking at
            glm::vec3 cameraUp;       // Up vector for camera orientation
            glm::vec3 cameraRight;
            
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

            // frustum
            std::vector<glm::vec4> frustrum;
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

        


        /////////////////////////////////////////////////////////////////////////////////////////////////
       // Assimp
        struct Vertex {
            glm::vec3 position;
            glm::vec3 normal;
            glm::vec2 texCoords;
        };

        struct PlaneModel
        {
            glm::vec3 Normal;
            float distance;
        };

        struct AABBC {
        public:
            glm::vec3 min;
            glm::vec3 max;

            //OBB
            glm::vec3 center;
            glm::vec3 halfExtents;
            glm::vec3 PA,Y,Z;
        };

        struct Sphere {
        public:
            glm::vec3 min;
            glm::vec3 max;
            glm::vec3 center;
            std::vector<glm::vec3> Normal;
            float radius;
            void normalL();

        };

        class Mesh {
        public:
            Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
            void cleanup();
            void SetupBuffers(const GLuint ID);
            void RenderMesh(Camera& cam, const glm::vec3& position, const glm::vec3& rot);
            std::vector<Vertex> Getvertices() { return vertices; }
        private:
            std::vector<Vertex> vertices;
            std::vector<unsigned int> indices;
            GLuint vao, vbo, ebo, shaderID;
            glm::mat4 ModeltoWorld;
            glm::mat4 MVP;
            glm::vec3 intersectionColour;

        };

        enum BV {
            AABB = 0,
            Ritter = 1,
            Larsson = 2,
            PCA = 3,
            OBB = 4
        };

        class BoundingVolumes {
        public:
            void LoadVolumes(BV, std::vector<glm::vec3>);
            glm::vec3 position;
            void GenerateAABB(std::vector<glm::vec3>);
            void GenerateRitterSphere(std::vector<glm::vec3>);
            void GenerateLarssonSphere(std::vector<glm::vec3>);
            void GeneratePCASphere(std::vector<glm::vec3>);
            void GenerateOBB(std::vector<glm::vec3>);

            void SetupBuffersBV(BV, const GLuint ID);
            void RenderMeshBV(Camera& cam, const glm::vec3& position,bool obb, const glm::vec3& angle);

            //for buffer
            std::vector<glm::vec3> vertices;
            std::vector<unsigned int> indices;
            GLuint vao, vbo, ebo, shaderID;
            glm::vec3 intersectionColour;
            glm::mat4 ModeltoWorld;
            glm::mat4 MVP;

            Sphere sphereR;
            Sphere sphereL;
            Sphere spherePCA;

            AABBC aabbbox;
            AABBC OBB;

        };

        class Model {
        public:
            void LoadModel(const std::string& filePath);
            void ProcessMesh(aiMesh* aiMesh, const aiScene* scene, std::vector<Vertex>& vertices, std::vector<unsigned int>& indices);
            void ProcessNode(aiNode* node, const aiScene* scene, std::vector<Mesh>& meshes);
            std::vector<BoundingVolumes> BVs;
            std::vector<Mesh> mesh;
            glm::vec3 position;
            glm::vec3 Rotangle;

        };
        /////////////////////////////////////////////////////////////////////////////////////////////////

        
    private:

        // member functions
        void initMembers();

        void SetupNanoGUI(GLFWwindow* pWwindow) override;

        void LoadScene();

        glm::vec4 calculatePlaneEquation(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3);

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Assimp

        std::vector<Model> models;
        std::vector<std::string> modelnames; //for imgui
        int selectedModelOption = 0;
        const char* BVnames[5] = { "AABB", "Ritter","Larsson","PCA","OBB"}; //for imgui
        int selectedBVOption = 0;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        
        // Camera
        std::vector<Camera> Cameras;
        // Mouse
        Mouse mouse;
        void InitCameras();
        void UpdateCamera();
        // data members
        GLFWwindow* currentWindow;
        int WindowHeight;
        int WindowWidth;
        GLuint  programID;

        // imgui checkbox
        bool enableWireframeModel = false;
        bool enableWireframeBV = false;

        // imgui camera swap
        const char* CameraNames[2] = { "Perspective Front", "Perspective Top" };
        int selectedCameraOption = 0;
        float deltaTime = 0.0f;	// Time between current frame and last frame
        float lastFrame = 0.0f; // Time of last frame


    };
}
