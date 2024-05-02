/* !
@file    AssignmentScene.h
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    16/07/2023

This file contains the declaration of namespace Assignment 4 that encapsulates
the functionality required to implement for assignment 4
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

namespace Assignment4 {

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
            int level;
            //OBB
            glm::vec3 center;
            glm::vec3 halfExtents;
            glm::vec3 PA,Y,Z;
        };

        struct Sphere {
            glm::vec3 min;
            glm::vec3 max;
            glm::vec3 center;
            float radius;
        };

        class Mesh {
        public:
            Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
            void cleanup();
            void SetupBuffers(const GLuint ID);
            void RenderMesh(Camera& cam, const glm::vec3& position, const glm::vec3& rot,glm::vec3 color);
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
            PCA = 1,
            OBB = 2
        };

        class BoundingVolumes {
        public:
            void LoadVolumes(BV, std::vector<glm::vec3>);
            glm::vec3 position;
            void GenerateAABB(std::vector<glm::vec3>);

            void GeneratePCASphere(std::vector<glm::vec3>);
            void GenerateOBB(std::vector<glm::vec3>);

            void SetupBuffersBV(BV, const GLuint ID);
            void RenderMeshBV(Camera& cam, const glm::vec3& position,bool obb, const glm::vec3& angle);
            void RenderMeshBV(Camera& cam, const glm::vec3& position,glm::vec3 color);

            //for buffer
            std::vector<glm::vec3> vertices;
            std::vector<unsigned int> indices;
            GLuint vao, vbo, ebo, shaderID;
            glm::mat4 ModeltoWorld;
            glm::mat4 MVP;
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
            int selectedBVOption = 0;

        };
        /////////////////////////////////////////////////////////////////////////////////////////////////

        enum type
        {
            Internal = 0,
            leaf = 1
        };

        struct Node {
            glm::vec3 center;
            type nodetype;
            glm::vec3 halfWidth;
            std::vector<Node*> pChild;
            std::vector<Model> objectist;
            BoundingVolumes box;
            int terminationcond = 10;
        };

        // Octree
        void buildOctree(Node* currentnode, int cond);
        void constructAABB(BoundingVolumes &box);
        void TraversePrint(Node* node, const int cam, bool BB, bool oct);
        void Traversedelete(Node* node);
        void RandomColor(int size, std::vector<glm::vec3>& colorvct);

        //KD-Tree
        void buildKDTree(Node* currentnode, int depth, int cond, int k = 0);

        // AABB
        int SplitAxisByMinVolumeAABB(std::vector<AABBC>&);
        int partitionbyMedianBVCAABB(std::vector<Model>&);
        float calculateTotalVolumeAABB(const std::vector<AABBC>& aabbList);
        BoundingVolumes* ComputeBVAABB(const std::vector<Model> objects);

        // Sphere
        int SplitAxisByMinVolumePCA(std::vector<Sphere>&);
        int partitionbyMedianBVCPCA(std::vector<Model>&);
        float calculateTotalVolumePCA(const std::vector<Sphere>& pcalist);
        BoundingVolumes* ComputeBVSphere(std::vector<Model> objects);

    private:

        // member functions
        void initMembers();

        void SetupNanoGUI(GLFWwindow* pWwindow) override;

        void LoadScene();


        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Assimp

        std::vector<Model> models;
        std::vector<std::string> modelnames; //for imgui
        int selectedModelOption = 0;
        const char* BVnames[3] = { "AABB","PCA","OBB"}; //for imgui

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


        std::vector<Node*> rootnodeOctree;
        std::vector<Node*> rootnodeKDTree;

        std::vector<glm::vec3> colorpickerOct;
        std::vector<glm::vec3> colorpickerKD;

        int ksplits = 10;

        int straddlingOct = 0;
        int straddlingKD = 0;

        const char* straddleOct[2] = { "w.r.t center", "Associate with current level" };
        const char* straddleKD[2] = { "median of BV center", "median of BV extent"/*, "k-even splits"*/};

        int boxID;

        // imgui checkbox
        bool enableWireframeModel = false;
        bool enableWireframeBV = false;
        bool enableBV = false;
        bool enableOctree = false;
        bool enableOctreeBV = false;

        bool enableKDtree = false;


        // imgui camera swap
        const char* CameraNames[2] = { "Perspective Front", "Perspective Top" };
        int selectedCameraOption = 0;
        float deltaTime = 0.0f;	// Time between current frame and last frame
        float lastFrame = 0.0f; // Time of last frame


    };
}
