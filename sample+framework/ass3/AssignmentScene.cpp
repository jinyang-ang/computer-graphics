/*!
@file    AssignmentScene.cpp
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    28/06/2023

This file implements functionality useful and necessary for assignment 3,
load the model, create the bounding volume and perform the respective
intersection test.

BVH construction of both top-down and bottom-up is also implemented here

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
#include <Eigen/Dense>
#include <numeric> 
#include <QuickHull.hpp>
# define PI 3.14159265358979323846  /* pi */

namespace Assignment3 {

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
		programID(0)
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
		// Top-down BVH
		for (auto& x : TopDownRootAABB)
			TraverseTreedelete(x);
		for (auto& x : TopDownRootSphere)
			TraverseTreedelete(x);

		// Bottom-up BVH
		BottomUpRootAABB.clear();
		BottomUpRootSphere.clear();
		TraverseTreedelete(BottomUpRootAABB_NODE);
		TraverseTreedelete(BottomUpRootSphere_NODE);



		for (auto& x : models)
			for (auto& y : x.mesh)
			{
				y.cleanup();
			}

		glDeleteProgram(programID);

		// Cleanup
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
	}

	/*  _________________________________________________________________________ */
	/*! AssignmentScene::LoadScene
	@param none
	@return none

	Load the models and their Bounding volumes from the specified filepath
	*/
	void AssignmentScene::LoadScene()
	{
		std::ifstream file("../Common/Ass3/ass3.scn");
		if (!file)
		{
			std::cout << "Failed to open file: " << "ass2.scn" << std::endl;
		}
		std::string line;
		std::string modelType;
		glm::vec3 positionxyz;

		while (std::getline(file, line))
		{
			std::istringstream iss(line);
			if (iss >> modelType >> positionxyz.x >> positionxyz.y >> positionxyz.z)
			{
				Model md;
				md.LoadModel("../Common/models/" + modelType + ".obj");
				for (size_t s = 0; s < md.mesh.size(); ++s)
					md.mesh[s].SetupBuffers(programID);
				md.position = positionxyz;
				md.Rotangle = glm::vec3(0.f);
				//only position is required
				std::vector<glm::vec3> pos;
				for (size_t s = 0; s < md.mesh.size(); ++s)
					for (auto& x : md.mesh[s].Getvertices())
						pos.push_back(x.position);

				// load bounding volume for each model currently testing one by one
				for (int i = AABB; i <= OBB; i++) {
					BoundingVolumes bv;
					bv.position = md.position;
					bv.LoadVolumes(static_cast<BV>(i), pos);
					bv.SetupBuffersBV(static_cast<BV>(i), programID);
					md.BVs.push_back(bv);
				}
				models.push_back(md);
				modelnames.push_back(modelType);
			}
		}

		// Top down BVH
		for (int i = 0; i < 3; ++i)
		{
			TopDownRootAABB.push_back(new TreeNode);
			TopDownTree(TopDownRootAABB[i], models, AABB, static_cast<int>(models.size()), 0,i);
			TopDownRootSphere.push_back(new TreeNode);
			TopDownTree(TopDownRootSphere[i], models, PCA, static_cast<int>(models.size()), 0, i);
		}
		// Bottom up BVH

		for (size_t i = 0; i < models.size(); ++i)
		{
			BottomUpRootAABB.push_back(new TreeNode);
			BoundingVolumes* box = new BoundingVolumes;
			box->aabbbox = models[i].BVs[0].aabbbox;
			box->aabbbox.center += models[i].position;
			box->position = models[i].position;
			BottomUpRootAABB[i]->nodetype = type::leaf;
			BottomUpRootAABB[i]->BVlevel = 0;
			BottomUpRootAABB[i]->BV = box;
			BottomUpRootAABB[i]->BV->SetupBuffersBV(AssignmentScene::AABB, AssignmentScene::programID);


			BottomUpRootSphere.push_back(new TreeNode);
			BoundingVolumes* sphere = new BoundingVolumes;
			sphere->spherePCA = models[i].BVs[1].spherePCA;
			sphere->spherePCA.center += models[i].position;
			sphere->position = models[i].position;
			BottomUpRootSphere[i]->nodetype = type::leaf;
			BottomUpRootSphere[i]->BVlevel = 0;
			BottomUpRootSphere[i]->BV = sphere;
			BottomUpRootSphere[i]->BV->SetupBuffersBV(AssignmentScene::PCA, AssignmentScene::programID);


		}
		BottomUpRootAABB_NODE = BottomUpTree(BottomUpRootAABB,AABB);
		BottomUpRootSphere_NODE = BottomUpTree(BottomUpRootSphere, PCA);

	}

	/*  _________________________________________________________________________ */
	/*! AssignmentScene::Mesh::cleanup
	@param none
	@return none

	Cleanup for mesh objects
	*/
	void AssignmentScene::Mesh::cleanup()
	{
		// Cleanup VBO
		// Clean up buffers and arrays
		glDeleteBuffers(1, &ebo);
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
	}
	/*  _________________________________________________________________________ */
	/*! AssignmentScene::SetupBuffers()
	@return none

	Setting up buffers to be sent to vertex shader
	*/
	void AssignmentScene::Mesh::SetupBuffers(const GLuint ID)
	{
		shaderID = ID;

		// Generate and bind vertex array object (VAO)
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		// Generate and bind vertex buffer object (VBO)
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

		// Enable vertex attributes and specify their layout
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));

		// Generate and bind element buffer object (EBO)
		glGenBuffers(1, &ebo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

		return;
	}

	/*  _________________________________________________________________________ */
	/*! AssignmentScene::Mesh::Mesh
	@param none
	@return none

	Construsctor for mesh objects
	*/
	AssignmentScene::Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices)
		:vertices(vertices), indices(indices), vao(0), vbo(0), ebo(0), shaderID(0), ModeltoWorld(0.f), MVP(0.f)
		, intersectionColour(0.5f, 0.5f, 0.5f) {}


	/*  _________________________________________________________________________ */
	/*! AssignmentScene::Model::LoadModel
	@param const std::string& filePath
	@return none

	Read in the model using assimp and process through each node,
	process mesh in each node and store the vertices
	*/
	void AssignmentScene::Model::LoadModel(const std::string& filePath)
	{
		Assimp::Importer importer;
		const aiScene* scene = importer.ReadFile(filePath, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs);

		if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
			std::cout << "Error loading model: " << importer.GetErrorString() << std::endl;
		}
		ProcessNode(scene->mRootNode, scene, mesh);
	}
	/*  _________________________________________________________________________ */
	/*! AssignmentScene::Model::ProcessNode
	@param aiNode* node
	@param const aiScene* scene
	@param std::vector<Mesh>& meshes
	@return none

	Process the children of each node and process the children's
	respective node to extract the vertices and indices
	*/
	void AssignmentScene::Model::ProcessNode(aiNode* node, const aiScene* scene, std::vector<Mesh>& meshes) {
		// Process all the node's meshes (if any)
		for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
			aiMesh* aiMesh = scene->mMeshes[node->mMeshes[i]];
			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;
			ProcessMesh(aiMesh, scene, vertices, indices);
			meshes.push_back(Mesh(vertices, indices));
		}

		// Process all the node's children (if any)
		for (unsigned int i = 0; i < node->mNumChildren; ++i) {
			ProcessNode(node->mChildren[i], scene, meshes);
		}
	}

	/*  _________________________________________________________________________ */
	/*! AssignmentScene::Model::ProcessMesh
	@param aiNode* node
	@param const aiScene* scene
	@param std::vector<Vertex>& vertices
	@param std::vector<unsigned int>& indices

	@return none
	Process through the mesh and store the information, vertices mesh, tex coord etc
	*/
	void AssignmentScene::Model::ProcessMesh(aiMesh* aiMesh, const aiScene* scene, std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) {
		// Process vertices
		for (unsigned int i = 0; i < aiMesh->mNumVertices; ++i) {
			Vertex vertex;
			vertex.position = glm::vec3(aiMesh->mVertices[i].x, aiMesh->mVertices[i].y, aiMesh->mVertices[i].z);
			vertex.normal = glm::vec3(aiMesh->mNormals[i].x, aiMesh->mNormals[i].y, aiMesh->mNormals[i].z);

			if (aiMesh->mTextureCoords[0]) {
				vertex.texCoords = glm::vec2(aiMesh->mTextureCoords[0][i].x, aiMesh->mTextureCoords[0][i].y);
			}
			else {
				vertex.texCoords = glm::vec2(0.0f);
			}
			vertices.push_back(vertex);
		}

		// Process indices
		for (unsigned int i = 0; i < aiMesh->mNumFaces; ++i) {
			aiFace face = aiMesh->mFaces[i];
			for (unsigned int j = 0; j < face.mNumIndices; ++j) {
				indices.push_back(face.mIndices[j]);
			}
		}
	}

	/*  _________________________________________________________________________ */
	/*! AssignmentScene::Mesh::RenderMesh
	@param Camera& cam
	@param const glm::vec3& position
	@param  const glm::vec3& rot

	@return none
	Renders the mesh(model) onto the screen
	*/
	void AssignmentScene::Mesh::RenderMesh(Camera& cam, const glm::vec3& position, const glm::vec3& rot) {

		// Set the MVP matrix for the mesh
		glm::mat4 modelMatrix(1.0f);
		glm::vec3 scaleVector = glm::vec3(1);
		ModeltoWorld =
			glm::translate(modelMatrix, position)
			* glm::rotate(glm::radians(rot.x), glm::vec3(1.0f, 0.0f, 0.0f))
			* glm::rotate(glm::radians(rot.y), glm::vec3(0.0f, 1.0f, 0.0f))
			* glm::rotate(glm::radians(rot.z), glm::vec3(0.0f, 0.0f, 1.0f))
			* glm::scale(scaleVector);

		glUseProgram(shaderID);

		// Set the uniform matrix locations
		GLint mvpMatrixLoc = glGetUniformLocation(shaderID, "mvpMatrixLoc");
		GLint fragmentColorLocation = glGetUniformLocation(shaderID, "fragmentColor");

		glUniform3f(fragmentColorLocation, intersectionColour.x, intersectionColour.y, intersectionColour.z);
		MVP = cam.mvpMatrix * ModeltoWorld;
		// Set the model matrix for the mesh
		glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &MVP[0][0]);

		// draw mesh
		glBindVertexArray(vao);
		glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}


	/*  _________________________________________________________________________ */
	/*! AssignmentScene::BoundingVolumes::LoadVolumes
	@param BV volume
	@param std::vector<glm::vec3> vertices

	@return none
	Generate the respective bounding volumes
	*/
	void AssignmentScene::BoundingVolumes::LoadVolumes(BV volume, std::vector<glm::vec3> vertices)
	{
		// write function to generate each BV and store the mesh 
		switch (volume)
		{
		case Assignment3::AssignmentScene::AABB:
			GenerateAABB(vertices);
			break;
		case Assignment3::AssignmentScene::PCA:
			GeneratePCASphere(vertices);
			break;
		case Assignment3::AssignmentScene::OBB:
			GenerateOBB(vertices);
			break;
		default:
			break;
		}

	}


	/*  _________________________________________________________________________ */
	/*! AssignmentScene::BoundingVolumes::GenerateAABB
	@param std::vector<glm::vec3> vertices

	@return none
	Generate the respective bounding volumes - AABB
	*/
	void AssignmentScene::BoundingVolumes::GenerateAABB(std::vector<glm::vec3> vertices)
	{
		// set both to first vertex
		this->aabbbox.min = this->aabbbox.max = vertices[0];
		for (auto& vert : vertices)
		{
			// Update minPoint and maxPoint along the X, Y, and Z axes
			this->aabbbox.min.x = std::min(this->aabbbox.min.x, vert.x);
			this->aabbbox.min.y = std::min(this->aabbbox.min.y, vert.y);
			this->aabbbox.min.z = std::min(this->aabbbox.min.z, vert.z);

			this->aabbbox.max.x = std::max(this->aabbbox.max.x, vert.x);
			this->aabbbox.max.y = std::max(this->aabbbox.max.y, vert.y);
			this->aabbbox.max.z = std::max(this->aabbbox.max.z, vert.z);
		}

		float halfExtentX = 0.5f * (this->aabbbox.max.x - this->aabbbox.min.x);
		float halfExtentY = 0.5f * (this->aabbbox.max.y - this->aabbbox.min.y);
		float halfExtentZ = 0.5f * (this->aabbbox.max.z - this->aabbbox.min.z);

		this->aabbbox.halfExtents = glm::vec3(halfExtentX, halfExtentY, halfExtentZ);
		this->aabbbox.PA = { 1.f,0.f,0.f };
		this->aabbbox.Y = { 0.f,1.f,0.f };
		this->aabbbox.Z = { 0.f,0.f,1.f };

		this->aabbbox.center = (this->aabbbox.max + this->aabbbox.min) / 2.f;
	}


	/*  _________________________________________________________________________ */
	/*! AssignmentScene::BoundingVolumes::GeneratePCASphere
	@param std::vector<glm::vec3> vertices

	@return none
	Generate the respective bounding volumes - PCASphere
	*/
	void AssignmentScene::BoundingVolumes::GeneratePCASphere(std::vector<glm::vec3> vertices)
	{
		// Create QuickHull object
		quickhull::QuickHull<float> qh;
		std::vector<quickhull::Vector3<float>> points;
		for (const auto& vertex : vertices) {
			points.emplace_back(vertex.x, vertex.y, vertex.z);
		}

		auto hull = qh.getConvexHull(points, true, false);
		//const auto& indexBuffer = hull.getIndexBuffer();
		const auto& vertexBuffer = hull.getVertexBuffer();

		// Convert convex hull back to glm::vec3 format
		std::vector<glm::vec3> convexHullVertices;
		convexHullVertices.reserve(vertexBuffer.size());
		for (const auto& point : vertexBuffer)
		{
			convexHullVertices.push_back(glm::vec3(point.x, point.y, point.z));
		}

		// Step 1: Compute the centroid
		glm::vec3 centroid(0.0f);
		for (const glm::vec3& vertex : convexHullVertices) {
			centroid += vertex;
		}
		centroid /= static_cast<float>(convexHullVertices.size());

		// Step 2: Compute the covariance matrix
		Eigen::Matrix3f covarianceMatrix = Eigen::Matrix3f::Zero();
		for (const glm::vec3& vertex : convexHullVertices) {
			glm::vec3 deviation = vertex - centroid;
			covarianceMatrix(0, 0) += deviation.x * deviation.x;
			covarianceMatrix(0, 1) += deviation.x * deviation.y;
			covarianceMatrix(0, 2) += deviation.x * deviation.z;
			covarianceMatrix(1, 0) += deviation.y * deviation.x;
			covarianceMatrix(1, 1) += deviation.y * deviation.y;
			covarianceMatrix(1, 2) += deviation.y * deviation.z;
			covarianceMatrix(2, 0) += deviation.z * deviation.x;
			covarianceMatrix(2, 1) += deviation.z * deviation.y;
			covarianceMatrix(2, 2) += deviation.z * deviation.z;
		}
		covarianceMatrix /= static_cast<float>(convexHullVertices.size());

		// Compute eigen vectors and eigen values
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covarianceMatrix);
		Eigen::Matrix3f eigenVectors = eigenSolver.eigenvectors().real();
		Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
		int maxEigenValueIndex = static_cast<int> (eigenValues.maxCoeff());

		// Compute the remaining axes (y-axis and z-axis)
		int yAxis, zAxis;
		if (maxEigenValueIndex == 0) {
			yAxis = 1;
			zAxis = 2;
		}
		else if (maxEigenValueIndex == 1) {
			yAxis = 0;
			zAxis = 2;
		}
		else {
			yAxis = 0;
			zAxis = 1;
		}

		// Find the maximum and minimum dot products between points and normals
		glm::vec3 normX = { eigenVectors.col(maxEigenValueIndex).x(),eigenVectors.col(maxEigenValueIndex).y(),eigenVectors.col(maxEigenValueIndex).z() };
		glm::vec3 normY = { eigenVectors.col(yAxis).x(),eigenVectors.col(yAxis).y(),eigenVectors.col(yAxis).z() };
		glm::vec3 normZ = { eigenVectors.col(zAxis).x(),eigenVectors.col(zAxis).y(),eigenVectors.col(zAxis).z() };
		std::vector<glm::vec3> norm;
		norm.push_back(normX);
		norm.push_back(normY);
		norm.push_back(normZ);


		// generate mininum sphere using ritter..
		this->spherePCA.min = this->spherePCA.max = vertices[0];
		for (auto& vert : vertices)
		{
			Eigen::Vector3f point =
				Eigen::Vector3f(vert.x, vert.y, vert.z);
			float xProjection = point.dot(eigenVectors.col(maxEigenValueIndex));
			float yProjection = point.dot(eigenVectors.col(yAxis));
			float zProjection = point.dot(eigenVectors.col(zAxis));

			// Update minPoint and maxPoint along the X, Y, and Z axes
			this->spherePCA.min.x = std::min(this->spherePCA.min.x, xProjection);
			this->spherePCA.min.y = std::min(this->spherePCA.min.y, yProjection);
			this->spherePCA.min.z = std::min(this->spherePCA.min.z, zProjection);

			this->spherePCA.max.x = std::max(this->spherePCA.max.x, xProjection);
			this->spherePCA.max.y = std::max(this->spherePCA.max.y, yProjection);
			this->spherePCA.max.z = std::max(this->spherePCA.max.z, zProjection);
		}

		// Calculate squared distances along each axis
		float distx = this->spherePCA.max.x - this->spherePCA.min.x;
		float disty = this->spherePCA.max.y - this->spherePCA.min.y;
		float distz = this->spherePCA.max.z - this->spherePCA.min.z;

		// Calculate radius as half the diameter
		spherePCA.radius = std::max(distx, std::max(disty, distz)) / 2.0f;

		// second pass, check if there are any points outside sphere..

		glm::vec3 C = (this->spherePCA.max + this->spherePCA.min) * 0.5f;
		glm::vec3 x = { eigenVectors.col(maxEigenValueIndex).x(),eigenVectors.col(yAxis).x(),eigenVectors.col(zAxis).x() };
		glm::vec3 y = { eigenVectors.col(maxEigenValueIndex).y(),eigenVectors.col(yAxis).y(),eigenVectors.col(zAxis).y() };
		glm::vec3 z = { eigenVectors.col(maxEigenValueIndex).z(),eigenVectors.col(yAxis).z(),eigenVectors.col(zAxis).z() };

		PointModel point;
		BoundingSphereModel sphere;
		sphere.position = { glm::dot(C,x),glm::dot(C,y) ,glm::dot(C,z) };
		sphere.radius = spherePCA.radius;
		for (auto& vert : vertices)
		{
			point.position = { glm::dot(vert,x),glm::dot(vert,y) ,glm::dot(vert,z) };
			if (!Intersection::pointSphereIntersection(point, sphere)) // point is outside sphere, expand sphere
			{
				// Calculate the vector from the current center to the point
				glm::vec3 centerToPoint = glm::normalize(point.position - sphere.position);
				// Find point directly opp..
				glm::vec3 pPrime = sphere.position - sphere.radius * centerToPoint;
				sphere.position = (point.position + pPrime) / 2.f;
				sphere.radius = glm::distance(point.position, sphere.position);
			}

		}
		spherePCA.center = sphere.position;
		spherePCA.radius = sphere.radius;
	}

	/*  _________________________________________________________________________ */
	/*! AssignmentScene::BoundingVolumes::GenerateOBB
	@param std::vector<glm::vec3> vertices

	@return none
	Generate the respective bounding volumes - OBB
	*/
	void AssignmentScene::BoundingVolumes::GenerateOBB(std::vector<glm::vec3> vertices)
	{
		// Create QuickHull object
		quickhull::QuickHull<float> qh;
		std::vector<quickhull::Vector3<float>> points;
		for (const auto& vertex : vertices) {
			points.emplace_back(vertex.x, vertex.y, vertex.z);
		}

		auto hull = qh.getConvexHull(points, true, false);
		//const auto& indexBuffer = hull.getIndexBuffer();
		const auto& vertexBuffer = hull.getVertexBuffer();

		// Convert convex hull back to glm::vec3 format
		std::vector<glm::vec3> convexHullVertices;
		convexHullVertices.reserve(vertexBuffer.size());
		for (const auto& point : vertexBuffer)
		{
			convexHullVertices.push_back(glm::vec3(point.x, point.y, point.z));
		}

		// Step 1: Compute the centroid
		glm::vec3 centroid(0.0f);
		for (const glm::vec3& vertex : convexHullVertices) {
			centroid += vertex;
		}
		centroid /= static_cast<float>(convexHullVertices.size());

		// Step 2: Compute the covariance matrix
		Eigen::Matrix3f covarianceMatrix = Eigen::Matrix3f::Zero();
		for (const glm::vec3& vertex : convexHullVertices) {
			glm::vec3 deviation = vertex - centroid;
			covarianceMatrix(0, 0) += deviation.x * deviation.x;
			covarianceMatrix(0, 1) += deviation.x * deviation.y;
			covarianceMatrix(0, 2) += deviation.x * deviation.z;
			covarianceMatrix(1, 0) += deviation.y * deviation.x;
			covarianceMatrix(1, 1) += deviation.y * deviation.y;
			covarianceMatrix(1, 2) += deviation.y * deviation.z;
			covarianceMatrix(2, 0) += deviation.z * deviation.x;
			covarianceMatrix(2, 1) += deviation.z * deviation.y;
			covarianceMatrix(2, 2) += deviation.z * deviation.z;
		}
		covarianceMatrix /= static_cast<float>(convexHullVertices.size());

		// Compute eigen vectors and eigen values
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covarianceMatrix);
		Eigen::Matrix3f eigenVectors = eigenSolver.eigenvectors().real();
		Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
		int maxEigenValueIndex = static_cast<int> (eigenValues.maxCoeff());

		// Compute the remaining axes (y-axis and z-axis)
		int yAxis, zAxis;
		if (maxEigenValueIndex == 0) {
			yAxis = 1;
			zAxis = 2;
		}
		else if (maxEigenValueIndex == 1) {
			yAxis = 0;
			zAxis = 2;
		}
		else {
			yAxis = 0;
			zAxis = 1;
		}


		// Compute the minimum and maximum points along all three axes
		glm::vec3 minPoint(std::numeric_limits<float>::max());
		glm::vec3 maxPoint(std::numeric_limits<float>::lowest());


		for (const glm::vec3& vertex : vertices) {
			Eigen::Vector3f point = 
				Eigen::Vector3f(vertex.x, vertex.y, vertex.z);

			float xProjection = point.dot(eigenVectors.col(maxEigenValueIndex));
			float yProjection = point.dot(eigenVectors.col(yAxis));
			float zProjection = point.dot(eigenVectors.col(zAxis));

			minPoint.x = std::min(minPoint.x, xProjection);
			minPoint.y = std::min(minPoint.y, yProjection);
			minPoint.z = std::min(minPoint.z, zProjection);

			maxPoint.x = std::max(maxPoint.x, xProjection);
			maxPoint.y = std::max(maxPoint.y, yProjection);
			maxPoint.z = std::max(maxPoint.z, zProjection);
		}

		OBB.halfExtents = glm::vec3((maxPoint - minPoint) * 0.5f);
		OBB.PA = { eigenVectors.col(maxEigenValueIndex).x(),eigenVectors.col(maxEigenValueIndex).y(),eigenVectors.col(maxEigenValueIndex).z() };
		OBB.Y = { eigenVectors.col(yAxis).x(),eigenVectors.col(yAxis).y(),eigenVectors.col(yAxis).z() };
		OBB.Z = { eigenVectors.col(zAxis).x(),eigenVectors.col(zAxis).y(),eigenVectors.col(zAxis).z() };

		glm::vec3 C = (maxPoint + minPoint) * 0.5f;
		glm::vec3 x = { eigenVectors.col(maxEigenValueIndex).x(),eigenVectors.col(yAxis).x(),eigenVectors.col(zAxis).x() };
		glm::vec3 y = { eigenVectors.col(maxEigenValueIndex).y(),eigenVectors.col(yAxis).y(),eigenVectors.col(zAxis).y() };
		glm::vec3 z = { eigenVectors.col(maxEigenValueIndex).z(),eigenVectors.col(yAxis).z(),eigenVectors.col(zAxis).z() };

		OBB.center = {glm::dot(C,x),glm::dot(C,y) ,glm::dot(C,z) };
	}


	/*  _________________________________________________________________________ */
	/*! AssignmentScene::BoundingVolumes::SetupBuffersBV
	@param BV volume
	@param const GLuint ID

	@return none
	Generate the vertices and indices needed for rendering for each
	respective specified bounding volume
	*/
	void AssignmentScene::BoundingVolumes::SetupBuffersBV(BV volume, const GLuint ID)
	{
		// clear memory of setup
		std::vector<glm::vec3>().swap(vertices);
		std::vector<unsigned int>().swap(indices);
		switch (volume)
		{
		case Assignment3::AssignmentScene::AABB:
		case Assignment3::AssignmentScene::OBB:
		{
			AABBC box = volume == AABB ? aabbbox : OBB;

			// Define the eight corners of the AABB
			vertices.resize(8);
			// Positive half extents along each axis
			vertices[0] = box.center + box.PA * box.halfExtents.x + box.Y * box.halfExtents.y + box.Z * box.halfExtents.z;
			vertices[1] = box.center + box.PA * box.halfExtents.x + box.Y * box.halfExtents.y - box.Z * box.halfExtents.z;
			vertices[2] = box.center + box.PA * box.halfExtents.x - box.Y * box.halfExtents.y - box.Z * box.halfExtents.z;
			vertices[3] = box.center + box.PA * box.halfExtents.x - box.Y * box.halfExtents.y + box.Z * box.halfExtents.z;

			// Negative half extents along each axis
			vertices[4] = box.center - box.PA * box.halfExtents.x + box.Y * box.halfExtents.y + box.Z * box.halfExtents.z;
			vertices[5] = box.center - box.PA * box.halfExtents.x + box.Y * box.halfExtents.y - box.Z * box.halfExtents.z;
			vertices[6] = box.center - box.PA * box.halfExtents.x - box.Y * box.halfExtents.y - box.Z * box.halfExtents.z;
			vertices[7] = box.center - box.PA * box.halfExtents.x - box.Y * box.halfExtents.y + box.Z * box.halfExtents.z;

			// Define the indices for the AABB faces
			indices = {
				// Front face
				 0, 1, 2,
				 2, 3, 0,

				 // Top face
				 3, 2, 6,
				 6, 7, 3,

				 // Back face
				 7, 6, 5,
				 5, 4, 7,

				 // Bottom face
				 4, 5, 1,
				 1, 0, 4,

				 // Left face
				 4, 0, 3,
				 3, 7, 4,

				 // Right face
				 1, 5, 6,
				 6, 2, 1
			};
		}
		break;

		case Assignment3::AssignmentScene::PCA:
		{
			Sphere sphere;

			sphere = spherePCA;

			//construct sphere base on radius and center...
			int sectorCount = 16;
			int stackCount = 16;

			float sectorStep = static_cast<float> (2 * PI / sectorCount);
			float stackStep = static_cast<float> (PI / stackCount);
			float sectorAngle, stackAngle;

			for (int i = 0; i <= stackCount; ++i)
			{
				stackAngle = static_cast<float> (PI / 2 - i * stackStep);        // starting from pi/2 to -pi/2
				float xy = sphere.radius * cosf(stackAngle);  // r * cos(u)
				float z = sphere.radius * sinf(stackAngle);   // r * sin(u)

				// add (sectorCount+1) vertices per stack
				// first and last vertices have same position and normal, but different tex coords
				for (int j = 0; j <= sectorCount; ++j)
				{
					sectorAngle = j * sectorStep;           // starting from 0 to 2pi

					// vertex position (x, y, z)
					float x = xy * cosf(sectorAngle);       // r * cos(u) * cos(v)
					float y = xy * sinf(sectorAngle);       // r * cos(u) * sin(v)

					vertices.push_back(glm::vec3(x, y, z) + sphere.center);
				}
			}

			// indices
			for (int i = 0; i < stackCount; ++i)
			{
				int k1 = i * (sectorCount + 1);     // current stack
				int k2 = k1 + sectorCount + 1;      // next stack

				for (int j = 0; j < sectorCount; ++j, ++k1, ++k2)
				{
					// indices for the two triangles of each sector
					if (i != 0)
					{
						indices.push_back(k1);
						indices.push_back(k2);
						indices.push_back(k1 + 1);
					}

					if (i != (stackCount - 1))
					{
						indices.push_back(k1 + 1);
						indices.push_back(k2);
						indices.push_back(k2 + 1);
					}
				}
			}
		}
		break;
		default:
			break;
		}

		shaderID = ID;

		// Generate and bind vertex array object (VAO)
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		// Generate and bind vertex buffer object (VBO)
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

		// Generate and bind element buffer object (EBO)
		glGenBuffers(1, &ebo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

		// Enable vertex attributes and specify their layout
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

		// Unbind VAO, VBO, and EBO
		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	}

	/*  _________________________________________________________________________ */
	/*! AssignmentScene::BoundingVolumes::RenderMeshBV
	@param Camera& cam
	@param const glm::vec3& position
	@param bool obb
	@param const glm::vec3& angle

	@return none
	Renders the bounding volume
	*/
	void AssignmentScene::BoundingVolumes::RenderMeshBV(Camera& cam, const glm::vec3& position, bool obb, const glm::vec3& angle)
	{
		// Set the MVP matrix for the mesh
		glm::mat4 modelMatrix(1.0f);
		glm::vec3 scaleVector = glm::vec3(1.f);
		glm::vec3 rotateVector = glm::vec3(1.f);

		if (obb)
			ModeltoWorld =
			glm::translate(modelMatrix, position)
			* glm::rotate(glm::radians(angle.x), glm::vec3(1.0f, 0.0f, 0.0f))
			* glm::rotate(glm::radians(angle.y), glm::vec3(0.0f, 1.0f, 0.0f))
			* glm::rotate(glm::radians(angle.z), glm::vec3(0.0f, 0.0f, 1.0f))
			* glm::scale(scaleVector);
		else
			ModeltoWorld =
			glm::translate(modelMatrix, position)
			* glm::rotate(0.f, rotateVector)
			* glm::scale(scaleVector);

		glUseProgram(shaderID);

		// Set the uniform matrix locations
		GLint mvpMatrixLoc = glGetUniformLocation(shaderID, "mvpMatrixLoc");
		GLint fragmentColorLocation = glGetUniformLocation(shaderID, "fragmentColor");

		glUniform3f(fragmentColorLocation, 0.f,0.f,1.f);


		// Set the model matrix for the mesh
		MVP = cam.mvpMatrix * ModeltoWorld;
		glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &MVP[0][0]);

		// draw mesh
		glBindVertexArray(vao);
		glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, (void*)0);
		glBindVertexArray(0);
	}


	void AssignmentScene::BoundingVolumes::RenderMeshBV(Camera& cam, const glm::vec3& position, const int level)
	{
		// Set the MVP matrix for the mesh
		glm::mat4 modelMatrix(1.0f);
		glm::vec3 scaleVector = glm::vec3(1.f);
		glm::vec3 rotateVector = glm::vec3(1.f);

		
		ModeltoWorld =
		glm::translate(modelMatrix, position)
		* glm::rotate(0.f, rotateVector)
		* glm::scale(scaleVector);

		glUseProgram(shaderID);

		// Set the uniform matrix locations
		GLint mvpMatrixLoc = glGetUniformLocation(shaderID, "mvpMatrixLoc");
		GLint fragmentColorLocation = glGetUniformLocation(shaderID, "fragmentColor");
		glm::vec3 color;
		switch (level)
		{
		case 0:
			color = { 1.f, 0.f, 0.f }; // red
			break;
		case 1:
			color = { 1.f, 0.7f, 0.f }; // orange
			break;
		case 2:
			color = { 0.7f, 1.f, 0.f }; // yellow
			break;
		case 3:
			color = { 0.0f, 1.0f, 1.f }; // cyan
			break;
		case 4:
			color = { 1.0f, 1.0f, 1.f }; // white
			break;
		case 5:
			color = { 1.0f, 0.0f, 1.0f }; // Purple
			break;
		case 6:
			color = { 2.0f, 0.5f, 1.0f }; // lilac
			break;

		}
		glUniform3f(fragmentColorLocation, color.x, color.y, color.z);


		// Set the model matrix for the mesh
		MVP = cam.mvpMatrix * ModeltoWorld;
		glUniformMatrix4fv(mvpMatrixLoc, 1, GL_FALSE, &MVP[0][0]);

		// draw mesh
		glBindVertexArray(vao);
		glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, (void*)0);
		glBindVertexArray(0);
	}


	/*  _________________________________________________________________________ */
	/*! AssignmentScene::Init()
	@return int

	Initialize the scene by loading shaders, geometry, Imgui and camera projection
	*/
	int AssignmentScene::Init()
	{
		// Create and compile our GLSL program from the shaders
		programID = LoadShaders("../Common/shaders/Ass2.vert",
			"../Common/shaders/Ass2.frag");

		SetupNanoGUI(currentWindow);

		InitCameras();
		LoadScene();

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
		ca1.cameraPosition = glm::vec3(0.0f, 0.0f, 30.0f);
		ca1.cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
		ca1.cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);  // Up vector for camera orientation

		Camera ca2;
		// Camera parameters normal
		ca2.cameraPosition = glm::vec3(0.0f, 30.0f, 0.0f);
		ca2.cameraFront = glm::vec3(0.0f, -1.0f, 0.0f);
		ca2.cameraUp = glm::vec3(0.0f, 0.0f, -1.0f);  // Up vector for camera orientation

		Cameras.push_back(ca1);
		Cameras.push_back(ca2);

		for (Camera& ca : Cameras)
		{
			// Field of view and aspect ratio
			ca.fov = 45.f;                      // Field of view (in degrees)
			ca.aspectRatio = static_cast<float>(WindowWidth) / static_cast<float>(WindowHeight);     // Aspect ratio of the window

			// Near and far clipping planes
			ca.nearPlane = 0.1f; // Adjusted near clipping plane
			ca.farPlane = 100.0f; // Adjusted far clipping plane

			// Calculate the view matrix
			ca.viewMatrix = glm::lookAt(ca.cameraPosition, ca.cameraPosition + ca.cameraFront, ca.cameraUp);

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

		 // main camera
		glViewport(0, 0, WindowWidth, WindowHeight);

		// Enable or disable wireframe rendering based on the checkbox state
		enableWireframeModel ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		UpdateCamera();
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// render model
		for (auto& y : models)
			for (auto& x :y.mesh)
				x.RenderMesh(Cameras[0], y.position, y.Rotangle);
			


		enableWireframeBV ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		// render BV
		for (auto& x : models)
			x.BVs[x.selectedBVOption].RenderMeshBV(Cameras[0], x.position,
				x.selectedBVOption == OBB, x.Rotangle);


		
		if (enableTopDownBVH)
		{
			enableWireframeBVH ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			if (selectedTopBVHOption)
				TraverseTreePrint(TopDownRootSphere[selectedrootOption], 0);
			else
				TraverseTreePrint(TopDownRootAABB[selectedrootOption], 0);

		}
		if (enableBottomUpBVH)
		{
			enableWireframeBVH ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			if (selectedTopBVHOption)
				TraverseTreePrint(BottomUpRootSphere_NODE, 0);
			else
				TraverseTreePrint(BottomUpRootAABB_NODE, 0);

		}


		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		//render top right
		int secondcamwidth = WindowWidth / 4;
		int secondcamheight = WindowHeight / 4;
		glViewport(WindowWidth - secondcamwidth, WindowHeight - secondcamheight, secondcamwidth, secondcamheight);
		enableWireframeModel ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		for (auto& y : models)
			for (auto& x : y.mesh)
				x.RenderMesh(Cameras[1], y.position, y.Rotangle);


		enableWireframeBV ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		// render BV
		for (auto& x : models)
			x.BVs[x.selectedBVOption].RenderMeshBV(Cameras[1], x.position,
				x.selectedBVOption == OBB, x.Rotangle);

		if (enableTopDownBVH)
		{
			enableWireframeBVH ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			if (selectedTopBVHOption)
				TraverseTreePrint(TopDownRootSphere[selectedrootOption], 1);
			else
				TraverseTreePrint(TopDownRootAABB[selectedrootOption], 1);

		}
		if (enableBottomUpBVH)
		{
			enableWireframeBVH ? glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) : glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			if (selectedTopBVHOption)
				TraverseTreePrint(BottomUpRootSphere_NODE, 1);
			else
				TraverseTreePrint(BottomUpRootAABB_NODE, 1);

		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// setting back to main
		glViewport(0, 0, WindowWidth, WindowHeight);

		// ImGui drag and drop options
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// all imgui code goes between newframe and renderdraw
		ImGui::Checkbox("Enable Wireframe for model", &enableWireframeModel);
		ImGui::Checkbox("Enable Wireframe for BV", &enableWireframeBV);
		ImGui::Checkbox("Enable Wireframe for BVH", &enableWireframeBVH);
		ImGui::Checkbox("Top-Down BVH", &enableTopDownBVH);
		ImGui::Checkbox("Bottom-Up BVH", &enableBottomUpBVH);

		// to change models 
		// Convert vector of strings to array of const char*
		std::vector<const char*> name;
		for (const auto& modelName : modelnames)
			name.push_back(modelName.c_str());
		ImGui::Combo("Change Model", &selectedModelOption, name.data(), static_cast<int>(name.size()));

		// to change models 
		ImGui::Combo("Change Model's BV", &models[selectedModelOption].selectedBVOption, BVnames, IM_ARRAYSIZE(BVnames));

		// to  termination conditions
		if (enableTopDownBVH)
		{
			ImGui::Combo("BVH", &selectedTopBVHOption, TOPDOWNAABBSPHERE, IM_ARRAYSIZE(TOPDOWNAABBSPHERE));
			ImGui::Combo("Change Termination Condition", &selectedrootOption, TerminationCond, IM_ARRAYSIZE(TerminationCond));
		}
		if (enableBottomUpBVH)
		{
			ImGui::Combo("BVH", &selectedTopBVHOption, TOPDOWNAABBSPHERE, IM_ARRAYSIZE(TOPDOWNAABBSPHERE));
		}

		// To change cameras
		//Movement
		ImGui::Text("Model Transformation Controls");
		// Create a unique ID for each drag control to differentiate them
		// and avoid affecting both points when modified
		std::string positionID = "Rotation";
		ImGui::DragFloat3(positionID.c_str(), &models[selectedModelOption].Rotangle.x);

		// Render ImGui
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		return 0;
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

		}
		if (glfwGetMouseButton(currentWindow, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE)
			mouse.firsttime = true;

		glm::vec3 front;
		front.x = cos(glm::radians(mouse.yaw)) * cos(glm::radians(mouse.pitch));
		front.y = sin(glm::radians(mouse.pitch));
		front.z = sin(glm::radians(mouse.yaw)) * cos(glm::radians(mouse.pitch));
		Cameras[0].cameraFront = glm::normalize(front);

		// also re-calculate the Right and Up vector
		Cameras[0].cameraRight = glm::normalize(glm::cross(Cameras[0].cameraFront, glm::vec3(0.f, 1.f, 0.f)));
		Cameras[0].cameraUp = glm::normalize(glm::cross(Cameras[0].cameraRight, Cameras[0].cameraFront));

		glfwSetScrollCallback(currentWindow, scroll_callback);

		float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		float cameraSpeed = mouse.keyboardspeed * deltaTime;
		if (glfwGetKey(currentWindow, GLFW_KEY_W) == GLFW_PRESS)
			Cameras[0].cameraPosition += cameraSpeed * Cameras[0].cameraUp;
		if (glfwGetKey(currentWindow, GLFW_KEY_S) == GLFW_PRESS)
			Cameras[0].cameraPosition -= cameraSpeed * Cameras[0].cameraUp;
		if (glfwGetKey(currentWindow, GLFW_KEY_A) == GLFW_PRESS)
			Cameras[0].cameraPosition -= glm::normalize(glm::cross(Cameras[0].cameraFront, Cameras[0].cameraUp)) * cameraSpeed;
		if (glfwGetKey(currentWindow, GLFW_KEY_D) == GLFW_PRESS)
			Cameras[0].cameraPosition += glm::normalize(glm::cross(Cameras[0].cameraFront, Cameras[0].cameraUp)) * cameraSpeed;

		// update camera 
		// Calculate the projection matrix
		// Calculate the view matrix
		Cameras[0].viewMatrix =
			glm::lookAt(Cameras[0].cameraPosition, Cameras[0].cameraPosition + Cameras[0].cameraFront, Cameras[0].cameraUp);

		Cameras[0].fov = fov;
		Cameras[0].projectionMatrix =
			glm::perspective(glm::radians(Cameras[0].fov), Cameras[0].aspectRatio, Cameras[0].nearPlane, Cameras[0].farPlane);

		// Combine the view and projection matrices for the final MVP matrix
		Cameras[0].mvpMatrix =
			Cameras[0].projectionMatrix * Cameras[0].viewMatrix;
	}



	void AssignmentScene::TopDownTree(TreeNode* node, std::vector<Model> objects, BV type, const int numObjects, int level, int cond)
	{
		if (type == AABB)
			node->BV = ComputeBVAABB(objects);
		else
			node->BV = ComputeBVSphere(objects);

		node->BVlevel = level;
		if ((cond == 0 && numObjects <= 1) || (cond == 1 && numObjects <= 2) || (cond == 2 && level == 2))
		{
			node->nodetype = type::leaf;
			node->objectdata = &objects[0];
			node->numObjects = numObjects;
		}
		else
		{
			node->nodetype = type::Internal;

			// based on partition
			int k = 0;
			if (type == AABB)
				k = partitionbyMedianBVCAABB(objects);
			else
				k = partitionbyMedianBVCPCA(objects);

			node->lchild = new TreeNode();
			node->rchild = new TreeNode();

			//recursive
			
			// Create the sub-vector
			std::vector<Model> subVectorL(objects.begin(), objects.begin() + k);

			// Create the sub-vector
			std::vector<Model> subVectorR(objects.begin() + k, objects.end());


			TopDownTree(node->lchild, subVectorL, type,k,level + 1, cond);
			TopDownTree(node->rchild, subVectorR, type,numObjects - k, level + 1, cond);

		}
	}

	AssignmentScene::BoundingVolumes* AssignmentScene::ComputeBVAABB(const std::vector<Model> objects)
	{
		// AABB BVH
		BoundingVolumes* BV = new BoundingVolumes;
		std::vector<glm::vec3> verticesAABB;
		
		for (const auto& model : objects) {

			verticesAABB.push_back(model.BVs[0].aabbbox.min + model.position);
			verticesAABB.push_back(model.BVs[0].aabbbox.max + model.position);
		}

		BV->GenerateAABB(verticesAABB);
		BV->SetupBuffersBV(AssignmentScene::AABB, AssignmentScene::programID);

		return BV;
	}

	AssignmentScene::BoundingVolumes* AssignmentScene::ComputeBVSphere(std::vector<Model> objects)
	{
		BoundingVolumes* BV = new BoundingVolumes;

		//Sphere BVH
		std::vector<glm::vec3> verticesSphere;

		glm::vec3 centroid(0.0f);
		for (const auto& pcaSphere : objects) {
			centroid += pcaSphere.BVs[1].spherePCA.center + pcaSphere.position;
		}
		centroid /= objects.size();

		float maxDistance = 0.0f;
		for (const auto& pcaSphere : objects) {
			float distance = glm::distance(centroid, pcaSphere.BVs[1].spherePCA.center + pcaSphere.position) + pcaSphere.BVs[1].spherePCA.radius;
			if (distance > maxDistance) {
				maxDistance = distance;
			}
		}

		BV->spherePCA.center = centroid;
		BV->spherePCA.radius = maxDistance;


		BV->SetupBuffersBV(AssignmentScene::PCA, AssignmentScene::programID);

		return BV;
	}

	void AssignmentScene::TraverseTreePrint(TreeNode* node,const int cam)
	{
		node->BV->RenderMeshBV(Cameras[cam], glm::vec3(0.f), node->BVlevel);


		if (node->nodetype == type::leaf) {
			return;
		}

		// Recursively traverse children
		TraverseTreePrint(node->lchild, cam);
		TraverseTreePrint(node->rchild, cam);
	}

	void AssignmentScene::TraverseTreedelete(TreeNode* node)
	{
		if (node->nodetype == type::leaf) {
			delete node->BV;
			delete node;
			return;
		}

		// Recursively traverse children
		TraverseTreedelete(node->lchild);
		TraverseTreedelete(node->rchild);

		delete node->BV;
		delete node;
	}


	int AssignmentScene::SplitAxisByMinVolumeAABB(std::vector<AABBC>& aabbs)
	{
		float minTotalVolume = std::numeric_limits<float>::max();
		int splitAxis = 0; // Initialize with the X axis

		// Calculate the total volume for each axis and find the minimum
		for (int axis = 0; axis < 3; ++axis) { // 0 for X, 1 for Y, 2 for Z
			std::vector<float> centers;

			// Calculate the center of each AABB along the current axis
			for (const auto& aabb : aabbs) 
				centers.push_back(aabb.center[axis]);
			
			// Sort the centers
			std::sort(centers.begin(), centers.end());

			// Calculate the total volume for the current axis
			float totalVolume = 0.0f;
			for (size_t i = 1; i < centers.size(); ++i) {
				float splitPoint = (centers[i - 1] + centers[i]) * 0.5f;
				std::vector<AABBC> leftAABBs(aabbs.begin(), aabbs.begin() + i);
				std::vector<AABBC> rightAABBs(aabbs.begin() + i, aabbs.end());

				// Calculate the total volume of the child nodes
				float leftVolume = calculateTotalVolumeAABB(leftAABBs);
				float rightVolume = calculateTotalVolumeAABB(rightAABBs);
				float currentTotalVolume = leftVolume + rightVolume;

				// Update the minimum total volume and split axis if necessary
				if (currentTotalVolume < minTotalVolume) {
					minTotalVolume = currentTotalVolume;
					splitAxis = axis;
				}
			}
		}

		return splitAxis;
	}

	float AssignmentScene::calculateTotalVolumeAABB(const std::vector<AABBC>& aabbList)
	{
		float totalVolume = 0.0f;

		for (const auto& aabb : aabbList) {
			glm::vec3 extent = aabb.max - aabb.min;
			float volume = extent.x * extent.y * extent.z;
			totalVolume += volume;
		}
		return totalVolume;
	}
	int AssignmentScene::partitionbyMedianBVCAABB(std::vector<Model>& modelss)
	{
		std::vector<AABBC> aabbs;
		for (auto& x : modelss)
			aabbs.push_back(x.BVs[0].aabbbox);

		int axis = SplitAxisByMinVolumeAABB(aabbs);
		std::vector<float> center;

		// Sort the models based on the center coordinates
		std::sort(modelss.begin(), modelss.end(), [&axis](const Model& a, const Model& b) {
			return a.BVs[0].position[axis] < b.BVs[0].position[axis];
			});

		for (auto& x : modelss)
			center.push_back(x.BVs[0].aabbbox.center[axis]);

		// Calculate the index to split (median index)
		int splitIndex = center.size() / 2;

		return splitIndex;
	}


	int AssignmentScene::SplitAxisByMinVolumePCA(std::vector<Sphere>& pcas)
	{
		float minTotalVolume = std::numeric_limits<float>::max();
		int splitAxis = 0; // Initialize with the X axis

		// Calculate the total volume for each axis and find the minimum
		for (int axis = 0; axis < 3; ++axis) { // 0 for X, 1 for Y, 2 for Z
			std::vector<float> centers;

			// Calculate the center of each AABB along the current axis
			for (const auto& pca : pcas)
				centers.push_back(pca.center[axis]);

			// Sort the centers
			std::sort(centers.begin(), centers.end());

			// Calculate the total volume for the current axis
			float totalVolume = 0.0f;
			for (size_t i = 1; i < centers.size(); ++i) {
				float splitPoint = (centers[i - 1] + centers[i]) * 0.5f;
				std::vector<Sphere> leftpcas(pcas.begin(), pcas.begin() + i);
				std::vector<Sphere> rightpcas(pcas.begin() + i, pcas.end());

				// Calculate the total volume of the child nodes
				float leftVolume = calculateTotalVolumePCA(leftpcas);
				float rightVolume = calculateTotalVolumePCA(rightpcas);
				float currentTotalVolume = leftVolume + rightVolume;

				// Update the minimum total volume and split axis if necessary
				if (currentTotalVolume < minTotalVolume) {
					minTotalVolume = currentTotalVolume;
					splitAxis = axis;
				}
			}
		}

		return splitAxis;
	}


	int AssignmentScene::partitionbyMedianBVCPCA(std::vector<Model>& modelss)
	{
		std::vector<Sphere> pcas;
		for (auto& x : modelss)
			pcas.push_back(x.BVs[1].spherePCA);

		int axis = SplitAxisByMinVolumePCA(pcas);
		std::vector<float> center;

		// Sort the models based on the center coordinates
		std::sort(modelss.begin(), modelss.end(), [&axis](const Model& a, const Model& b) {
			return a.BVs[1].position[axis] < b.BVs[1].position[axis];
			});

		for (auto& x : modelss)
			center.push_back(x.BVs[1].spherePCA.center[axis]);

		// Calculate the index to split (median index)
		int splitIndex = center.size() / 2;

		return splitIndex;
	}
	float AssignmentScene::calculateTotalVolumePCA(const std::vector<Sphere>& pcalist)
	{
		float totalVolume = 0.0f;

		for (const auto& pca : pcalist) {
			float volume = (4.f / 3.f) * static_cast<float>(PI) * (pca.radius * pca.radius * pca.radius);
			totalVolume += volume;
		}
		return totalVolume;
	}

	AssignmentScene::TreeNode::TreeNode(TreeNode* first, TreeNode* second, const int& cond, const GLuint& programID)
		:lchild{ first }, rchild{second}
	{
		nodetype = Internal;
		if (cond == AABB)
		{
			BoundingVolumes* BoundingV = new BoundingVolumes;
			for (int i = 0; i < 3; ++i)
			{
				BoundingV->aabbbox.min[i] = std::min(first->BV->aabbbox.min[i] + first->BV->position[i], second->BV->aabbbox.min[i] + second->BV->position[i]);
				BoundingV->aabbbox.max[i] = std::max(first->BV->aabbbox.max[i] + first->BV->position[i], second->BV->aabbbox.max[i] + second->BV->position[i]);
			}
			float halfExtentX = 0.5f * (BoundingV->aabbbox.max.x - BoundingV->aabbbox.min.x);
			float halfExtentY = 0.5f * (BoundingV->aabbbox.max.y - BoundingV->aabbbox.min.y);
			float halfExtentZ = 0.5f * (BoundingV->aabbbox.max.z - BoundingV->aabbbox.min.z);

			BoundingV->aabbbox.halfExtents = glm::vec3(halfExtentX, halfExtentY, halfExtentZ);
			BoundingV->aabbbox.PA = { 1.f,0.f,0.f };
			BoundingV->aabbbox.Y = { 0.f,1.f,0.f };
			BoundingV->aabbbox.Z = { 0.f,0.f,1.f };

			BoundingV->aabbbox.center = (BoundingV->aabbbox.max + BoundingV->aabbbox.min) / 2.f;

			BV = BoundingV;

			BV->SetupBuffersBV(AssignmentScene::AABB, programID);
		}
		else
		{
			BoundingVolumes* BoundingV = new BoundingVolumes;

			//Sphere BVH
			glm::vec3 centroid = first->BV->spherePCA.center + second->BV->spherePCA.center;
			
			centroid /= 2.f;

			float maxDistance = 0.0f;

			float distance = glm::distance(centroid, first->BV->spherePCA.center) + first->BV->spherePCA.radius;
			if (distance > maxDistance) {
				maxDistance = distance;
			}
			distance = glm::distance(centroid, second->BV->spherePCA.center ) + second->BV->spherePCA.radius;
			if (distance > maxDistance) {
				maxDistance = distance;
			}
			

			BoundingV->spherePCA.center = centroid;
			BoundingV->spherePCA.radius = maxDistance;

			BV = BoundingV;
			BV->SetupBuffersBV(AssignmentScene::PCA, programID);


		}
		
	}


	AssignmentScene::TreeNode* AssignmentScene::BottomUpTree(std::vector<TreeNode*> nodes, const int& cond)
	{
		while (nodes.size() > 1)
		{
			TreeNode* first, *second;
			
			//merging condition
			FindNodesToMerge(nodes, first, second, cond);

			TreeNode* parent = new TreeNode(first,second,cond,programID);
			parent->BVlevel = std::max(first->BVlevel, second->BVlevel) + 1;
			// Remove first & second from the vector.
			nodes.erase(std::remove(nodes.begin(), nodes.end(), first), nodes.end());
			nodes.erase(std::remove(nodes.begin(), nodes.end(), second), nodes.end());

			// Add the parent node to the vector.
			nodes.push_back(parent);
		}

		// return the root node.
		return nodes[0];
	}

	void AssignmentScene::FindNodesToMerge(std::vector<TreeNode*>& nodes, TreeNode*& first, TreeNode*& second, const int& cond) {
		// Initialize variables to track the best merging candidates
		float nearestNeighborDistance = std::numeric_limits<float>::max();
		float minCombinedVolume = std::numeric_limits<float>::max();
		float minCombinedSurfaceArea = std::numeric_limits<float>::max();

		for (size_t i = 0; i < nodes.size(); ++i) {
			for (size_t j = i + 1; j < nodes.size(); ++j) {


				// Calculate distance between bounding volumes (nearest neighbor)
				float distance = CalculateDistance(*nodes[i]->BV, *nodes[j]->BV, cond);

				// Calculate combined volume of the child bounding volumes
				float combinedVolume = CalculateCombinedVolume(*nodes[i]->BV, *nodes[j]->BV, cond);

				// Calculate combined surface area of the child bounding volumes
				float combinedSurfaceArea = CalculateCombinedSurfaceArea(*nodes[i]->BV, *nodes[j]->BV, cond);

				// Check if the current combination is better than the previous best
				if (distance < nearestNeighborDistance ||
					(distance == nearestNeighborDistance && combinedVolume < minCombinedVolume) ||
					(distance == nearestNeighborDistance && combinedVolume == minCombinedVolume && combinedSurfaceArea < minCombinedSurfaceArea)) {
					nearestNeighborDistance = distance;
					minCombinedVolume = combinedVolume;
					minCombinedSurfaceArea = combinedSurfaceArea;
					first = nodes[i];
					second = nodes[j];
				}
			}
		}
	}

	float AssignmentScene::CalculateDistance(const BoundingVolumes& volume1, const BoundingVolumes& volume2, const int& cond) {

		if (cond == AABB)
			return glm::distance(volume1.aabbbox.center, volume2.aabbbox.center);
		else
			return glm::distance(volume1.spherePCA.center, volume2.spherePCA.center);
	}

	float AssignmentScene::CalculateCombinedVolume(const BoundingVolumes& volume1, const BoundingVolumes& volume2, const int& cond)
	{
		if (cond == AABB)
		{
			float totalVolume = 0.0f;

			glm::vec3 extent = volume1.aabbbox.max - volume1.aabbbox.min;
			float volume = extent.x * extent.y * extent.z;
			extent = volume2.aabbbox.max - volume2.aabbbox.min;
			volume = extent.x * extent.y * extent.z;
			totalVolume += volume;
			
			return totalVolume;
		}
		else
		{
			float totalVolume = 0.0f;

			float volume = (4.f / 3.f) * static_cast<float>(PI) * (volume1.spherePCA.radius * volume1.spherePCA.radius * volume1.spherePCA.radius);
			volume = (4.f / 3.f) * static_cast<float>(PI) * (volume2.spherePCA.radius * volume2.spherePCA.radius * volume2.spherePCA.radius);
			totalVolume += volume;
			
			return totalVolume;
		}
	}

	float AssignmentScene::CalculateCombinedSurfaceArea(const BoundingVolumes& volume1, const BoundingVolumes& volume2, const int& cond)
	{
		if (cond == AABB)
		{
			float totalSA = 0.f;
			// Calculate the dimensions of the AABB
			float width = volume1.aabbbox.max.x - volume1.aabbbox.min.x;
			float height = volume1.aabbbox.max.y - volume1.aabbbox.min.y;
			float depth = volume1.aabbbox.max.z - volume1.aabbbox.min.z;

			// Calculate the surface area of the AABB
			float surfaceArea = 2.0f * (width * height + width * depth + height * depth);

			totalSA += surfaceArea;

			width = volume2.aabbbox.max.x - volume2.aabbbox.min.x;
			height = volume2.aabbbox.max.y - volume2.aabbbox.min.y;
			depth = volume2.aabbbox.max.z - volume2.aabbbox.min.z;

			// Calculate the surface area of the AABB
			surfaceArea = 2.0f * (width * height + width * depth + height * depth);

			totalSA += surfaceArea;
			return totalSA;
		}
		else
		{
			float totalSA = 0.f;
			float surfaceArea = 4.f * static_cast<float>(PI) * (volume1.spherePCA.radius * volume1.spherePCA.radius);
			totalSA += surfaceArea;

			surfaceArea = 4.f * static_cast<float>(PI) * (volume2.spherePCA.radius * volume2.spherePCA.radius);
			totalSA += surfaceArea;

			return totalSA;

		}
	}



}