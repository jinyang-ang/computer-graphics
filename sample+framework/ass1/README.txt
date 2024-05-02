A.    When application is launched, there are multiple options that the user can select.
	1. User can select whether to render the primitives as wireframe or filled by clicking on Enable Wireframe
	2. User can navigate around the scene using WSAD to move front,back,left and right respectively. User can also
	   perform camera look-around by holding down the right click and moving the cursor
	3. User can select the different cases of intersection test that they wish to see using the Imgui checkbox
	   Namely they are : Basic Intersection Test
				   Point-Based Intersection Test
				   Ray-Based Intersection Test
				   Plane-Based Intersection Test

B.    1. If the mesh of the models are seperated to multiple lines it would crash the program.

	2. When selecting Ray-Based intersection test, it only test for ray and the respective models and not
	   respective models against one another. For example, sphere and aabb test will not work in
	   Ray-Based Intersection Test but will work in Basic Intersection Test

C.	All requirement parts of the assignment has been completed

D.	None

E.	Source Codes:
	AssignmentScene.h 	: sample+framework\ass1\AssignmentScene.h
	AssignmentScene.cpp 	: sample+framework\ass1\AssignmentScene.cpp
	Intersection.h		: sample+framework\ass1\Intersection.h	
	Intersection.cpp		: sample+framework\ass1\Intersection.cpp
	
	Shaders:
	Ass1.vert			: sample+framework\Common\shaders\Ass1.vert
	Ass1.frag			: sample+framework\Common\shaders\Ass1.frag

	Mesh file:
	Point.msh			: sample+framework\Common\Ass1
	Plane.msh			: sample+framework\Common\Ass1
	Triangle.msh		: sample+framework\Common\Ass1
	BoundingSphere.msh	: sample+framework\Common\Ass1
	AABB.msh			: sample+framework\Common\Ass1
	Ray.msh			: sample+framework\Common\Ass1

	Scene file:
	ass1.scn			: sample+framework\Common\Ass1

	All intersection codes are in Intersection.h and Intersection.cpp, code related to
	scene creation are in AssignmentScene.h and AssignmentScene.cpp


F.	OS 				: Microsoft Windows 11 Home
	GPU 				: NVIDIA GeForce GTX 1650 Ti/PC
	OpenGL Driver Version 	: 27.20.100.9365

G.	9 hours / week

H.	Researching on how to generate vertices for a sphere