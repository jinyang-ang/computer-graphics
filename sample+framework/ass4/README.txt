A.    When application is launched, there are multiple options that the user can select.
	1. User can enable wireframe for model and model's bounding volume respectively
	2. User can enable Oct-tree which will include additional options exclusive for Octree
	3. User can enable KD-tree which will include additional options exclusive for KD-tree
	4. User can navigate around the scene using WSAD to move in,out,left and right respectively. User can also
	   perform camera look-around by holding down the right click and moving the cursor
	
B.      If termination condition is less than 1 the program will crash

C.	All requirement parts of the assignment has been completed

D.	None

E.	Source Codes:
	AssignmentScene.h 	: sample+framework\ass4\AssignmentScene.h
	AssignmentScene.cpp 	: sample+framework\ass4\AssignmentScene.cpp
	Intersection.h		: sample+framework\ass4\Intersection.h	
	Intersection.cpp	: sample+framework\ass4\Intersection.cpp
	
	Shaders:
	Ass2.vert			: sample+framework\Common\shaders\Ass2.vert
	Ass2.frag			: sample+framework\Common\shaders\Ass2.frag

	Model file:			All objects files are inside the following folders
	ppsection4			: sample+framework\Common\models\ppsection4
	ppsection5			: sample+framework\Common\models\ppsection5
	ppsection6			: sample+framework\Common\models\ppsection6

	Scene file:
	Section4.txt			: sample+framework\Common\models\Section4.txt
	Section5.txt			: sample+framework\Common\models\Section5.txt
	Section6.txt			: sample+framework\Common\models\Section6.txt


	All intersection codes are in Intersection.h and Intersection.cpp,
 	code related to scene creation and BVH are in AssignmentScene.h and AssignmentScene.cpp


F.	OS 				: Microsoft Windows 11 Home
	GPU 				: NVIDIA GeForce GTX 1650 Ti/PC
	OpenGL Driver Version 	: 27.20.100.9365

G.	9 hours / week

H.	Researching on creating KD-tree dynamically