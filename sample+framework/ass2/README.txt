A.    When application is launched, there are multiple options that the user can select.
	1. User can select which models to render
	2. User can navigate around the scene using WSAD to move in,out,left and right respectively. User can also
	   perform camera look-around by holding down the right click and moving the cursor
	3. User can enable and disable wireframe for both models and their respective bounding volumes using
	   the checkbox

B.      If the mesh of the models are seperated to multiple lines it would crash the program.
	For ogre, in OBB it only detects top plane intersection properly if the user uses arrow key movements
	and bottom plane properly using right hold mouse interaction
	

C.	All requirement parts of the assignment has been completed

D.	None

E.	Source Codes:
	AssignmentScene.h 	: sample+framework\ass2\AssignmentScene.h
	AssignmentScene.cpp 	: sample+framework\ass2\AssignmentScene.cpp
	Intersection.h		: sample+framework\ass2\Intersection.h	
	Intersection.cpp	: sample+framework\ass2\Intersection.cpp
	
	Shaders:
	Ass2.vert			: sample+framework\Common\shaders\Ass2.vert
	Ass2.frag			: sample+framework\Common\shaders\Ass2.frag

	Model file:
	suzanne.obj			: sample+framework\Common\models\suzanne.obj
	ogre.obj			: sample+framework\Common\models\ogre.obj
	dodecahedron.obj		: sample+framework\Common\models\dodecahedron.obj
	head.obj			: sample+framework\Common\models\head.obj

	Scene file:
	ass2.scn			: sample+framework\Common\Ass2

	All intersection codes are in Intersection.h and Intersection.cpp, code related to
	scene creation are in AssignmentScene.h and AssignmentScene.cpp


F.	OS 				: Microsoft Windows 11 Home
	GPU 				: NVIDIA GeForce GTX 1650 Ti/PC
	OpenGL Driver Version 	: 27.20.100.9365

G.	9 hours / week

H.	Researching on how to generate vertices for a sphere and
	PCA method for sphere