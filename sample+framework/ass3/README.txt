A.    When application is launched, there are multiple options that the user can select.
	1. User can enable wireframe for model, model's bounding volume and BVH respectively
	2. User can enable top-down or bottom-up BVH
	3. User can change the respective model's bounding volume (AABB,PCA sphere and OBB)
	4. User can navigate around the scene using WSAD to move in,out,left and right respectively. User can also
	   perform camera look-around by holding down the right click and moving the cursor
	
B.      If the mesh of the models are seperated to multiple lines it would crash the program.

C.	All requirement parts of the assignment has been completed

D.	None

E.	Source Codes:
	AssignmentScene.h 	: sample+framework\ass3\AssignmentScene.h
	AssignmentScene.cpp 	: sample+framework\ass3\AssignmentScene.cpp
	Intersection.h		: sample+framework\ass3\Intersection.h	
	Intersection.cpp	: sample+framework\ass3\Intersection.cpp
	
	Shaders:
	Ass2.vert			: sample+framework\Common\shaders\Ass2.vert
	Ass2.frag			: sample+framework\Common\shaders\Ass2.frag

	Model file:
	suzanne.obj			: sample+framework\Common\models\suzanne.obj
	ogre.obj			: sample+framework\Common\models\ogre.obj
	dodecahedron.obj		: sample+framework\Common\models\dodecahedron.obj
	head.obj			: sample+framework\Common\models\head.obj
	rock.obj			: sample+framework\Common\models\rock.obj
	octahedron.obj			: sample+framework\Common\models\octahedron.obj
	gourd.obj			: sample+framework\Common\models\gourd.obj
	violin_case.obj			: sample+framework\Common\models\violin_case.obj

	Scene file:
	ass3.scn			: sample+framework\Common\Ass3

	All intersection codes are in Intersection.h and Intersection.cpp,
 	code related to scene creation and BVH are in AssignmentScene.h and AssignmentScene.cpp


F.	OS 				: Microsoft Windows 11 Home
	GPU 				: NVIDIA GeForce GTX 1650 Ti/PC
	OpenGL Driver Version 	: 27.20.100.9365

G.	9 hours / week

H.	Researching on what method works best for merging BVs.
	Nearest Neighbour 			: produce a relatively balanced structure with least overlapping
	Minimum Combined Child Volume		: minimizes overall size of BVH
	Minimum Combined Child Surface Area	: better for future implementations involving surface area intersections
	As such this project uses a combination of all 3 test where it choses the best combination out of them