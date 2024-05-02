/* !
@file    Intersection.h
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    16/07/2023

This file contains the declaration of namespace Assignment 4, Intersection
that encapsulates the functionality of all the inteserction test cases
required to implement for assignment 4
*//*__________________________________________________________________________*/
#pragma once
#include <glm/vec3.hpp>
#include "AssignmentScene.h"
#include <limits>

namespace Assignment4
{
	namespace Intersection
	{
		bool pointSphereIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::BoundingSphereModel& sphere);
		bool pointAABBIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::AABBC& aabb);
		bool AABBAABBIntersection(const AssignmentScene::AABBC& aabb1, const AssignmentScene::AABBC& aabb2);
	}
}