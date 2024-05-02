/* !
@file    Intersection.h
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    10/06/2023

This file contains the declaration of namespace Assignment 2, Intersection
that encapsulates the functionality of all the inteserction test cases
required to implement for assignment 2
*//*__________________________________________________________________________*/
#pragma once
#include <glm/vec3.hpp>
#include "AssignmentScene.h"
#include <limits>

namespace Assignment2
{
	namespace Intersection
	{
		bool pointSphereIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::BoundingSphereModel& sphere);
		bool PointPlane(const AssignmentScene::PointModel& point, const AssignmentScene::PlaneModel& plane);

		int SphereFrustrumIntersection(const AssignmentScene::BoundingSphereModel& sphere,const std::vector<glm::vec4>& frustrum);
		int AABBFrustrumIntersection(const AssignmentScene::AABBC& box, const std::vector<glm::vec4>& frustrum, bool OBB);

	}
}