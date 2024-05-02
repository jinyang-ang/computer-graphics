/* !
@file    Intersection.h
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    20/05/2023

This file contains the declaration of namespace Assignment 1, Intersection
that encapsulates the functionality of all the inteserction test cases
required to implement for assignment 1
*//*__________________________________________________________________________*/

#include <glm/vec3.hpp>
#include "AssignmentScene.h"
#include <limits>

namespace Assignment1
{
	namespace Intersection
	{
		// Basice Intersections
		bool SphereSphereIntersection(const AssignmentScene::BoundingSphereModel& sphere1, const AssignmentScene::BoundingSphereModel& sphere2);
		bool AABBAABBIntersection(const AssignmentScene::AABBModel& aabb1, const AssignmentScene::AABBModel& aabb2);
		bool SphereAABBIntersection(const AssignmentScene::BoundingSphereModel& sphere, const AssignmentScene::AABBModel& aabb);

		// Point Intersections
		bool PointPlane(const AssignmentScene::PointModel& point, const AssignmentScene::PlaneModel& plane);
		bool pointTriangleIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::TriangleModel& triangle);
		bool pointSphereIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::BoundingSphereModel& sphere);
		bool pointAABBIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::AABBModel& aabb);


		// Ray Intersections
		std::pair<bool, float> RayPlaneIntersection(const AssignmentScene::RayModel& ray, const AssignmentScene::PlaneModel& plane, const bool forAABB = false);
		bool RayTriangleIntersection(const AssignmentScene::RayModel& ray, const AssignmentScene::TriangleModel& triangle);
		bool RaySphereIntersection(const AssignmentScene::RayModel& ray, const AssignmentScene::BoundingSphereModel& sphere);
		bool RayAABBIntersection(const AssignmentScene::RayModel& ray, const AssignmentScene::AABBModel& aabb);
		int ComputeOC(const AssignmentScene::AABBModel& aabb, const AssignmentScene::PointModel& point);

		// Plane Intersections
		bool PlaneSphereIntersection(const AssignmentScene::PlaneModel& plane, const AssignmentScene::BoundingSphereModel& sphere);
		bool PlaneAABBIntersection(const AssignmentScene::PlaneModel& plane, const AssignmentScene::AABBModel& aabb);

		enum OutCode
		{
			LEFT = 1,	// 2^0
			RIGHT = 2,	// 2^1
			BOTTOM = 4,	// 2^2
			TOP = 8,	// 2^3
			BACK = 16,	// 2^4
			FRONT = 32	// 2^5
		};
	}
}