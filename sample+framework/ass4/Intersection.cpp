#include "Intersection.h"
#include <iostream>
/*!
@file    Intersection.cpp
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    16/07/2023

This file implements functionality for all intersection test required for
assignment 4

*//*__________________________________________________________________________*/

namespace Assignment4 {

    namespace Intersection
    {
        /*  _________________________________________________________________________ */
        /*! pointSphereIntersection
        @param const AssignmentScene::PointModel& point
        @param const AssignmentScene::BoundingSphereModel& sphere
        @return bool

        Returns true if there is intersection else false
        */
        bool pointSphereIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::BoundingSphereModel& sphere)
        {
            // if (c-p)^2 - r^2 <= 0 meaning point is on/inside the sphere
            // dot itself is distance^2
            return (glm::dot((sphere.position - point.position), (sphere.position - point.position)) - (sphere.radius * sphere.radius))
                < std::numeric_limits<float>::epsilon();
        }

        bool pointAABBIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::AABBC& aabb)
        {
            // Checking for every axis x,y,z
            // if point is smaller than min or larger than max, it is not in box
            for (int i = 0; i < 3; ++i)
                if (point.position[i] < aabb.min[i] || point.position[i] > aabb.max[i])
                    return false;

            return true;
        }
        bool AABBAABBIntersection(const AssignmentScene::AABBC& aabb1, const AssignmentScene::AABBC& aabb2)
        {
            // Checking for every axis x,y,z
           // if the whole of one's aabb is smaller(left) / larger(right) of the other, there is no intersection
            for (int i = 0; i < 3; ++i)
                if (aabb1.max[i] < aabb2.min[i] || aabb2.max[i] < aabb1.min[i])
                    return false;
            return true;
        }
    }
}