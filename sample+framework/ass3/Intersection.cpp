#include "Intersection.h"
#include <iostream>
/*!
@file    Intersection.cpp
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    10/06/2023

This file implements functionality for all intersection test required for
assignment 2

*//*__________________________________________________________________________*/

namespace Assignment3 {

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
    }
}