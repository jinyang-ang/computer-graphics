#include "Intersection.h"
#include <iostream>
/*!
@file    Intersection.cpp
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    10/06/2023

This file implements functionality for all intersection test required for
assignment 2

*//*__________________________________________________________________________*/

namespace Assignment2 {

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

        /*  _________________________________________________________________________ */
        /*! SphereFrustrumIntersection
        @param const AssignmentScene::BoundingSphereModel& sphere
        @param const std::vector<glm::vec4>& frustrum
        @return bool

        Returns 0 is intersecting
                1 if inside
                2 if outside
        */
        int SphereFrustrumIntersection(const AssignmentScene::BoundingSphereModel& sphere, const std::vector<glm::vec4>& frustrum)
        {
            for (const auto& plane : frustrum)
            {
                float distance = glm::dot(glm::vec3(plane), sphere.position) - plane.w;
                if (distance > -sphere.radius && distance < sphere.radius)
                {
                    return 0;
                }
                if (distance < -sphere.radius)
                {
                    continue;
                }
                else
                    return 2;
            }
            return 1;
        }

        /*  _________________________________________________________________________ */
        /*! AABBFrustrumIntersection
        @param const AssignmentScene::AABBC& box
        @param const std::vector<glm::vec4>& frustrum
        @return bool OBB

        Returns 0 is intersecting
                1 if inside
                2 if outside
        */
        int AABBFrustrumIntersection(const AssignmentScene::AABBC& box, const std::vector<glm::vec4>& frustrum, bool OBB)
        {

            for (const auto& plane : frustrum)
            {

                glm::vec3 planeeqn(0.f);
                
                if (OBB)
                {
                    glm::vec3 x = { box.PA.x,box.Y.x,box.Z.x };
                    glm::vec3 y = { box.PA.y,box.Y.y,box.Z.y };
                    glm::vec3 z = { box.PA.z,box.Y.z,box.Z.z };
                    planeeqn = { glm::dot(glm::vec3(plane),x),glm::dot(glm::vec3(plane),y) ,glm::dot(glm::vec3(plane),z) };

                }
                else
                    planeeqn = plane;
            
                int s0 = planeeqn.x < 0.f ? -1 : 1;
                int s1 = planeeqn.y < 0.f ? -1 : 1;
                int s2 = planeeqn.z < 0.f ? -1 : 1;

                AssignmentScene::PointModel pointPositive;
                pointPositive.position = box.center + (s0 * box.halfExtents.x * glm::abs(box.PA)) + (s1 * box.halfExtents.y * glm::abs(box.Y))
                    + (s2 * box.halfExtents.z * glm::abs(box.Z));


                float distance = glm::dot(glm::vec3(planeeqn), pointPositive.position) - plane.w;
                // Pvtx is outside plane
                if (distance > 0.f)
                {
                    //check if Nvtx is also outside
                    AssignmentScene::PointModel pointNegative;
                    pointNegative.position = box.center - (s0 * box.halfExtents.x * glm::abs(box.PA)) - (s1 * box.halfExtents.y * glm::abs(box.Y))
                        - (s2 * box.halfExtents.z * glm::abs(box.Z));

                    distance = glm::dot(glm::vec3(planeeqn), pointNegative.position) - plane.w;
                    // Nvtx also outside box outside
                    if (distance > 0.f)
                        return 2;
                    else // Nvtx is inside, intersecting
                        return 0;

                }
                else // Ptv is inside, meaning box inside
                    continue;

            }
            return 1;
        }
    }
}