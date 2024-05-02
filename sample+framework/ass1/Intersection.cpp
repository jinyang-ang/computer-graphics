#include "Intersection.h"
#include <iostream>
/*!
@file    Intersection.cpp
@author	 Ang Jin Yang (jinyang.ang@digipen.edu)
@date    20/05/2023

This file implements functionality for all intersection test required for
assignment 1

*//*__________________________________________________________________________*/

namespace Assignment1 {

    namespace Intersection
    {
        /*  _________________________________________________________________________ */
        /*! SphereSphereIntersection
        @param const AssignmentScene::BoundingSphereModel& sphere1
        @param const AssignmentScene::BoundingSphereModel& sphere2
        @return bool

        Returns true if there is intersection else false
       */
        bool SphereSphereIntersection(const AssignmentScene::BoundingSphereModel& sphere1, const AssignmentScene::BoundingSphereModel& sphere2)
        {
            // if (C1 - C0)^2 <= (r1 + r0)^2 the spheres are intersecting
            return (glm::dot((sphere1.position - sphere2.position), (sphere1.position - sphere2.position))
                - ((sphere1.radius + sphere2.radius) * (sphere1.radius + sphere2.radius))) < std::numeric_limits<float>::epsilon();
        }

        /*  _________________________________________________________________________ */
        /*! AABBAABBIntersection
        @param const AssignmentScene::AABBModel& aabb1
        @param const AssignmentScene::AABBModel& aabb2
        @return bool

        Returns true if there is intersection else false
        */
        bool AABBAABBIntersection(const AssignmentScene::AABBModel& aabb1, const AssignmentScene::AABBModel& aabb2)
        {
            // Checking for every axis x,y,z
           // if the whole of one's aabb is smaller(left) / larger(right) of the other, there is no intersection
            for (int i = 0; i < 3; ++i)
                if (aabb1.max[i] < aabb2.min[i] || aabb2.max[i] < aabb1.min[i])
                    return false;
            return true;
        }

        /*  _________________________________________________________________________ */
        /*! SphereAABBIntersection
        @param const AssignmentScene::BoundingSphereModel& sphere
        @param const AssignmentScene::AABBModel& aabb
        @return bool

        Returns true if there is intersection else false
        */
        bool SphereAABBIntersection(const AssignmentScene::BoundingSphereModel& sphere, const AssignmentScene::AABBModel& aabb)
        {
            // Calculate the closest point on the AABB to the sphere
            // If x is less than min, it will be clamped to min. 
            // If x is greater than max, it will be clamped to max. 
            // Otherwise, if x is already within the range, it will remain unchanged.
            AssignmentScene::PointModel point;
            point.position = glm::clamp(sphere.position, aabb.min, aabb.max);
            // Calculate Point vs Sphere
            return pointSphereIntersection(point, sphere);
        }

        /*  _________________________________________________________________________ */
        /*! PointPlane
        @param const AssignmentScene::PointModel& point
        @param const AssignmentScene::PlaneModel& plane
        @return bool

        Returns true if there is intersection else false
        */
        bool PointPlane(const AssignmentScene::PointModel& point, const AssignmentScene::PlaneModel& plane)
        {
            // Calculate the plane's normal
            glm::vec3 normal = glm::normalize(glm::cross(plane.P1 - plane.P0, plane.P2 - plane.P0));
            // Compute the distance from the point to the plane
            float distance = glm::dot(point.position - plane.P0, normal);
            // Check if the point is on the plane == 0 (within a tolerance)
            return std::abs(distance) < std::numeric_limits<float>::epsilon();
        }

        /*  _________________________________________________________________________ */
        /*! pointTriangleIntersection
        @param const AssignmentScene::PointModel& point
        @param const AssignmentScene::TriangleModel& triangle
        @return bool

        Returns true if there is intersection else false
        */
        bool pointTriangleIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::TriangleModel& triangle)
        {
            // check if point is on the triangle's plane, using PointPlane func
            // if not even on the triangle's plane then no need to continue test..
            AssignmentScene::PlaneModel plane;
            plane.P0 = triangle.v0;
            plane.P1 = triangle.v1;
            plane.P2 = triangle.v2;
            if (!PointPlane(point, plane))
                return false;

            // Point is on triangle's plane, check whether point is INSIDE triangle..
            glm::vec3 edge1 = triangle.v1 - triangle.v0;
            glm::vec3 edge2 = triangle.v2 - triangle.v0;
            glm::vec3 normal = glm::cross(edge1, edge2);
            glm::vec3 vec = point.position - triangle.v0;

            // u is from v0 to v1 
            // if point is at v2, vec cross edge2 = 0 vector, u = 0
            // glm::dot(edge1, glm::cross(edge2, normal) > 0 if edge 1 points in same dir as the cross vec of edge2 and norm
            float u = glm::dot(glm::cross(vec, edge2), normal) / glm::dot(edge1, glm::cross(edge2, normal));
            // v is from v0 to v2
            float v = glm::dot(glm::cross(edge1, vec), normal) / glm::dot(edge1, glm::cross(edge2, normal));

            bool intersected = (u >= 0.0f && v >= 0.0f && u + v <= 1.0f);

            return intersected;
        }

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
        /*! pointAABBIntersection
        @param const AssignmentScene::PointModel& point
        @param const AssignmentScene::AABBModel& aabb
        @return bool

        Returns true if there is intersection else false
        */
        bool pointAABBIntersection(const AssignmentScene::PointModel& point, const AssignmentScene::AABBModel& aabb)
        {
            // Checking for every axis x,y,z
            // if point is smaller than min or larger than max, it is not in box
            for (int i = 0; i < 3; ++i)
                if (point.position[i] < aabb.min[i] || point.position[i] > aabb.max[i])
                    return false;

            return true;
        }
        /*  _________________________________________________________________________ */
        /*! RayPlaneIntersection
        @param const AssignmentScene::RayModel& ray
        @param const AssignmentScene::PlaneModel& plane
        @param const bool forAABB
        @return std::pair<bool, float>

        Returns true if there is intersection else false together with the time
        of intersection
        */
        std::pair<bool, float> RayPlaneIntersection(const AssignmentScene::RayModel& ray, const AssignmentScene::PlaneModel& plane, const bool forAABB)
        {
            // if ray starts on the plane
            AssignmentScene::PointModel point;
            point.position = ray.origin;
            if (PointPlane(point, plane))
                return std::make_pair(true, 0.f);

            // Calculate the plane's normal
            glm::vec3 normal{};
            if (forAABB)
                normal = plane.data;
            else
                normal = glm::normalize(glm::cross(plane.P1 - plane.P0, plane.P2 - plane.P0));

            // Check if ray is parallel to plane, n.d == 0 is parallel/ orthogonal which is also denom for next step
            // denominator might be negative
            float denominator = glm::dot(glm::normalize(ray.direction), normal);
            if (std::abs(denominator) <= std::numeric_limits<float>::epsilon())
                return std::make_pair(false, -1.f);

            // calculate t using formula from slides..
            float t = glm::dot(plane.P0 - ray.origin, normal) / denominator;
            // ray travelling opp dir
            if (t < 0.f)
                return std::make_pair(false, t);

            return std::make_pair(true, t);

        }

        /*  _________________________________________________________________________ */
        /*! RayTriangleIntersection
        @param const AssignmentScene::RayModel& ray
        @param const AssignmentScene::TriangleModel& triangle
        @return bool

        Returns true if there is intersection else false
        */
        bool RayTriangleIntersection(const AssignmentScene::RayModel& ray, const AssignmentScene::TriangleModel& triangle)
        {

            AssignmentScene::PlaneModel plane;
            plane.P0 = triangle.v0;
            plane.P1 = triangle.v1;
            plane.P2 = triangle.v2;
            std::pair<bool, float> temp = RayPlaneIntersection(ray, plane);
            if (temp.first)
            {
                // compute position at time t, P = S + Dt
                AssignmentScene::PointModel point;
                point.position = ray.origin + glm::normalize(ray.direction) * temp.second;
                return pointTriangleIntersection(point, triangle);
            }
            return false;

        }

        /*  _________________________________________________________________________ */
        /*! RaySphereIntersection
        @param const AssignmentScene::RayModel& ray
        @param const AssignmentScene::BoundingSphereModel& sphere
        @return bool

        Returns true if there is intersection else false
        */
        bool RaySphereIntersection(const AssignmentScene::RayModel& ray, const AssignmentScene::BoundingSphereModel& sphere)
        {
            // Trivial acceptance, when the starting point is inside the sphere
            AssignmentScene::PointModel point;
            point.position = ray.origin;
            if (pointSphereIntersection(point, sphere))
                return true;

            // Starting point is outside the sphere, check direction of ray if its towards the sphere
            // Trivial rejection, if direction is pointing opp side, the dot product will be negative
            float projection = glm::dot((sphere.position - ray.origin), glm::normalize(ray.direction));
            if (projection < 0.f)
                return false;

            // Ray is travelling towards sphere's direction but may miss the sphere, 
            // calculate vector N to determine if ray is intersecting or passing by.
            // Using squared distance to remove use of sqrt..
            float Nsquare = glm::dot((sphere.position - ray.origin), (sphere.position - ray.origin)) - (projection * projection);

            // if N is smaller than radius, ray intersecting else passing by
            return (Nsquare < (sphere.radius * sphere.radius));
        }

        /*  _________________________________________________________________________ */
        /*! RayAABBIntersection
        @param const AssignmentScene::RayModel& ray
        @param const AssignmentScene::AABBModel& aabb
        @return bool

        Returns true if there is intersection else false
        */
        bool RayAABBIntersection(const AssignmentScene::RayModel& ray, const AssignmentScene::AABBModel& aabb)
        {

            glm::vec3 P0 = ray.origin;
            glm::vec3 P1 = ray.origin + ray.direction * 40.f;
            AssignmentScene::PointModel point;
            float t = 0.f;
            while (true) {
                // Compute region codes for both endpoints
                point.position = P0;
                int outcodeP0 = ComputeOC(aabb,point);
                point.position = P1;
                int outcodeP1 = ComputeOC(aabb, point);

                // Trivial rejection: if both endpoints are outside the AABB, reject the intersection
                if ((outcodeP0 & outcodeP1) != 0)
                    return false;

                // Trivial acceptance: if both endpoints are inside the AABB, accept the intersection
                if ((outcodeP0 | outcodeP1) == 0)
                    return true;

                // Compute intersection points
                glm::vec3 P0_prime = P0;
                glm::vec3 P1_prime = P1;

                // Clip against the left boundary
                if (outcodeP0 & LEFT) {
                    t = (aabb.min.x - P0.x) / (P1.x - P0.x);
                    P0_prime = P0 + t * (P1 - P0);
                }
                else if (outcodeP1 & LEFT) {
                    t = (aabb.min.x - P1.x) / (P0.x - P1.x);
                    P1_prime = P1 + t * (P0 - P1);
                }

                // Clip against the right boundary
                if (outcodeP0 & RIGHT) {
                    t = (aabb.max.x - P0.x) / (P1.x - P0.x);
                    P0_prime = P0 + t * (P1 - P0);
                }
                else if (outcodeP1 & RIGHT) {
                    t = (aabb.max.x - P1.x) / (P0.x - P1.x);
                    P1_prime = P1 + t * (P0 - P1);
                }

                // Clip against the bottom boundary
                if (outcodeP0 & BOTTOM) {
                    t = (aabb.min.y - P0.y) / (P1.y - P0.y);
                    P0_prime = P0 + t * (P1 - P0);
                }
                else if (outcodeP1 & BOTTOM) {
                    t = (aabb.min.y - P1.y) / (P0.y - P1.y);
                    P1_prime = P1 + t * (P0 - P1);
                }

                // Clip against the top boundary
                if (outcodeP0 & TOP) {
                    t = (aabb.max.y - P0.y) / (P1.y - P0.y);
                    P0_prime = P0 + t * (P1 - P0);
                }
                else if (outcodeP1 & TOP) {
                    t = (aabb.max.y - P1.y) / (P0.y - P1.y);
                    P1_prime = P1 + t * (P0 - P1);
                }

               
                // Clip against the back boundary
                if (outcodeP0 & BACK) {
                    t = (aabb.min.z - P0.z) / (P1.z - P0.z);
                    P0_prime = P0 + t * (P1 - P0);
                }
                else if (outcodeP1 & BACK) {
                    t = (aabb.min.z - P1.z) / (P0.z - P1.z);
                    P1_prime = P1 + t * (P0 - P1);
                }

                // Clip against the front boundary
                if (outcodeP0 & FRONT) {
                    t = (aabb.max.z - P0.z) / (P1.z - P0.z);
                    P0_prime = P0 + t * (P1 - P0);
                }
                else if (outcodeP1 & FRONT) {
                    t = (aabb.max.z - P1.z) / (P0.z - P1.z);
                    P1_prime = P1 + t * (P0 - P1);
                }

                // causing it to loop forever..
                // when end point is intersecting
                if (t == 1.f)
                    return false;
     
                // Update the ray
                P0 = P0_prime;
                P1 = P1_prime;
            }
        }
        /*  _________________________________________________________________________ */
        /*! ComputeOC
        @param const AssignmentScene::AABBModel& ray
        @param const AssignmentScene::PointModel& point
        @return int

        Returns the outcode computed
        */

        int ComputeOC(const AssignmentScene::AABBModel& aabb, const AssignmentScene::PointModel& point)
        {
            int OC = 0;
            // if point is inside aabb return OC = 0
            if (pointAABBIntersection(point, aabb))
                return OC;

            // Point is outside aabb, find point w.r.t each plane,
            // use min/max instead of computing each plane's normal..
            if (point.position.x < aabb.min.x)
                OC |= OutCode::LEFT;
            else if (point.position.x > aabb.max.x)
                OC |= OutCode::RIGHT;

            if (point.position.y < aabb.min.y)
                OC |= OutCode::BOTTOM;
            else if (point.position.y > aabb.max.y)
                OC |= OutCode::TOP;

            if (point.position.z < aabb.min.z)
                OC |= OutCode::BACK;
            else if (point.position.z > aabb.max.z)
                OC |= OutCode::FRONT;

            return OC;
        }

        /*  _________________________________________________________________________ */
        /*! PlaneSphereIntersection
        @param const AssignmentScene::PlaneModel& plane
        @param const AssignmentScene::BoundingSphereModel& sphere
        @return bool

        Returns true if there is intersection else false
        */
        bool PlaneSphereIntersection(const AssignmentScene::PlaneModel& plane, const AssignmentScene::BoundingSphereModel& sphere)
        {
            // Calculate the plane's normal
            glm::vec3 normal = glm::normalize(glm::cross(plane.P1 - plane.P0, plane.P2 - plane.P0));
            // Compute the distance from the point to the plane
            float distance = glm::dot(sphere.position - plane.P0, normal);
            // only care about the distance not the sign
            return std::abs(distance) <= sphere.radius;
        }

        /*  _________________________________________________________________________ */
        /*! PlaneAABBIntersection
        @param const AssignmentScene::PlaneModel& plane
        @param const AssignmentScene::AABBModel& aabb
        @return bool

        Returns true if there is intersection else false
        */
        bool PlaneAABBIntersection(const AssignmentScene::PlaneModel& plane, const AssignmentScene::AABBModel& aabb)
        {
            // check similar to aabb vs sphere..
            // check if distance between aabb centre from plane normal < aabb "radius" from plane normal
            glm::vec3 aabbcenterPos = (aabb.min + aabb.max) * 0.5f;
            float distance = glm::dot(plane.P0 - aabbcenterPos, glm::normalize(glm::cross(plane.P1 - plane.P0, plane.P2 - plane.P0)));
            glm::vec3 halfExtent = (aabb.max - aabb.min) * 0.5f;
            // Need check equal, equal when plane is on the edge of the aabb, z value of plane is at min/max
            // only care about the distance not the sign
            return glm::abs(distance) <= glm::abs(glm::dot(halfExtent, glm::normalize(glm::cross(plane.P1 - plane.P0, plane.P2 - plane.P0))));
        }



    }
}