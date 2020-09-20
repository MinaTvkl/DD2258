/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Tuesday, October 17, 2017 - 10:24:56
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labraytracer/sphere.h>
#include <labraytracer/util.h>

namespace inviwo {

Sphere::Sphere(const vec3& center, const double& radius) {
    center_ = center;
    radius_ = radius;
}

bool Sphere::closestIntersection(const Ray& ray, double maxLambda,
                                 RayIntersection& intersection) const {
    // Programming TASK 1: implement this method
    // Your code should compute the intersection between a ray and a sphere;

    // If you detect an intersection, the return type should look similar to this:
    // if(rayIntersectsSphere)
    //{
    //  intersection = RayIntersection(ray,shared_from_this(),lambda,normalVec,uvw);
    //  return true;
    //}
    //
    // Hints:
    // lambda is the distance form the ray origin an the intersection point.
    // Ray origin p_r : ray.getOrigin()
    // Ray direction t_r : ray.getDirection()
    // If you need the intersection point, use ray.pointOnRay(lambda)
    // You can ignore the uvw (texture coordinates)

    const vec3 A = ray.getOrigin(); // ray origin
    const vec3 B = ray.getDirection(); // ray direction
    const vec3 C = center_; // sphere center
    float r = radius_; // sphere radius
    
    float a = dot(B, B);
    float b = 2 * dot(B, (A - C));
    float c = dot((A - C), (A - C)) - r*r;
    
    float discriminant = b*b - 4*a*c;
    
    if (discriminant < 0.0) { // does not hit sphere
        return false;
    }
    
    float lambda1 = ((-b + sqrt(discriminant)) / 2*a);
    float lambda2 = ((-b - sqrt(discriminant)) / 2*a);
    float lambda;
    
    if (lambda1 >= 0.0 && lambda2 <= 0.0) {
        lambda = lambda1;
    } else if (lambda2 >= 0.0 && lambda1 <= 0.0) {
        lambda = lambda2;
    } else if (lambda1 >= 0.0 && lambda2 >= 0.0) {
        lambda = std::min(lambda1, lambda2);
    } else {
        return false;
    }
    
    if (lambda + Util::epsilon > maxLambda) { //from plane.cpp
        return false;
    }

    const vec3 p = ray.pointOnRay(lambda); // point where ray hits sphere
    const vec3 normal = p - C; // the normal, point minus center
    const vec3 uvw(0, 0, 0); // from plane.cpp
    
    intersection = RayIntersection(ray, shared_from_this(), lambda, normal, uvw);
    return true;
}

bool Sphere::anyIntersection(const Ray& ray, double maxLambda) const {
    RayIntersection temp;
    return closestIntersection(ray, maxLambda, temp);
}

void Sphere::drawGeometry(std::shared_ptr<BasicMesh> mesh,
                          std::vector<BasicMesh::Vertex>& vertices) const {
    auto indexBuffer = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);

    int lat = 8;
    int lon = 10;

    for (int i = 0; i < lat - 1; i++) {
        float theta1 = float(i * M_PI) / (lat - 1);
        float theta2 = float((i + 1) * M_PI) / (lat - 1);

        for (int j = 0; j < lon - 1; j++) {
            float phi1 = float(j * 2 * M_PI) / (lon - 1);
            float phi2 = float((j + 1) * 2 * M_PI) / (lon - 1);

            vec3 v1 = vec3(radius_ * sin(theta1) * cos(phi1), radius_ * sin(theta1) * sin(phi1),
                           radius_ * cos(theta1)) +
                      center_;
            vec3 v2 = vec3(radius_ * sin(theta2) * cos(phi1), radius_ * sin(theta2) * sin(phi1),
                           radius_ * cos(theta2)) +
                      center_;
            vec3 v3 = vec3(radius_ * sin(theta2) * cos(phi2), radius_ * sin(theta2) * sin(phi2),
                           radius_ * cos(theta2)) +
                      center_;

            Util::drawLineSegment(v1, v2, vec4(0.2, 0.2, 0.2, 1), indexBuffer.get(), vertices);
            Util::drawLineSegment(v2, v3, vec4(0.2, 0.2, 0.2, 1), indexBuffer.get(), vertices);
        }
    }
}

}  // namespace inviwo
