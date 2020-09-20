/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Tuesday, October 24, 2017 - 17:17:44
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labraytracer/phongmaterial.h>
#include <labraytracer/util.h>

namespace inviwo {

PhongMaterial::PhongMaterial(const vec3& color, const double reflectance, const double shininess,
    const vec3& ambientMaterialColor, const vec3& diffuseMaterialColor, const vec3& specularMatierialColor) 
    : Material(color, reflectance) {

    constexpr double LightIntensity = 100.0;

    shininess_ = shininess;
    ambientMaterialColor_   = Util::scalarMult(LightIntensity, ambientMaterialColor);
    diffuseMaterialColor_   = Util::scalarMult(LightIntensity, diffuseMaterialColor);
    specularMatierialColor_ = Util::scalarMult(LightIntensity, specularMatierialColor);
}

vec4 PhongMaterial::shade(const RayIntersection& intersection, const Light& light) const {
    // get normal and light direction
    vec3 N = intersection.getNormal();
    vec3 L = glm::normalize(light.getPosition() - intersection.getPosition());

    double cosNL = std::max(double(dot(N, L)), double(0));

    vec3 V = glm::normalize(-intersection.getRay().getDirection()); // The view vector V is the direction of the ray which intersects the object.
    vec3 R = normalize(2 * cosNL * N - L); // reflection vector
    double cosVR = std::max(double(dot(V, R)), double(0));

    // distance from light source from the point of intersection
    double dist = length(light.getPosition() - intersection.getPosition());
    double attentuation = 1 / pow(dist, 2);

    vec3 diffuseLightColor  = light.getDiffuseColor() * attentuation;
    vec3 specularLightColor = light.getSpecularColor() * attentuation;

    vec3 c_diffuse = diffuseLightColor * diffuseMaterialColor_ * cosNL;
    vec3 c_specular = specularLightColor * specularMatierialColor_ * glm::pow(cosVR, shininess_);

    vec3 color = c_diffuse + c_specular;

    return vec4(color, 1.0);

    // Programming Task 2: Extend this method.
    // This method currently implements a Lambert's material with ideal
    // diffuse reflection.
    // Your task is to implement a Phong shading model.
    //
    // Hints:
    //
    // 1. This function should return the sum of diffuse and specular parts (no ambient part)
    // 2. The used light color for each type (diffuse/specular) from the light source 
    //    is the light color divided by the quadratic distance of the light source from 
    //    the point of intersection. (quadratic falloff)
    // 3. The view vector V is the direction of the ray which intersects the object.
    // 4. The rest of the terms as per the slides are as follows
    // 5. You have to take into account shininess_ (p), material colors, light colors 
    //    light, view, reflection and normal vector.
    //    
    //
}

} // namespace
