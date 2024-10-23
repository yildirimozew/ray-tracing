#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
/*may need to add pthread*/

typedef unsigned char RGB[3];

struct Ray{
    parser::Vec3f origin;
    parser::Vec3f direction;
};

parser::Vec3f cross_product(parser::Vec3f a, parser::Vec3f b){
    parser::Vec3f c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

parser::Vec3f multipy(parser::Vec3f a, parser::Vec3f b){
    parser::Vec3f c;
    c.x = a.x * b.x;
    c.y = a.y * b.y;
    c.z = a.z * b.z;
    return c;
}

parser::Vec3f multipy_with_constant(parser::Vec3f a, float b){
    parser::Vec3f c;
    c.x = a.x * b;
    c.y = a.y * b;
    c.z = a.z * b;
    return c;
}

parser::Vec3f subtract(parser::Vec3f a, parser::Vec3f b){
    parser::Vec3f c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
}

parser::Vec3f add(parser::Vec3f a, parser::Vec3f b){
    parser::Vec3f c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

float dot(parser::Vec3f a, parser::Vec3f b){
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

parser::Vec3f normalize(parser::Vec3f a){
    float length = pow(a.x * a.x + a.y * a.y + a.z * a.z, 0.5);
    parser::Vec3f c;
    c.x = a.x / length;
    c.y = a.y / length;
    c.z = a.z / length;
    return c;
}

float intersect_triangle(parser::Vec3i vertex_ids, Ray view_ray, std::vector<parser::Vec3f> vertices)
{
    parser::Vec3f E1 = subtract(vertices[vertex_ids.y - 1], vertices[vertex_ids.x - 1]);
    parser::Vec3f E2 = subtract(vertices[vertex_ids.z - 1], vertices[vertex_ids.x - 1]);
    parser::Vec3f P = cross_product(view_ray.direction, E2);
    float det = dot(E1, P);
    float inv_det = 1 / det;
    parser::Vec3f T = subtract(view_ray.origin, vertices[vertex_ids.x - 1]);
    float u = dot(T, P) * inv_det;
    if (u < 0 || u > 1)
    {
        return -1;
    }
    parser::Vec3f Q = cross_product(T, E1);
    float v = dot(view_ray.direction, Q) * inv_det;
    if (v < 0 || u + v > 1)
    {
        return -1;
    }
    float intersection_t = dot(E2, Q) * inv_det;
    if (intersection_t < 0)
    {
        return -1;
    }
    return intersection_t;
}

float intersect_sphere(parser::Vec3f center, float radius, Ray view_ray)
{
    float d_sq = dot(view_ray.direction, view_ray.direction);
    parser::Vec3f o_minus_c = subtract(view_ray.origin, center);
    float minus_d_times_oc = -dot(view_ray.direction, o_minus_c);
    float discriminant = pow(-minus_d_times_oc, 2) - d_sq * (dot(o_minus_c, o_minus_c) - pow(radius, 2));
    float intersect_t = (minus_d_times_oc - pow(discriminant, 0.5)) / d_sq;
    return intersect_t;
}

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);
    std::vector<parser::Vec3f> vertices = scene.vertex_data;
    std::vector<parser::Mesh> meshes = scene.meshes;
    std::vector<parser::Triangle> triangles = scene.triangles;
    std::vector<parser::Material> materials = scene.materials;
    std::vector<Ray> normals(triangles.size());
    for (int m = 0; m < meshes.size(); m++){

    }
    for (int t = 0; t < triangles.size(); t++)
    {
        parser::Vec3f direction_one = subtract(vertices[scene.triangles[t].indices.v0_id - 1], vertices[scene.triangles[t].indices.v2_id - 1]);
        parser::Vec3f direction_two = subtract(vertices[scene.triangles[t].indices.v1_id - 1], vertices[scene.triangles[t].indices.v2_id - 1]);
        Ray one = {vertices[scene.triangles[t].indices.v2_id - 1], direction_one};
        Ray two = {vertices[scene.triangles[t].indices.v2_id - 1], direction_two};
        normals[t] = Ray{vertices[scene.triangles[t].indices.v2_id - 1], cross_product(one.direction, two.direction)};
    }

    for (int i = 0; i < scene.cameras.size(); i++)
    {
        parser::Camera cam = scene.cameras[i];
        int height = cam.image_height;
        int width = cam.image_width;
        std::string name = cam.image_name;
        /*not sure about these*/
        float l = cam.near_plane.x;
        float r = cam.near_plane.w;
        float b = cam.near_plane.z;
        float t = cam.near_plane.y;
        unsigned char *image = new unsigned char[height * width * 3];

        parser::Vec3f m = add(cam.position, multipy_with_constant(cam.gaze,cam.near_distance));
        parser::Vec3f u = cross_product(cam.gaze, cam.up);
        parser::Vec3f q = add(m, add(multipy_with_constant(u, l), multipy_with_constant(cam.up, t)));

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                float s_u = (x + 0.5) * (r - l) / width;
                float s_v = (y + 0.5) * (t - b) / height;
                int min_t = __INT_MAX__;
                parser::Vec3f s = add(q, add(multipy_with_constant(u, s_u), multipy_with_constant(cam.up, -s_v)));
                Ray view_ray = {cam.position, normalize(subtract(s, cam.position))};
                printf("Percentage finished: %f\n", 100 * (float)(y * width + x) / (height * width));
                for (int t = 0; t < triangles.size(); t++){
                    parser::Vec3i vertex_ids = {triangles[t].indices.v0_id, triangles[t].indices.v1_id, triangles[t].indices.v2_id};
                    float intersection_t = intersect_triangle(vertex_ids, view_ray, vertices);
                    if (intersection_t < min_t && intersection_t > 0)
                    {
                        min_t = intersection_t;
                        /*save object data here*/
                    }
                }
                for (int m = 0; m < meshes.size(); m++){
                    for (int f = 0; f < meshes[m].faces.size(); f++){
                        parser::Vec3i vertex_ids = {meshes[m].faces[f].v0_id, meshes[m].faces[f].v1_id, meshes[m].faces[f].v2_id};
                        float intersection_t = intersect_triangle(vertex_ids, view_ray, vertices);
                        if (intersection_t < min_t && intersection_t > 0)
                        {
                            min_t = intersection_t;
                            /*save object data here*/
                        }
                    }
                }
                for (int s = 0; s < scene.spheres.size(); s++){
                    float intersection_t = intersect_sphere(vertices[scene.spheres[s].center_vertex_id - 1], scene.spheres[s].radius, view_ray);
                    if (intersection_t < min_t && intersection_t > 0)
                    {
                        min_t = intersection_t;
                        /*save object data here*/
                    }
                }
                if (min_t == __INT_MAX__)
                {
                    image[3 * (y * width + x)] = scene.background_color.x;
                    image[3 * (y * width + x) + 1] = scene.background_color.y;
                    image[3 * (y * width + x) + 2] = scene.background_color.z;
                }else{
                    /*calculate color here*/
                    /*temporarily making it red*/
                    image[3 * (y * width + x)] = 255;
                    image[3 * (y * width + x) + 1] = 0;
                    image[3 * (y * width + x) + 2] = 0;
                }
            }
        };

        write_ppm(name.c_str(), image, width, height);
    }



    

}
