#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
#include <chrono>
/*may need to add pthread*/

typedef unsigned char RGB[3];

struct Ray
{
    parser::Vec3f origin;
    parser::Vec3f direction;
};

parser::Vec3f cross_product(parser::Vec3f &a, parser::Vec3f &b)
{
    parser::Vec3f c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

parser::Vec3f multipy(parser::Vec3f &a, parser::Vec3f &b)
{
    parser::Vec3f c;
    c.x = a.x * b.x;
    c.y = a.y * b.y;
    c.z = a.z * b.z;
    return c;
}

parser::Vec3f multipy_with_constant(parser::Vec3f &a, float &b)
{
    parser::Vec3f c;
    c.x = a.x * b;
    c.y = a.y * b;
    c.z = a.z * b;
    return c;
}

parser::Vec3f subtract(parser::Vec3f &a, parser::Vec3f &b)
{
    parser::Vec3f c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
}

parser::Vec3f add(parser::Vec3f &a, parser::Vec3f &b)
{
    parser::Vec3f c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

parser::Vec3f divide(parser::Vec3f &a, float &b)
{
    parser::Vec3f c;
    c.x = a.x / b;
    c.y = a.y / b;
    c.z = a.z / b;
    return c;
}

float dot(parser::Vec3f &a, parser::Vec3f &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

parser::Vec3f normalize(parser::Vec3f &a)
{
    float length = pow(a.x * a.x + a.y * a.y + a.z * a.z, 0.5);
    parser::Vec3f c;
    c.x = a.x / length;
    c.y = a.y / length;
    c.z = a.z / length;
    return c;
}

float intersect_triangle(parser::Face &vertex_ids, Ray &view_ray, std::vector<parser::Vec3f> &vertices)
{
    parser::Vec3f E1 = subtract(vertices[vertex_ids.v1_id - 1], vertices[vertex_ids.v0_id - 1]);
    parser::Vec3f E2 = subtract(vertices[vertex_ids.v2_id - 1], vertices[vertex_ids.v0_id - 1]);
    parser::Vec3f P = cross_product(view_ray.direction, E2);
    float det = dot(E1, P);
    float inv_det = 1 / det;
    parser::Vec3f T = subtract(view_ray.origin, vertices[vertex_ids.v0_id - 1]);
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

float intersect_sphere(parser::Vec3f &center, float &radius, Ray &view_ray)
{
    float d_sq = dot(view_ray.direction, view_ray.direction);
    parser::Vec3f o_minus_c = subtract(view_ray.origin, center);
    float minus_d_times_oc = -dot(view_ray.direction, o_minus_c);
    float discriminant = pow(-minus_d_times_oc, 2) - d_sq * (dot(o_minus_c, o_minus_c) - pow(radius, 2));
    float intersect_t = (minus_d_times_oc - pow(discriminant, 0.5)) / d_sq;
    return intersect_t;
}

parser::Vec3f diffuse_shading(std::vector<parser::PointLight> &point_lights, Ray &normal, parser::Vec3f &point, parser::Material &material)
{
    parser::Vec3f summed_result = {0, 0, 0};
    for (int l = 0; l < point_lights.size(); l++)
    {
        /*normal bulma algoritmasını buraya taşımayı dene*/
        parser::Vec3f light_position = point_lights[l].position;
        parser::Vec3f distance_vector = subtract(light_position, point);
        parser::Vec3f normalized_distance_vector = normalize(distance_vector);
        float distance_squared = pow(distance_vector.x, 2) + pow(distance_vector.y, 2) + pow(distance_vector.z, 2);
        parser::Vec3f received_irradiance = divide(point_lights[l].intensity, distance_squared);
        float cos_theta = dot(normal.direction, normalized_distance_vector);
        cos_theta = cos_theta > 0 ? cos_theta : 0;
        parser::Vec3f result_1 = multipy(material.diffuse, received_irradiance);
        parser::Vec3f result = multipy_with_constant(result_1, cos_theta);
        summed_result = add(result, summed_result);
    }
    return summed_result;
}

parser::Vec3f ambient_shading(parser::Vec3f ambient_coefficient, parser::Material &material)
{
    parser::Vec3f ambient = multipy(ambient_coefficient, material.ambient);
    return ambient;
}

int main(int argc, char *argv[])
{
    auto start = std::chrono::high_resolution_clock::now();
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);
    std::vector<parser::Vec3f> vertices = scene.vertex_data;
    std::vector<parser::Mesh> meshes = scene.meshes;
    std::vector<parser::Triangle> triangles = scene.triangles;
    std::vector<parser::Material> materials = scene.materials;
    for (int m = 0; m < meshes.size(); m++)
    {
        for (int f = 0; f < meshes[m].faces.size(); f++)
        {
            parser::Triangle triangle;
            triangle.indices = meshes[m].faces[f];
            triangle.material_id = meshes[m].material_id;
            triangles.push_back(triangle);
        }
    }
    std::vector<Ray> normals(triangles.size());
    for (int t = 0; t < triangles.size(); t++)
    {
        parser::Vec3f direction_one = subtract(vertices[triangles[t].indices.v0_id - 1], vertices[triangles[t].indices.v2_id - 1]);
        parser::Vec3f direction_two = subtract(vertices[triangles[t].indices.v1_id - 1], vertices[triangles[t].indices.v2_id - 1]);
        parser::Vec3f normal = cross_product(direction_one, direction_two);
        parser::Vec3f normalized_normal = normalize(normal);
        normals[t] = Ray{vertices[triangles[t].indices.v2_id - 1], normalized_normal};
    }
    for (int i = 0; i < scene.cameras.size(); i++)
    {
        parser::Camera cam = scene.cameras[i];
        int height = cam.image_height;
        int width = cam.image_width;
        std::string name = cam.image_name;
        float l = cam.near_plane.x;
        float r = cam.near_plane.w;
        float b = cam.near_plane.z;
        float t = cam.near_plane.y;
        unsigned char *image = new unsigned char[height * width * 3];
        int print_counter = 0;
        parser::Vec3f m_2 = multipy_with_constant(cam.gaze, cam.near_distance);
        parser::Vec3f m = add(cam.position, m_2);
        parser::Vec3f u = cross_product(cam.gaze, cam.up);
        parser::Vec3f q_2_1 = multipy_with_constant(u, l);
        parser::Vec3f q_2_2 = multipy_with_constant(cam.up, t);
        parser::Vec3f q_2 = add(q_2_1, q_2_2);
        parser::Vec3f q = add(m, q_2);
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                Ray normal;
                parser::Material material;
                parser::Vec3f hit_point = {0, 0, 0};
                float s_u = (x + 0.5) * (r - l) / width;
                float minus_s_v = -(y + 0.5) * (t - b) / height;
                float min_t = __INT_MAX__;
                parser::Vec3f s_1_1 = multipy_with_constant(u, s_u);
                parser::Vec3f s_1_2 = multipy_with_constant(cam.up, minus_s_v);
                parser::Vec3f s_1 = add(s_1_1, s_1_2);
                parser::Vec3f s = add(q, s_1);
                parser::Vec3f vr_1 = subtract(s, cam.position);
                Ray view_ray = {cam.position, normalize(vr_1)};
                if (print_counter++ == 500)
                {
                    float percentage = (float)(y * width + x) / (height * width) * 100;
                    printf("Percentage finished: %.2f\n", percentage);
                    print_counter = 0;
                }
                for (int t = 0; t < triangles.size(); t++)
                {
                    float intersection_t = intersect_triangle(triangles[t].indices, view_ray, vertices);
                    if (intersection_t < min_t && intersection_t > 0)
                    {
                        min_t = intersection_t;
                        normal = normals[t];
                        material = scene.materials[triangles[t].material_id - 1];
                        parser::Vec3f hit_point_temp = multipy_with_constant(view_ray.direction, intersection_t);
                        hit_point = add(cam.position, hit_point_temp);
                    }
                }
                for (int s = 0; s < scene.spheres.size(); s++)
                {
                    float intersection_t = intersect_sphere(vertices[scene.spheres[s].center_vertex_id - 1], scene.spheres[s].radius, view_ray);
                    if (intersection_t < min_t && intersection_t > 0)
                    {
                        min_t = intersection_t;
                        /*save object data here*/
                        /*TODO*/
                    }
                }
                if (min_t == __INT_MAX__)
                {
                    image[3 * (y * width + x)] = scene.background_color.x;
                    image[3 * (y * width + x) + 1] = scene.background_color.y;
                    image[3 * (y * width + x) + 2] = scene.background_color.z;
                }
                else
                {
                    if (hit_point.x != 0)
                    {
                        parser::Vec3f ambient_shading_part = ambient_shading(scene.ambient_light, material);
                        parser::Vec3f diffuse_shading_part = diffuse_shading(scene.point_lights, normal, hit_point, material);
                        parser::Vec3f color = add(ambient_shading_part, diffuse_shading_part);
                        float red = color.x > 255 ? 255 : color.x;
                        float green = color.y > 255 ? 255 : color.y;
                        float blue = color.z > 255 ? 255 : color.z;
                        image[3 * (y * width + x)] = (int)red;
                        image[3 * (y * width + x) + 1] = (int)green;
                        image[3 * (y * width + x) + 2] = (int)blue;
                    }
                    else
                    {
                        image[3 * (y * width + x)] = 255;
                        image[3 * (y * width + x) + 1] = 0;
                        image[3 * (y * width + x) + 2] = 0;
                    }
                }
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Time taken: " << duration << " milliseconds" << std::endl;
        write_ppm(name.c_str(), image, width, height);
    }
}

/*monkey takes 67 seconds without shading*/
/*car camera 1 takes 335 seconds without shading*/