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

struct HitRecord{
    parser::Vec3f hit_point;
    parser::Material material;
    Ray normal;
    float min_t;
};


parser::Vec3f cross_product(parser::Vec3f &a, parser::Vec3f &b)
{
    parser::Vec3f c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

parser::Vec3f multiply(parser::Vec3f &a, parser::Vec3f &b)
{
    parser::Vec3f c;
    c.x = a.x * b.x;
    c.y = a.y * b.y;
    c.z = a.z * b.z;
    return c;
}

parser::Vec3f multiply_with_constant(parser::Vec3f &a, float b)
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

parser::Vec3f diffuse_shading(parser::PointLight &point_light, Ray &normal, parser::Vec3f &point, parser::Material &material)
{
        /*normal bulma algoritmasını buraya taşımayı dene*/
        parser::Vec3f light_position = point_light.position;
        parser::Vec3f distance_vector = subtract(light_position, point);
        parser::Vec3f normalized_distance_vector = normalize(distance_vector);
        float distance_squared = pow(distance_vector.x, 2) + pow(distance_vector.y, 2) + pow(distance_vector.z, 2);
        parser::Vec3f received_irradiance = divide(point_light.intensity, distance_squared);
        float cos_theta = dot(normal.direction, normalized_distance_vector);
        cos_theta = cos_theta > 0 ? cos_theta : 0;
        parser::Vec3f result_1 = multiply(material.diffuse, received_irradiance);
        parser::Vec3f result = multiply_with_constant(result_1, cos_theta);
        return result;
}

parser::Vec3f ambient_shading(parser::Vec3f ambient_coefficient, parser::Material &material)
{
    parser::Vec3f ambient = multiply(ambient_coefficient, material.ambient);
    return ambient;
}

parser::Vec3f blinnphong_shading(parser::PointLight &point_light, Ray &normal, parser::Vec3f &point, parser::Material &material, parser::Vec3f &outgoingVec){
    
        parser::Vec3f light_position = point_light.position;
        parser::Vec3f distance_vector = subtract(light_position, point);
        parser::Vec3f normalized_distance_vector = normalize(distance_vector);
        float distance_squared = pow(distance_vector.x, 2) + pow(distance_vector.y, 2) + pow(distance_vector.z, 2);
        parser::Vec3f received_irradiance = divide(point_light.intensity, distance_squared);
        parser::Vec3f halfVec = add(normalized_distance_vector,outgoingVec);
        parser::Vec3f normalizedhalfVec = normalize(halfVec);

        

        float cos_alpha = std::max(float(0),dot(normal.direction, normalizedhalfVec));
        float cos_alpha_exponed = pow(cos_alpha,material.phong_exponent);
        parser::Vec3f result_1 = multiply(material.specular, received_irradiance);
        parser::Vec3f result = multiply_with_constant(result_1, cos_alpha_exponed);
        return result;
}

HitRecord findHitRecord(std::vector<parser::Triangle>& triangles, parser::Scene& scene, Ray& view_ray, std::vector<Ray>& normals){
    std::vector<parser::Vec3f>& vertices = scene.vertex_data;
    float min_t = __FLT_MAX__;
    parser::Vec3f hit_point = {0, 0, 0};
    Ray normal;
    parser::Material material;
    for (int t = 0; t < triangles.size(); t++)
    {
        float intersection_t = intersect_triangle(triangles[t].indices, view_ray, vertices);
        if (intersection_t < min_t && intersection_t > 0)
        {
            min_t = intersection_t;
            normal = normals[t];
            material = scene.materials[triangles[t].material_id - 1];
            parser::Vec3f hit_point_temp = multiply_with_constant(view_ray.direction, intersection_t);
            hit_point = add(view_ray.origin, hit_point_temp);
        }
    }
    for (int s = 0; s < scene.spheres.size(); s++)
    {
        float intersection_t = intersect_sphere(vertices[scene.spheres[s].center_vertex_id - 1], scene.spheres[s].radius, view_ray);
        if (intersection_t < min_t && intersection_t > 0)
        {
            min_t = intersection_t;
            material = scene.materials[scene.spheres[s].material_id-1];
            parser::Vec3f tmp_d = multiply_with_constant(view_ray.direction,intersection_t);
            hit_point = add(view_ray.origin, tmp_d);
            tmp_d = subtract(hit_point,scene.vertex_data[scene.spheres[s].center_vertex_id-1]);
            normal.direction = normalize(tmp_d);
            normal.origin = hit_point;

        }
    }
    HitRecord result;
    result.material = material;
    result.hit_point = hit_point;
    result.normal = normal;
    result.min_t = min_t;
    return result;
}

parser::Vec3f computeColor(std::vector<parser::Triangle>& triangles, parser::Scene& scene, Ray& view_ray, std::vector<Ray>& normals, int current_depth,int max_depth);


parser::Vec3f apply_shading(parser::Scene& scene,HitRecord& hit_record,Ray& view_ray, int current_depth, int max_depth, std::vector<Ray>& normals, std::vector<parser::Triangle>& triangles){
    parser::Material& material = hit_record.material;
    parser::Vec3f& hit_point = hit_record.hit_point;
    Ray& normal = hit_record.normal;
    parser::Vec3f color = ambient_shading(scene.ambient_light, material);
    
    
    if(material.is_mirror){
        Ray reflect_ray;

        //May have to multiply view ray with -1
        float minusone =-1;
        parser::Vec3f wo = multiply_with_constant(view_ray.direction, minusone);

        float tmp = dot(wo, normal.direction);
        tmp *= 2;
        parser::Vec3f tmp2 = multiply_with_constant(normal.direction,tmp);
        parser::Vec3f tmp3 = add(tmp2,view_ray.direction);
        reflect_ray.direction = normalize(tmp3);
        reflect_ray.origin = hit_point;
        reflect_ray.origin.y += 1e-5;
        parser::Vec3f color2 = computeColor(triangles,scene,reflect_ray,normals,current_depth+1,max_depth);
        color2 = multiply(color2,material.mirror);
        color = add(color,color2);
    }

    for(int l= 0; l< scene.point_lights.size();l++){
        parser::Vec3f light_position = scene.point_lights[l].position;
        parser::Vec3f distance_vector = subtract(light_position, hit_point);
        parser::Vec3f normalized_distance_vector = normalize(distance_vector);
        float t = distance_vector.x/normalized_distance_vector.x;
        Ray tmp_ray;
        tmp_ray.direction = normalized_distance_vector;
        parser::Vec3f epsilon = multiply_with_constant(normalized_distance_vector,1e-5);
        tmp_ray.origin = add(hit_point,epsilon);
        HitRecord lightHitRecord = findHitRecord(triangles,scene,tmp_ray,normals);
        if(lightHitRecord.min_t == __FLT_MAX__ || lightHitRecord.min_t >= t)
        {
            parser::Vec3f diffuse_shading_part = diffuse_shading(scene.point_lights[l], normal, hit_point, material);
            parser::Vec3f outgoingVec = view_ray.direction;
            outgoingVec.x = -outgoingVec.x;
            outgoingVec.y = -outgoingVec.y;
            outgoingVec.z = -outgoingVec.z;
            parser::Vec3f BlinnPhong_part = blinnphong_shading(scene.point_lights[l], normal, hit_point, material, outgoingVec);
            parser::Vec3f tmp= add(diffuse_shading_part,BlinnPhong_part);
            color = add(color,tmp);            
        }

    }


    return color;
    
}




parser::Vec3f computeColor(std::vector<parser::Triangle>& triangles, parser::Scene& scene, Ray& view_ray, std::vector<Ray>& normals, int current_depth,int max_depth){
    if(current_depth>= max_depth){
        parser::Vec3f tmp;
        tmp.x = 0,tmp.y=0,tmp.z=0;
        return tmp;
    }
    float min_t = __FLT_MAX__;
    HitRecord hit_record = findHitRecord(triangles, scene, view_ray, normals);
    parser::Vec3f& hit_point = hit_record.hit_point;
    parser::Material& material = hit_record.material;
    Ray& normal = hit_record.normal;
    min_t = hit_record.min_t;
    parser::Vec3f resultingColor;
    
    if (min_t == __FLT_MAX__)
    {
        resultingColor.x =  scene.background_color.x;
        resultingColor.y =  scene.background_color.y;
        resultingColor.z =  scene.background_color.z;
    }
    else
    {
        if (hit_point.x != 0)
        {
            resultingColor = apply_shading(scene,hit_record,view_ray,current_depth,max_depth, normals, triangles);
            resultingColor.x = resultingColor.x > 255 ? 255 : resultingColor.x;
            resultingColor.y = resultingColor.y > 255 ? 255 : resultingColor.y;
            resultingColor.z = resultingColor.z > 255 ? 255 : resultingColor.z;
        }
        else
        {
            resultingColor.x = 255;
            resultingColor.y = 0;
            resultingColor.z = 0;
            
        }
    }
    return resultingColor;
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
        parser::Vec3f m_2 = multiply_with_constant(cam.gaze, cam.near_distance);
        parser::Vec3f m = add(cam.position, m_2);
        parser::Vec3f u = cross_product(cam.gaze, cam.up);
        parser::Vec3f q_2_1 = multiply_with_constant(u, l);
        parser::Vec3f q_2_2 = multiply_with_constant(cam.up, t);
        parser::Vec3f q_2 = add(q_2_1, q_2_2);
        parser::Vec3f q = add(m, q_2);
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                //Compute view ray
                float s_u = (x + 0.5) * (r - l) / width;
                float minus_s_v = -(y + 0.5) * (t - b) / height;
                float min_t = __INT_MAX__;
                parser::Vec3f s_1_1 = multiply_with_constant(u, s_u);
                parser::Vec3f s_1_2 = multiply_with_constant(cam.up, minus_s_v);
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

                //compute color
                parser::Vec3f color = computeColor(triangles, scene, view_ray, normals, 0,scene.max_recursion_depth);
                image[3 * (y * width + x)] = color.x;
                image[3 * (y * width + x) + 1] = color.y;
                image[3 * (y * width + x) + 2] = color.z;
                
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