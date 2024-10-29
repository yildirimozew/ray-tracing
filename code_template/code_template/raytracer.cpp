#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
#include "float.h"
/*may need to add pthread*/

typedef unsigned char RGB[3];

struct frame_coordinates{
    float xmin=FLT_MAX;
    float xmax=-FLT_MAX;
    float ymin=FLT_MAX;
    float ymax=-FLT_MAX;
    float zmin=FLT_MAX;
    float zmax=-FLT_MAX;
};



class BVHNode{
    public:
        BVHNode* left;
        BVHNode* right;
        std::vector<int> triangles;
        std::vector<int> spheres;
        frame_coordinates coords;
        BVHNode(){
            left = nullptr;
            right = nullptr;
        }
        ~BVHNode(){
            if (left)
                delete left;
            if (right)
                delete right;
            delete this;
        }
};






class BVHTree{
    public:
        BVHNode* root;
        int max_leaf_elem_count;
        enum splitAxis{
            vertical=0, // cut left and right
            horizontal=1, //cut down and up
            curtain=2 // cut front and back

        };
        

        BVHTree(){
            root = nullptr;
            max_leaf_elem_count = 1;
        }
        BVHTree(int elem_count){
            root = nullptr;
            max_leaf_elem_count = elem_count;
        }
        ~BVHTree(){
            delete root;
        }
        


        BVHNode* construct_helpers(
            std::vector<parser::Vec3f>& vertices,
            std::vector<parser::Triangle>& triangles,
            std::vector<parser::Sphere>& spheres,
            std::vector<parser::Vec3f>& tri_centers,
            std::vector<int>& tri_ids,
            std::vector<int>& sphere_ids,
            splitAxis axis
        ){

            frame_coordinates edge_vals;
            for(int tri_order=0, size = tri_ids.size();tri_order<size; tri_order++){
                //calculate min max values
                edge_vals.xmin = edge_vals.xmin>vertices[triangles[tri_order].indices.v0_id].x?vertices[triangles[tri_order].indices.v0_id].x:edge_vals.xmin;
                edge_vals.xmin = edge_vals.xmin>vertices[triangles[tri_order].indices.v1_id].x?vertices[triangles[tri_order].indices.v1_id].x:edge_vals.xmin;
                edge_vals.xmin = edge_vals.xmin>vertices[triangles[tri_order].indices.v2_id].x?vertices[triangles[tri_order].indices.v2_id].x:edge_vals.xmin;
                edge_vals.xmax = edge_vals.xmax<vertices[triangles[tri_order].indices.v0_id].x?vertices[triangles[tri_order].indices.v0_id].x:edge_vals.xmax;
                edge_vals.xmax = edge_vals.xmax<vertices[triangles[tri_order].indices.v1_id].x?vertices[triangles[tri_order].indices.v1_id].x:edge_vals.xmax;
                edge_vals.xmax = edge_vals.xmax<vertices[triangles[tri_order].indices.v2_id].x?vertices[triangles[tri_order].indices.v2_id].x:edge_vals.xmax;
                edge_vals.ymin = edge_vals.ymin>vertices[triangles[tri_order].indices.v0_id].y?vertices[triangles[tri_order].indices.v0_id].y:edge_vals.ymin;
                edge_vals.ymin = edge_vals.ymin>vertices[triangles[tri_order].indices.v1_id].y?vertices[triangles[tri_order].indices.v1_id].y:edge_vals.ymin;
                edge_vals.ymin = edge_vals.ymin>vertices[triangles[tri_order].indices.v2_id].y?vertices[triangles[tri_order].indices.v2_id].y:edge_vals.ymin;
                edge_vals.ymax = edge_vals.ymax<vertices[triangles[tri_order].indices.v0_id].y?vertices[triangles[tri_order].indices.v0_id].y:edge_vals.ymax;
                edge_vals.ymax = edge_vals.ymax<vertices[triangles[tri_order].indices.v1_id].y?vertices[triangles[tri_order].indices.v1_id].y:edge_vals.ymax;
                edge_vals.ymax = edge_vals.ymax<vertices[triangles[tri_order].indices.v2_id].y?vertices[triangles[tri_order].indices.v2_id].y:edge_vals.ymax;
                edge_vals.zmin = edge_vals.zmin>vertices[triangles[tri_order].indices.v0_id].z?vertices[triangles[tri_order].indices.v0_id].z:edge_vals.zmin;
                edge_vals.zmin = edge_vals.zmin>vertices[triangles[tri_order].indices.v1_id].z?vertices[triangles[tri_order].indices.v1_id].z:edge_vals.zmin;
                edge_vals.zmin = edge_vals.zmin>vertices[triangles[tri_order].indices.v2_id].z?vertices[triangles[tri_order].indices.v2_id].z:edge_vals.zmin;
                edge_vals.zmax = edge_vals.zmax<vertices[triangles[tri_order].indices.v0_id].z?vertices[triangles[tri_order].indices.v0_id].z:edge_vals.zmax;
                edge_vals.zmax = edge_vals.zmax<vertices[triangles[tri_order].indices.v1_id].z?vertices[triangles[tri_order].indices.v1_id].z:edge_vals.zmax;
                edge_vals.zmax = edge_vals.zmax<vertices[triangles[tri_order].indices.v2_id].z?vertices[triangles[tri_order].indices.v2_id].z:edge_vals.zmax;
                
            }
            for(int sphere_order=0, size = sphere_ids.size();sphere_order<size; sphere_order++){
                //calculate min max values
                parser::Vec3f center = vertices[spheres[sphere_order].center_vertex_id];
                float radius = spheres[sphere_order].radius;
                edge_vals.xmin = edge_vals.xmin > center.x - radius ? center.x - radius : edge_vals.xmin;
                edge_vals.xmax = edge_vals.xmax < center.x + radius ? center.x + radius : edge_vals.xmax;
                edge_vals.ymin = edge_vals.ymin > center.y - radius ? center.y - radius : edge_vals.ymin;
                edge_vals.ymax = edge_vals.ymax < center.y + radius ? center.y + radius : edge_vals.ymax;
                edge_vals.zmin = edge_vals.zmin > center.z - radius ? center.z - radius : edge_vals.zmin;
                edge_vals.zmax = edge_vals.zmax < center.z + radius ? center.z + radius : edge_vals.zmax;
            }
            
            
            if(tri_ids.size() + sphere_ids.size() <= max_leaf_elem_count){ //If element count is lower, then make it leaf node
                BVHNode* current_node = new BVHNode;
                current_node -> triangles = tri_ids;
                current_node -> spheres = sphere_ids;
                current_node -> coords = edge_vals;
                return current_node;
            }



            std::vector<int> left_tri_ids;
            std::vector<int> left_sphere_ids;
            std::vector<int> right_tri_ids;
            std::vector<int> right_sphere_ids;

            
            
            if(axis==vertical){
                // split objects into vectors
                float x_avg = (edge_vals.xmax + edge_vals.xmin)/2.;
                for(int tri_order=0, size = tri_ids.size();tri_order<size; tri_order++){
                    if (tri_centers[tri_ids[tri_order]].x < x_avg)
                        left_tri_ids.push_back(tri_ids[tri_order]);
                    else
                        right_tri_ids.push_back(tri_ids[tri_order]);
                }
                for(int sphere_order=0, size = sphere_ids.size();sphere_order<size; sphere_order++){
                    if (vertices[spheres[sphere_order].center_vertex_id].x < x_avg)
                        left_sphere_ids.push_back(sphere_ids[sphere_order]);
                    else
                        right_sphere_ids.push_back(sphere_ids[sphere_order]);
                }
            }
            else if(axis==horizontal){
                //split objects into vectors
                float y_avg = (edge_vals.ymax + edge_vals.ymin)/2.;
                for(int tri_order=0, size = tri_ids.size();tri_order<size; tri_order++){
                    if (tri_centers[tri_ids[tri_order]].y < y_avg)
                        left_tri_ids.push_back(tri_ids[tri_order]);
                    else
                        right_tri_ids.push_back(tri_ids[tri_order]);
                }
                for(int sphere_order=0, size = sphere_ids.size();sphere_order<size; sphere_order++){
                    if (vertices[spheres[sphere_order].center_vertex_id].y < y_avg)
                        left_sphere_ids.push_back(sphere_ids[sphere_order]);
                    else
                        right_sphere_ids.push_back(sphere_ids[sphere_order]);
                }
            }
            else{
                //split objects into vectors
                float z_avg = (edge_vals.zmax + edge_vals.zmin)/2.;
                for(int tri_order=0, size = tri_ids.size();tri_order<size; tri_order++){
                    if (tri_centers[tri_ids[tri_order]].z < z_avg)
                        left_tri_ids.push_back(tri_ids[tri_order]);
                    else
                        right_tri_ids.push_back(tri_ids[tri_order]);
                }
                for(int sphere_order=0, size = sphere_ids.size();sphere_order<size; sphere_order++){
                    if (vertices[spheres[sphere_order].center_vertex_id].z < z_avg)
                        left_sphere_ids.push_back(sphere_ids[sphere_order]);
                    else
                        right_sphere_ids.push_back(sphere_ids[sphere_order]);
                }
            }
            BVHNode* current_node = new BVHNode;
            current_node -> left = construct_helpers(
                vertices,
                triangles,
                spheres,
                tri_centers,
                left_tri_ids,
                left_sphere_ids,
                static_cast<splitAxis>(axis+1%3)
            );
            current_node -> right = construct_helpers(
                vertices,
                triangles,
                spheres,
                tri_centers,
                right_tri_ids,
                right_sphere_ids,
                static_cast<splitAxis>(axis+1%3)
            );
            current_node -> coords = edge_vals;
            return current_node;
        }




        void construct(
            std::vector<parser::Vec3f>& vertices,
            std::vector<parser::Triangle>& triangles,
            std::vector<parser::Sphere>& spheres,
            std::vector<parser::Vec3f>& tri_centers
        ){  
            std::vector<int> tri_ids;
            std::vector<int> sphere_ids;
            for(int i=0, triangle_num=tri_centers.size();i<triangle_num;++i){
                tri_ids.push_back(i);
            }
            for(int i=0, sphere_num=spheres.size();i<sphere_num;++i){
                sphere_ids.push_back(i);
            }

            root = construct_helpers(
                vertices,
                triangles,
                spheres,
                tri_centers,
                tri_ids,
                sphere_ids,
                vertical
            );
        }
};



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
