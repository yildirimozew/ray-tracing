#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
#include "float.h"
#include <bits/stdc++.h>
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
        bool is_leaf;
        BVHNode* left;
        BVHNode* right;
        std::vector<int> triangles;
        std::vector<int> spheres;
        frame_coordinates coords;
        BVHNode(){
            left = nullptr;
            right = nullptr;
            is_leaf = false;
        }
        // ~BVHNode(){
        //     if (left)
        //         delete left;
        //     if (right)
        //         delete right;
        //     delete this;
        // }
};


struct Ray{
    parser::Vec3f origin;
    parser::Vec3f direction;
};

float find_t(Ray& ray, float point, char axis){
    float origin_p;
    float direction;
    if(axis == 'x') {origin_p = ray.origin.x; direction = ray.direction.x;}
    else if(axis == 'y') {origin_p = ray.origin.y; direction = ray.direction.y;}
    else if(axis == 'z') {origin_p = ray.origin.z; direction = ray.direction.z;}
    float t = (point - origin_p)/direction;
    return t;
}






class BVHTree{
    public:
        BVHNode* root;
        int max_leaf_elem_count;
        enum splitAxis{
            vertical=0, // cut left and right
            horizontal=1, //cut down and up
            curtain=2 // cut front and back

        };
        std::vector<parser::Triangle> tris;
        std::vector<parser::Vec3f> tris_center;
        std::vector<parser::Sphere> spheres;
        std::vector<parser::Vec3f> verts;

        BVHTree(){
            root = nullptr;
            max_leaf_elem_count = 2;
        }
        BVHTree(int elem_count){
            root = nullptr;
            max_leaf_elem_count = elem_count;
        }
        // ~BVHTree(){
        //     delete root;
        // }
        
        struct xcomp{
            std::vector<parser::Vec3f>* tris_center;
            xcomp(std::vector<parser::Vec3f>* pnt){
                this -> tris_center = pnt;
            }
            bool operator() (int tri1, int tri2) {
                return (*tris_center)[tri1].x < (*tris_center)[tri2].x;
            }
        };

        struct ycomp{
            std::vector<parser::Vec3f>* tris_center;
            ycomp(std::vector<parser::Vec3f>* pnt){
                this -> tris_center = pnt;
            }
            bool operator() (int tri1, int tri2) {
                return (*tris_center)[tri1].y < (*tris_center)[tri2].y;
            }
        };
        
        struct zcomp{
            std::vector<parser::Vec3f>* tris_center;
            zcomp(std::vector<parser::Vec3f>* pnt){
                this -> tris_center = pnt;
            }
            bool operator() (int tri1, int tri2) {
                return (*tris_center)[tri1].z < (*tris_center)[tri2].z;
            }
        };

        // bool DoesIntersect(Ray viewRay, frame_coordinates coords){
        //     float t1,t2;
        //     //for x 
        //     if(viewRay.direction.x == 0){
        //         if (viewRay.origin.x > coords.xmax || viewRay.origin.x < coords.xmin)
        //             return false;
        //     }
        //     else{
        //         t1 = find_t(viewRay, coords.xmin, 'x');
        //         t2 = find_t(viewRay, coords.xmax, 'x');
        //         if(t1 < 0&&t2 < 0)
        //             return false;
        //     }
        //     //for y
        //     if(viewRay.direction.y == 0){
        //         if (viewRay.origin.y > coords.ymax || viewRay.origin.y < coords.ymin)
        //             return false;
        //     }
        //     else{
        //         t1 = find_t(viewRay, coords.xmin, 'y');
        //         t2 = find_t(viewRay, coords.xmax, 'y');
        //         if(t1 < 0&&t2 < 0)
        //             return false;
        //     }
        //     //for z
        //     if(viewRay.direction.z == 0){
        //         if (viewRay.origin.z > coords.zmax || viewRay.origin.z < coords.zmin)
        //             return false;
        //     }
        //     else{
        //         t1 = find_t(viewRay, coords.zmin, 'z');
        //         t2 = find_t(viewRay, coords.zmax, 'z');
        //         if(t1 < 0&&t2 < 0)
        //             return false;
        //     }
        //     return true;
        // }

        bool DoesIntersect(Ray ray, frame_coordinates coords){
            float tx1 = (coords.xmin - ray.origin.x) / ray.direction.x, tx2 = (coords.xmax - ray.origin.x) / ray.direction.x;
            float tmin = std::min( tx1, tx2 ), tmax = std::max( tx1, tx2 );
            float ty1 = (coords.ymin - ray.origin.y) / ray.direction.y, ty2 = (coords.ymax - ray.origin.y) / ray.direction.y;
            tmin = std::max( tmin, std::min( ty1, ty2 ) ), tmax = std::min( tmax, std::max( ty1, ty2 ) );
            float tz1 = (coords.zmin - ray.origin.z) / ray.direction.z, tz2 = (coords.zmax - ray.origin.z) / ray.direction.z;
            tmin = std::max( tmin, std::min( tz1, tz2 ) ), tmax = std::min( tmax, std::max( tz1, tz2 ) );
            return tmax >= tmin && tmax > 0;
        }



        int find_depth(BVHNode* current_node){
            if(current_node->is_leaf)
                return 0;
            int right = find_depth(current_node -> right);
            int left = find_depth(current_node -> left);
            return (left>right?left:right) + 1;
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
            float x_total,y_total,z_total;
            bool is_tri_exist = false;
            if(tri_ids.size()>0){
                is_tri_exist = true;
                parser::Vec3f ver0,ver1,ver2;
                ver0 = vertices[triangles[tri_ids[0]].indices.v0_id-1];
                ver1 = vertices[triangles[tri_ids[0]].indices.v1_id-1];
                ver2 = vertices[triangles[tri_ids[0]].indices.v2_id-1];
                edge_vals.xmin = std::min(std::min(ver0.x,ver1.x),ver2.x);
                edge_vals.xmax = std::max(std::max(ver0.x,ver1.x),ver2.x);
                edge_vals.ymin = std::min(std::min(ver0.y,ver1.y),ver2.y);
                edge_vals.ymax = std::max(std::max(ver0.y,ver1.y),ver2.y);
                edge_vals.zmin = std::min(std::min(ver0.z,ver1.z),ver2.z);
                edge_vals.zmax = std::max(std::max(ver0.z,ver1.z),ver2.z);
                
            }   
            for(int tri_order=1, size = tri_ids.size();tri_order<size; tri_order++){
                //calculate min max values
                parser::Vec3f ver0,ver1,ver2;
                ver0 = vertices[triangles[tri_ids[tri_order]].indices.v0_id-1];
                ver1 = vertices[triangles[tri_ids[tri_order]].indices.v1_id-1];
                ver2 = vertices[triangles[tri_ids[tri_order]].indices.v2_id-1];
                edge_vals.xmin = std::min(edge_vals.xmin, std::min(std::min(ver0.x,ver1.x),ver2.x));
                edge_vals.xmax = std::max(edge_vals.xmax, std::max(std::max(ver0.x,ver1.x),ver2.x));
                edge_vals.ymin = std::min(edge_vals.ymin, std::min(std::min(ver0.y,ver1.y),ver2.y));
                edge_vals.ymax = std::max(edge_vals.ymax, std::max(std::max(ver0.y,ver1.y),ver2.y));
                edge_vals.zmin = std::min(edge_vals.zmin, std::min(std::min(ver0.z,ver1.z),ver2.z));
                edge_vals.zmax = std::max(edge_vals.zmax, std::max(std::max(ver0.z,ver1.z),ver2.z));



            }
            
            for(int sphere_order=0, size = sphere_ids.size();sphere_order<size; sphere_order++){
                //calculate min max values
                parser::Vec3f center = vertices[spheres[sphere_order].center_vertex_id-1];
                float radius = spheres[sphere_order].radius;
                edge_vals.xmin = std::min(center.x-radius, edge_vals.xmin);
                edge_vals.xmax = std::max(center.x+radius, edge_vals.xmax);
                edge_vals.ymin = std::min(center.y-radius, edge_vals.ymin);
                edge_vals.ymax = std::max(center.y+radius, edge_vals.ymax);
                edge_vals.zmin = std::min(center.z-radius, edge_vals.zmin);
                edge_vals.zmax = std::max(center.z+radius, edge_vals.zmax);

                x_total += center.x;
                y_total += center.y;
                z_total += center.z;               



            }
            
            // for (int i = 0, size = tri_centers.size(); i<size;i++){
            //     x_total += tri_centers[i].x;
            //     y_total += tri_centers[i].y;
            //     z_total += tri_centers[i].z;
            // }


            float x_avg,y_avg,z_avg;
            int size =  sphere_ids.size();
            x_avg = x_total / size;
            y_avg = y_total / size;
            z_avg = z_total / size;
            
            if(tri_ids.size() + sphere_ids.size() <= max_leaf_elem_count){ //If element count is lower, then make it leaf node
                BVHNode* current_node = new BVHNode;
                current_node -> triangles = tri_ids;
                current_node -> spheres = sphere_ids;
                current_node -> coords = edge_vals;
                current_node -> is_leaf = true;
                return current_node;
            }



            std::vector<int> left_tri_ids;
            std::vector<int> left_sphere_ids;
            std::vector<int> right_tri_ids;
            std::vector<int> right_sphere_ids;

            
            
            if(axis==vertical){
                // split objects into vectors
                
                // for(int tri_order=0, size = tri_ids.size();tri_order<size; tri_order++){
                //     if (tri_centers[tri_ids[tri_order]].x <= x_avg)
                //         left_tri_ids.push_back(tri_ids[tri_order]);
                //     else
                //         right_tri_ids.push_back(tri_ids[tri_order]);
                // }
                std::sort(tri_ids.begin(),tri_ids.end(),xcomp(&tri_centers));
                int i=0;
                for(int hsize = tri_ids.size()/2; i< hsize;i++)
                    left_tri_ids.push_back(tri_ids[i]);
                for(int size = tri_ids.size(); i< size; i++)
                    right_tri_ids.push_back(tri_ids[i]);

                
                for(int sphere_order=0, size = sphere_ids.size();sphere_order<size; sphere_order++){
                    if (vertices[spheres[sphere_order].center_vertex_id].x <= x_avg)
                        left_sphere_ids.push_back(sphere_ids[sphere_order]);
                    else
                        right_sphere_ids.push_back(sphere_ids[sphere_order]);
                }
            }
            else if(axis==horizontal){
                //split objects into vectors
                
                // for(int tri_order=0, size = tri_ids.size();tri_order<size; tri_order++){
                //     if (tri_centers[tri_ids[tri_order]].y <= y_avg)
                //         left_tri_ids.push_back(tri_ids[tri_order]);
                //     else
                //         right_tri_ids.push_back(tri_ids[tri_order]);
                // }

                std::sort(tri_ids.begin(),tri_ids.end(),ycomp(&tri_centers));
                int i=0;
                for(int hsize = tri_ids.size()/2; i< hsize;i++)
                    left_tri_ids.push_back(tri_ids[i]);
                for(int size = tri_ids.size(); i< size; i++)
                    right_tri_ids.push_back(tri_ids[i]);


                for(int sphere_order=0, size = sphere_ids.size();sphere_order<size; sphere_order++){
                    if (vertices[spheres[sphere_order].center_vertex_id].y <= y_avg)
                        left_sphere_ids.push_back(sphere_ids[sphere_order]);
                    else
                        right_sphere_ids.push_back(sphere_ids[sphere_order]);
                }
            }
            else{
                //split objects into vectors
                
                // for(int tri_order=0, size = tri_ids.size();tri_order<size; tri_order++){
                //     if (tri_centers[tri_ids[tri_order]].z <= z_avg)
                //         left_tri_ids.push_back(tri_ids[tri_order]);
                //     else
                //         right_tri_ids.push_back(tri_ids[tri_order]);
                // }

                std::sort(tri_ids.begin(),tri_ids.end(),zcomp(&tri_centers));
                int i=0;
                for(int hsize = tri_ids.size()/2; i< hsize;i++)
                    left_tri_ids.push_back(tri_ids[i]);
                for(int size = tri_ids.size(); i< size; i++)
                    right_tri_ids.push_back(tri_ids[i]);

                for(int sphere_order=0, size = sphere_ids.size();sphere_order<size; sphere_order++){
                    if (vertices[spheres[sphere_order].center_vertex_id].z <= z_avg)
                        left_sphere_ids.push_back(sphere_ids[sphere_order]);
                    else
                        right_sphere_ids.push_back(sphere_ids[sphere_order]);
                }
            }
            splitAxis nextSplitAxis;
            switch (axis) {
                case horizontal:
                    nextSplitAxis=curtain;
                    break;
                case curtain:
                    nextSplitAxis=vertical;
                    break;
                default:
                    nextSplitAxis = horizontal;
                    break;
            }
            BVHNode* current_node = new BVHNode;
            current_node -> left = construct_helpers(
                vertices,
                triangles,
                spheres,
                tri_centers,
                left_tri_ids,
                left_sphere_ids,
                nextSplitAxis
            );
            current_node -> right = construct_helpers(
                vertices,
                triangles,
                spheres,
                tri_centers,
                right_tri_ids,
                right_sphere_ids,
                nextSplitAxis
            );
            current_node -> coords = edge_vals;
            return current_node;
        }




        void construct(
            std::vector<parser::Vec3f>& vertices,
            std::vector<parser::Triangle>& triangles,
            std::vector<parser::Sphere>& spheres,
            std::vector<parser::Vec3f>& tri_centers,
            std::vector<int>& tri_ids,
            std::vector<int>& sphere_ids
        ){  
            tris = triangles;
            this->spheres = spheres;
            verts = vertices;
            tris_center = tri_centers;
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


        void get_objects(Ray& viewRay, std::vector<int>& triangles_ids, std::vector<int>& spheres_ids, BVHNode* current_node, int& counter){
            bool does_intersect = DoesIntersect(viewRay,current_node->coords);
            counter++;
            if(current_node->is_leaf){
                if(does_intersect){
                    for(int i=0,size=current_node->triangles.size();i<size;++i)
                        triangles_ids.push_back(current_node->triangles[i]);
                    for(int i=0,size=current_node->spheres.size();i<size;++i)
                        spheres_ids.push_back(current_node->spheres[i]);
                }
            }
            else{
                if(does_intersect){
                    get_objects(viewRay, triangles_ids, spheres_ids, current_node->left, counter);
                    get_objects(viewRay, triangles_ids, spheres_ids, current_node->right, counter);
                }
            }
        }



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

float intersect_triangle(parser::Vec3i& vertex_ids, Ray& view_ray, std::vector<parser::Vec3f>& vertices)
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
    std::vector<Ray> normals;


    // create a triangles center vector
    std::vector<parser::Vec3f> triangle_centers;
    for(int i=0,size=triangles.size();i<size;i++){
        parser::Face& this_face = triangles[i].indices;
        parser::Vec3f center;
        parser::Vec3f& v0 = vertices[this_face.v0_id-1];
        parser::Vec3f& v1 = vertices[this_face.v1_id-1];
        parser::Vec3f& v2 = vertices[this_face.v2_id-1];

        center.x = (v0.x + v1.x + v2.x) / 3;
        center.y = (v0.y + v1.y + v2.y) / 3;
        center.z = (v0.z + v1.z + v2.z) / 3;

        triangle_centers.push_back(center);
    }

    //Put meshes into triangles
    for(int mesh_id = 0, mesh_vector_size = scene.meshes.size();mesh_id<mesh_vector_size;mesh_id++){
        parser::Mesh& this_mesh = scene.meshes[mesh_id];
        int material_id = this_mesh.material_id;
        for(int face_order=0, mesh_size=this_mesh.faces.size();face_order<mesh_size;face_order++){
            parser::Triangle triangle_to_add = parser::Triangle();
            triangle_to_add.material_id=material_id;
            triangle_to_add.indices=this_mesh.faces[face_order];
            triangles.push_back(triangle_to_add);
            parser::Face& this_face = triangle_to_add.indices;
            parser::Vec3f center;
            parser::Vec3f& v0 = vertices[this_face.v0_id-1];
            parser::Vec3f& v1 = vertices[this_face.v1_id-1];
            parser::Vec3f& v2 = vertices[this_face.v2_id-1];

            center.x = (v0.x + v1.x + v2.x) / 3;
            center.y = (v0.y + v1.y + v2.y) / 3;
            center.z = (v0.z + v1.z + v2.z) / 3;

            triangle_centers.push_back(center);
        }
    }
    for (int t = 0; t < triangles.size(); t++)
    {
        parser::Vec3f direction_one = subtract(vertices[triangles[t].indices.v0_id - 1], vertices[triangles[t].indices.v2_id - 1]);
        parser::Vec3f direction_two = subtract(vertices[triangles[t].indices.v1_id - 1], vertices[triangles[t].indices.v2_id - 1]);
        Ray one = {vertices[triangles[t].indices.v2_id - 1], direction_one};
        Ray two = {vertices[triangles[t].indices.v2_id - 1], direction_two};
        normals.push_back({vertices[triangles[t].indices.v2_id - 1], cross_product(one.direction, two.direction)});
    }

    
    BVHTree bvh_tree;
    int size = triangles.size();
    std::vector<int> tri_ids_1_1(size,0),sphere_ids_1_1(scene.spheres.size(),0);
    for(int i=0, triangle_num=triangle_centers.size();i<triangle_num;++i){
                tri_ids_1_1[i]=i;
            }
    for(int i=0, sphere_num=sphere_ids_1_1.size();i<sphere_num;++i){
        sphere_ids_1_1[i]=i;;
    }
    bvh_tree.construct(vertices,triangles,scene.spheres,triangle_centers,tri_ids_1_1,sphere_ids_1_1);

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
            printf("Percentage finished: %f\n", 100 * (float)(y) / (height));
            for (int x = 0; x < width; ++x)
            {
                float s_u = (x + 0.5) * (r - l) / width;
                float s_v = (y + 0.5) * (t - b) / height;
                int min_t = __INT_MAX__;
                parser::Vec3f s = add(q, add(multipy_with_constant(u, s_u), multipy_with_constant(cam.up, -s_v)));
                Ray view_ray = {cam.position, normalize(subtract(s, cam.position))};
                


                // Find the triangles that we will work on
                std::vector<int> triangle_ids,sphere_ids;
                int counter = 0;
                bvh_tree.get_objects(view_ray,triangle_ids,sphere_ids,bvh_tree.root, counter);



                for (int t = 0; t < triangle_ids.size(); t++){
                    parser::Triangle& triangle=triangles[triangle_ids[t]];
                    parser::Vec3i vertex_ids = {triangle.indices.v0_id, triangle.indices.v1_id, triangle.indices.v2_id};
                    float intersection_t = intersect_triangle(vertex_ids, view_ray, vertices);
                    if (intersection_t < min_t && intersection_t > 0)
                    {
                        min_t = intersection_t;
                        /*save object data here*/
                    }
                }
                for (int s = 0; s < sphere_ids.size(); s++){
                    parser::Sphere sphere = scene.spheres[sphere_ids[s]];
                    float intersection_t = intersect_sphere(vertices[sphere.center_vertex_id - 1], sphere.radius, view_ray);
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
