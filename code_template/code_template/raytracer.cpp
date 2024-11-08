#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
#include <chrono>
#include "bits/stdc++.h"
/*may need to add pthread*/
typedef unsigned char RGB[3];

struct Ray
{
    parser::Vec3f origin;
    parser::Vec3f direction;
};

struct HitRecord
{
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

parser::Vec3f divide(parser::Vec3f &a, float b)
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

bool isParallel(parser::Vec3f &ray1, parser::Vec3f &ray2)
{
    return ray1.x == ray2.x && ray1.y == ray2.y && ray1.z == ray2.z;
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






struct MyTriangle{
    parser::Vec3f center;
    parser::Vec3f min;
    parser::Vec3f max;
    parser::Triangle* tri;
    Ray* normal;
};

bool compx(MyTriangle* tri1,MyTriangle* tri2){
    return (tri1)->center.x < (tri2)->center.x;
}

bool compy(MyTriangle* tri1,MyTriangle* tri2){
    return (tri1)->center.y < (tri2)->center.y;
}

bool compz(MyTriangle* tri1,MyTriangle* tri2){
    return (tri1)->center.z < (tri2)->center.z;
}

void getMin(parser::Vec3f& vec1,parser::Vec3f& result){
    result.x = std::min(vec1.x,result.x);
    result.y = std::min(vec1.y,result.y);
    result.z = std::min(vec1.z,result.z);
}
void getMax(parser::Vec3f& vec1,parser::Vec3f& result){
    result.x = std::max(vec1.x,result.x);
    result.y = std::max(vec1.y,result.y);
    result.z = std::max(vec1.z,result.z);
}


bool DoesHit(Ray& ray, parser::Vec3f& bmin, parser::Vec3f& bmax )
{
    float tx1 = (bmin.x - ray.origin.x) / ray.direction.x, tx2 = (bmax.x - ray.origin.x) / ray.direction.x;
    float tmin = std::min( tx1, tx2 ), tmax = std::max( tx1, tx2 );
    float ty1 = (bmin.y - ray.origin.y) / ray.direction.y, ty2 = (bmax.y - ray.origin.y) / ray.direction.y;
    tmin = std::max( tmin, std::min( ty1, ty2 ) ), tmax = std::min( tmax, std::max( ty1, ty2 ) );
    float tz1 = (bmin.z - ray.origin.z) / ray.direction.z, tz2 = (bmax.z - ray.origin.z) / ray.direction.z;
    tmin = std::max( tmin, std::min( tz1, tz2 ) ), tmax = std::min( tmax, std::max( tz1, tz2 ) );
    return tmax >= tmin && tmax > 0;
}



class BVHNodeTri{
    public:
        std::vector<MyTriangle*> triangles;
        BVHNodeTri* left=nullptr;
        BVHNodeTri* right=nullptr;
        bool isLeaf=false;
        parser::Vec3f min,max;
};

class BVHTreeTri{
    public:
        int max_leaf_count = 2;
        BVHNodeTri* root=nullptr;
        std::vector<MyTriangle*> bvh_triangles;
    BVHTreeTri(int leaf_count,std::vector<MyTriangle>& triangles){
        this -> max_leaf_count = leaf_count;
        for(std::vector<MyTriangle>::iterator start = triangles.begin(),end = triangles.end();start != end;start++){
            bvh_triangles.push_back(&*start);
        }
        std::vector<MyTriangle*> tri = bvh_triangles;
        this -> root = construct_helper(tri,'y');
    }
    BVHNodeTri* construct_helper(std::vector<MyTriangle*>& triangles, char orientation){
        parser::Vec3f min_vec = triangles[0]->min;
        parser::Vec3f max_vec = triangles[0]->max;
        std::vector<MyTriangle*>::iterator start = triangles.begin(),end= triangles.end();
            start++;
        while(start!=end){
            getMin((*start)->min,min_vec);
            getMax((*start)->max,max_vec);
            start++;
        }
        BVHNodeTri* currentNode = new BVHNodeTri();
        currentNode->min = min_vec;
        currentNode->max = max_vec;
        if(triangles.size()<=this->max_leaf_count){
            currentNode->isLeaf = true;
            currentNode-> triangles = triangles;
            return currentNode;
        }
        std::vector<MyTriangle*> left_tris;
        std::vector<MyTriangle*> right_tris;

        if (orientation == 'x'){
            std::sort(triangles.begin(),triangles.end(),compx);
            int size = triangles.size();
            int half_size = size / 2;
            int i = 0;
            start = triangles.begin();
            for(;i < half_size;i++,start++){
                left_tris.push_back((*start));
            }
            for(;i < size;i++,start++){
                right_tris.push_back((*start));
            }
        }
        else if(orientation == 'y') {
            std::sort(triangles.begin(),triangles.end(),compy);
            int size = triangles.size();
            int half_size = size / 2;
            int i = 0;
            start = triangles.begin();
            for(;i < half_size;i++,start++){
                left_tris.push_back((*start));
            }
            for(;i < size;i++,start++){
                right_tris.push_back((*start));
            }
        }
        else{
            std::sort(triangles.begin(),triangles.end(),compz);
            int size = triangles.size();
            int half_size = size / 2;
            int i = 0;
            start = triangles.begin();
            for(;i < half_size;i++,start++){
                left_tris.push_back((*start));
            }
            for(;i < size;i++,start++){
                right_tris.push_back((*start));
            }
        }
        char next_ori;
        switch(orientation){
            case 'x':
                next_ori = 'y';
                break;
            case 'y':
                next_ori = 'z';
                break;
            default:
                next_ori = 'x';
                break;
        }
        currentNode -> left = construct_helper(left_tris,next_ori);
        currentNode -> right = construct_helper(right_tris,next_ori);
        return currentNode;  
    }
    void getTriangles(Ray& ray, std::vector<MyTriangle*>& triangleList,std::vector<Ray*>& normalList){
        getTrianglesHelper(ray,triangleList,normalList,this->root);
    }

    void getTrianglesHelper(Ray& ray, std::vector<MyTriangle*>& triangleList,std::vector<Ray*>& normalList, BVHNodeTri* currentNode){
        bool hit = DoesHit(ray,currentNode->min,currentNode->max);
        if(hit){
            if(currentNode -> isLeaf){
                for(std::vector<MyTriangle*>::iterator start = currentNode -> triangles.begin(), end = currentNode -> triangles.end();start!= end;start++){
                    triangleList.push_back(*start);
                }
            }
            else{
                getTrianglesHelper(ray, triangleList,normalList,currentNode->left);
                getTrianglesHelper(ray, triangleList,normalList,currentNode->right);
            }
        }
    }
};













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

parser::Vec3f blinnphong_shading(parser::PointLight &point_light, Ray &normal, parser::Vec3f &point, parser::Material &material, parser::Vec3f &outgoingVec)
{

    parser::Vec3f light_position = point_light.position;
    parser::Vec3f distance_vector = subtract(light_position, point);
    parser::Vec3f normalized_distance_vector = normalize(distance_vector);
    float cos_theta = dot(normal.direction, normalized_distance_vector);
    cos_theta = cos_theta > 0 ? cos_theta : 0;
    if (cos_theta == 0)
    {
        return {0, 0, 0};
    }

    float distance_squared = pow(distance_vector.x, 2) + pow(distance_vector.y, 2) + pow(distance_vector.z, 2);
    parser::Vec3f received_irradiance = divide(point_light.intensity, distance_squared);
    parser::Vec3f halfVec = add(normalized_distance_vector, outgoingVec);
    parser::Vec3f normalizedhalfVec = normalize(halfVec);

    float cos_alpha = std::max(float(0), dot(normal.direction, normalizedhalfVec));
    float cos_alpha_exponed = pow(cos_alpha, material.phong_exponent);
    parser::Vec3f result_1 = multiply(material.specular, received_irradiance);
    parser::Vec3f result = multiply_with_constant(result_1, cos_alpha_exponed);
    return result;
}

HitRecord findHitRecord(std::vector<parser::Triangle *> &triangles, std::vector<parser::Sphere *> &spheres, parser::Scene &scene, Ray &view_ray, std::vector<Ray *> &normals,BVHTreeTri& bvhTree)
{
    std::vector<parser::Vec3f> &vertices = scene.vertex_data;
    float min_t = __FLT_MAX__;
    parser::Vec3f hit_point = {0, 0, 0};
    Ray normal;
    parser::Material material;
    //std::vector<parser::Triangle*> triangleList;
    //bvhTree.getTriangles(view_ray,triangleList);
    for (int t = 0; t < triangles.size(); t++)
    {
        float intersection_t = intersect_triangle((*(triangles[t])).indices, view_ray, vertices);
        if (intersection_t < min_t && intersection_t > 0)
        {
            min_t = intersection_t;
            normal = *(normals[t]);
            material = scene.materials[(*(triangles[t])).material_id - 1];
            parser::Vec3f hit_point_temp = multiply_with_constant(view_ray.direction, intersection_t);
            hit_point = add(view_ray.origin, hit_point_temp);
        }
    }
    for (int s = 0; s < spheres.size(); s++)
    {
        parser::Sphere &sphere = *(spheres[s]);
        float intersection_t = intersect_sphere(vertices[sphere.center_vertex_id - 1], sphere.radius, view_ray);
        if (intersection_t < min_t && intersection_t > 0)
        {
            min_t = intersection_t;
            material = scene.materials[sphere.material_id - 1];
            parser::Vec3f tmp_d = multiply_with_constant(view_ray.direction, intersection_t);
            hit_point = add(view_ray.origin, tmp_d);
            tmp_d = subtract(hit_point, scene.vertex_data[sphere.center_vertex_id - 1]);
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

parser::Vec3f computeColor(std::vector<parser::Triangle *> &triangles, std::vector<parser::Sphere *> &spheres, parser::Scene &scene, Ray &view_ray, std::vector<Ray *> &normals, int current_depth, int max_depth,BVHTreeTri& bvhTree);

parser::Vec3f apply_shading(parser::Scene &scene, HitRecord &hit_record, Ray &view_ray, int current_depth, int max_depth, std::vector<Ray *> &normals, std::vector<parser::Triangle *> &triangles, std::vector<parser::Sphere *> &spheres,BVHTreeTri& bvhTree)
{
    parser::Material &material = hit_record.material;
    parser::Vec3f &hit_point = hit_record.hit_point;
    Ray &normal = hit_record.normal;
    parser::Vec3f color = ambient_shading(scene.ambient_light, material);

    if (material.is_mirror)
    {
        Ray reflect_ray;

        // May have to multiply view ray with -1
        float minusone = -1;
        parser::Vec3f wo = multiply_with_constant(view_ray.direction, minusone);

        float tmp = dot(wo, normal.direction);
        tmp *= 2;
        parser::Vec3f tmp2 = multiply_with_constant(normal.direction, tmp);
        parser::Vec3f tmp3 = add(tmp2, view_ray.direction);
        reflect_ray.direction = tmp3;
        reflect_ray.origin = hit_point;
        /*reflect_ray.origin.y += scene.shadow_ray_epsilon;*/
        parser::Vec3f o1 = multiply_with_constant(normal.direction, scene.shadow_ray_epsilon);
        reflect_ray.origin = add(reflect_ray.origin, o1);
        /*shadow ray epsilon should be added towards normal direction*/

        parser::Vec3f color2 = computeColor(triangles, spheres, scene, reflect_ray, normals, current_depth + 1, max_depth,bvhTree);
        color2 = multiply(color2, material.mirror);
        color = add(color, color2);
    }

    for (int l = 0; l < scene.point_lights.size(); l++)
    {
        parser::Vec3f light_position = scene.point_lights[l].position;
        parser::Vec3f distance_vector = subtract(light_position, hit_point);
        parser::Vec3f normalized_distance_vector = normalize(distance_vector);
        float t = distance_vector.x / normalized_distance_vector.x;
        Ray tmp_ray;
        tmp_ray.direction = normalized_distance_vector;
        /*parser::Vec3f epsilon = multiply_with_constant(normalized_distance_vector, scene.shadow_ray_epsilon);*/
        /*burada distance vector değil normalle çarpılması lazım*/
        parser::Vec3f epsilon = multiply_with_constant(normal.direction, scene.shadow_ray_epsilon);
        tmp_ray.origin = add(hit_point, epsilon);
        HitRecord lightHitRecord = findHitRecord(triangles, spheres, scene, tmp_ray, normals,bvhTree);
        if (lightHitRecord.min_t == __FLT_MAX__ || (lightHitRecord.min_t >= t && lightHitRecord.min_t > 0))
        {
            parser::Vec3f diffuse_shading_part = diffuse_shading(scene.point_lights[l], normal, hit_point, material);
            parser::Vec3f outgoingVec = view_ray.direction;
            outgoingVec.x = -outgoingVec.x;
            outgoingVec.y = -outgoingVec.y;
            outgoingVec.z = -outgoingVec.z;
            parser::Vec3f BlinnPhong_part = blinnphong_shading(scene.point_lights[l], normal, hit_point, material, outgoingVec);
            parser::Vec3f tmp = add(diffuse_shading_part, BlinnPhong_part);
            color = add(color, tmp);
        }
    }

    return color;
}

parser::Vec3f computeColor(std::vector<parser::Triangle *> &triangles, std::vector<parser::Sphere *> &spheres, parser::Scene &scene, Ray &view_ray, std::vector<Ray *> &normals, int current_depth, int max_depth,BVHTreeTri& bvhTree)
{
    if (current_depth >= max_depth)
    {
        parser::Vec3f tmp;
        tmp.x = 0, tmp.y = 0, tmp.z = 0;
        return tmp;
    }
    float min_t = __FLT_MAX__;
    HitRecord hit_record = findHitRecord(triangles, spheres, scene, view_ray, normals,bvhTree);
    parser::Vec3f &hit_point = hit_record.hit_point;
    parser::Material &material = hit_record.material;
    Ray &normal = hit_record.normal;
    min_t = hit_record.min_t;
    parser::Vec3f resultingColor;

    if (min_t == __FLT_MAX__)
    {
        if(current_depth==0){
            resultingColor.x = scene.background_color.x;
            resultingColor.y = scene.background_color.y;
            resultingColor.z = scene.background_color.z;
        }
    }
    else
    {

        resultingColor = apply_shading(scene, hit_record, view_ray, current_depth, max_depth, normals, triangles, spheres,bvhTree);
        resultingColor.x = resultingColor.x > 255 ? 255 : resultingColor.x;
        resultingColor.y = resultingColor.y > 255 ? 255 : resultingColor.y;
        resultingColor.z = resultingColor.z > 255 ? 255 : resultingColor.z;
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
    std::vector<parser::Triangle> pre_triangles = scene.triangles;
    std::vector<parser::Material> materials = scene.materials;
    std::vector<parser::Sphere *> spheres;
    for (int m = 0; m < meshes.size(); m++)
    {
        for (int f = 0; f < meshes[m].faces.size(); f++)
        {
            parser::Triangle triangle;
            triangle.indices = meshes[m].faces[f];
            triangle.material_id = meshes[m].material_id;
            pre_triangles.push_back(triangle);
        }
    }
    std::vector<Ray> pre_normals(pre_triangles.size());
    for (int t = 0; t < pre_triangles.size(); t++)
    {
        parser::Vec3f direction_one = subtract(vertices[pre_triangles[t].indices.v0_id - 1], vertices[pre_triangles[t].indices.v2_id - 1]);
        parser::Vec3f direction_two = subtract(vertices[pre_triangles[t].indices.v1_id - 1], vertices[pre_triangles[t].indices.v2_id - 1]);
        parser::Vec3f normal = cross_product(direction_one, direction_two);
        parser::Vec3f normalized_normal = normalize(normal);
        pre_normals[t] = Ray{vertices[pre_triangles[t].indices.v2_id - 1], normalized_normal};
    }
    for (int s = 0, size = scene.spheres.size(); s < size; s++)
    {
        spheres.push_back(&scene.spheres[s]);
    }
    std::vector<Ray *> normals;
    std::vector<parser::Triangle *> triangles;
    std::vector<MyTriangle> bvh_triangles;
    std::vector<Ray>::iterator normal_start = pre_normals.begin();
    for(std::vector<parser::Triangle>::iterator start = pre_triangles.begin(), end = pre_triangles.end(); start != end; start++, normal_start++){
        parser::Triangle& tri = *start;
        parser::Vec3f& vec0=vertices[tri.indices.v0_id-1],vec1=vertices[tri.indices.v1_id-1],vec2=vertices[tri.indices.v2_id-1];
        parser::Vec3f center = add(vec0,vec1);
        center = add (vec2,center);
        center = divide(center,3);
        MyTriangle tmp;
        tmp.center = center;
        parser::Vec3f min  = vec0;
        getMin(vec1,min);
        getMin(vec2,min);
        parser::Vec3f max  = vec0;
        getMax(vec1,max);
        getMax(vec2,max);
        tmp.max = max;
        tmp.min = min;
        tmp.tri = &(*start);
        tmp.normal = &(*normal_start);
        bvh_triangles.push_back(tmp);

    }

    BVHTreeTri bvhTree(2,bvh_triangles);

    for (int i = 0; i < scene.cameras.size(); i++)
    {
        parser::Camera cam = scene.cameras[i];
        int height = cam.image_height;
        int width = cam.image_width;
        unsigned char *image = new unsigned char[height * width * 3];
        std::string name = cam.image_name;
        float l = cam.near_plane.x;
        float r = cam.near_plane.y;
        float b = cam.near_plane.z;
        float t = cam.near_plane.w;
        int print_counter = 0;
        float distance = cam.near_distance;
        parser::Vec3f u = cross_product(cam.gaze, cam.up);
        u = normalize(u);
        parser::Vec3f m_distance = multiply_with_constant(cam.gaze, distance);
        parser::Vec3f m = add(cam.position, m_distance);
        parser::Vec3f tmp = multiply_with_constant(u, l);
        parser::Vec3f tmp2 = multiply_with_constant(cam.up, t);
        parser::Vec3f q = add(m, tmp);
        q = add(q, tmp2);

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                // Compute view ray
                parser::Vec3f q_u = multiply_with_constant(u, (x + 0.5) * (r - l) / width);
                parser::Vec3f q_v = multiply_with_constant(cam.up, (y + 0.5) * (t - b) / height);
                parser::Vec3f s;
                s = subtract(q_u, q_v);
                s = add(q, s);
                parser::Vec3f d = subtract(s, cam.position);

                Ray view_ray;
                view_ray.origin = cam.position;
                view_ray.direction = normalize(d);

                if (print_counter++ == 500)
                {
                    float percentage = (float)(y * width + x) / (height * width) * 100;
                    printf("Percentage finished: %.2f\n", percentage);
                    print_counter = 0;
                }
                // compute color
                parser::Vec3f color = computeColor(triangles, spheres, scene, view_ray, normals, 0, scene.max_recursion_depth,bvhTree);
                image[3 * (y * width + x)] = std::round(color.x);
                image[3 * (y * width + x) + 1] = std::round(color.y);
                image[3 * (y * width + x) + 2] = std::round(color.z);
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Time taken: " << duration << " milliseconds" << std::endl;
        write_ppm(name.c_str(), image, width, height);
    }
}
