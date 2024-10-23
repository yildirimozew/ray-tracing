#include <iostream>
#include "parser.h"
#include "ppm.h"
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



int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);
    std::vector<parser::Vec3f> vertices = scene.vertex_data;
    std::vector<parser::Mesh> meshes = scene.meshes;
    std::vector<parser::Triangle> triangles = scene.triangles;
    std::vector<Ray> normals(triangles.size());
    for (int t = 0; t < triangles.size(); t++){
        parser::Vec3f direction_one = subtract(vertices[scene.triangles[t].indices.v0_id], vertices[scene.triangles[t].indices.v2_id]);
        parser::Vec3f direction_two = subtract(vertices[scene.triangles[t].indices.v1_id], vertices[scene.triangles[t].indices.v2_id]);
        Ray one = {vertices[scene.triangles[t].indices.v2_id], direction_one};
        Ray two = {vertices[scene.triangles[t].indices.v2_id], direction_two};
        normals[t] = Ray{vertices[scene.triangles[t].indices.v2_id] , cross_product(one.direction, two.direction)};
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
                parser::Vec3f s = add(q, add(multipy_with_constant(u, s_u), multipy_with_constant(cam.up, -s_v)));
                Ray view_ray = {cam.position, subtract(s, cam.position)};
            }
        };

        write_ppm(name.c_str(), image, width, height);
    }



    

}
