#include <iostream>
#include "parser.h"
#include "ppm.h"
/*may need to add pthread*/

typedef unsigned char RGB[3];

struct Ray{
    parser::Vec3f origin;
    parser::Vec3f direction;
};

parser::Vec3f cross_product(Ray a, Ray b){
    parser::Vec3f c;
    c.x = a.direction.y * b.direction.z - a.direction.z * b.direction.y;
    c.y = a.direction.z * b.direction.x - a.direction.x * b.direction.z;
    c.z = a.direction.x * b.direction.y - a.direction.y * b.direction.x;
    return c;
}

parser::Vec3f subtract(parser::Vec3f a, parser::Vec3f b){
    parser::Vec3f c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
}



int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);
    std::vector vertices = scene.vertex_data;
    std::vector meshes = scene.meshes;
    std::vector triangles = scene.triangles;
    std::vector<Ray> normals(triangles.size());
    for (int t = 0; t < triangles.size(); t++){
        parser::Vec3f direction_one = subtract(vertices[scene.triangles[t].indices.v0_id], vertices[scene.triangles[t].indices.v2_id]);
        parser::Vec3f direction_two = subtract(vertices[scene.triangles[t].indices.v1_id], vertices[scene.triangles[t].indices.v2_id]);
        Ray one = {vertices[scene.triangles[t].indices.v2_id], direction_one};
        Ray two = {vertices[scene.triangles[t].indices.v2_id], direction_two};
        normals[t] = Ray{vertices[scene.triangles[t].indices.v2_id] , cross_product(one, two)};
    }

        for (int i = 0; i < scene.cameras.size(); i++)
        {
            int height = scene.cameras[i].image_height;
            int width = scene.cameras[i].image_width;
            std::string name = scene.cameras[i].image_name;
            unsigned char *image = new unsigned char[height * width * 3];

            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                }
            };

            write_ppm(name.c_str(), image, width, height);
        }



    

}
