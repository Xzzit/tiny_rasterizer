#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "rasterizer.hpp"
#include "obj_loader.hpp"
#include "triangle.hpp"

const float PI = 3.1415926f;

int main(int argc, const char *argv[])
{
    // Initialize the rasterizer
    int width = 700;
    int height = 700;
    rst::rasterizer r(width, height);
    r.clear();

    // Set camera position
    Eigen::Vector3f eye_pos = Eigen::Vector3f(-1, 0, 2);

    // Load the obj file
    objl::Loader Loader;
    std::string model_path = "/home/ray/tiny_renderer/models/spot.obj";
    bool loadout = Loader.LoadFile(model_path);

    if (!loadout)
    {
        std::cerr << "Failed to load obj file." << std::endl;
        return -1;
    }

    // Convert to triangle list
    std::vector<Triangle*> TriangleList;

    for (objl::Mesh curMesh : Loader.LoadedMeshes)
    {
        // Adjecent 3 vertices form a triangle
        for (int i = 0; i < curMesh.Vertices.size(); i += 3)
        {
            Triangle* t = new Triangle();

            // Store the vertices
            for (int j = 0; j < 3; j++)
            {
                t->set_vertex(j, Eigen::Vector4f(
                    curMesh.Vertices[i + j].Position.X, 
                    curMesh.Vertices[i + j].Position.Y, 
                    curMesh.Vertices[i + j].Position.Z, 
                    1.0f));
            }

            TriangleList.push_back(t);
        }
    }

    // Set MVP matrix
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f view;
    view << 1, 0, 0, -eye_pos[0],
            0, 1, 0, -eye_pos[1],
            0, 0, 1, -eye_pos[2],
            0, 0, 0, 1;

    float eye_fov = 45;
    float aspect_ratio = 1;
    float zNear = 0.1;
    float zFar = 50;
    Eigen::Matrix4f perspective;
    perspective <<  zNear / (aspect_ratio * zNear * tan(eye_fov / 360 * PI)), 0, 0, 0,
                    0, zNear / (zNear * tan(eye_fov / 360 * PI)), 0, 0,
                    0, 0, -(zFar + zNear) / (zFar - zNear), -2 * zFar * zNear / (zFar - zNear),
                    0, 0, -1, 0;

    Eigen::Matrix4f orthographic;

    r.set_model(model);
    r.set_view(view);
    r.set_projection(perspective);

    // Draw the triangle & measure the time
    auto start = std::chrono::high_resolution_clock::now();

    r.rasterize_line(TriangleList); // Draw the triangle with lines
    // r.rasterize_triangle(TriangleList); // Draw the triangle with solid random color

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Rasterize function took " << duration.count() << " seconds." << std::endl;

    // Save the frame buffer
    cv::Mat image(r.get_height(), r.get_width(), CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);

    std::string filename = "output.png";
    cv::imwrite(filename, image);

    return 0;
}