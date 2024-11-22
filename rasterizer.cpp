#include <iostream>
#include <cmath>
#include <random>

#include "rasterizer.hpp"
#include "triangle.hpp"
using namespace Eigen;

const float PI = 3.1415926f;

// Initialize the rasterizer
rst::rasterizer::rasterizer(int w, int h, int sample_rate)
{
    // Initialize the frame buffer and depth buffer
    width = w;
    height = h;
    frame_buf = std::vector<Vector3f>(w * h);
    depth_buf = std::vector<float>(w * h * sample_rate);

    // Clear the frame buffer
    clear();

    // Set M 
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Set V
    Eigen::Vector3f eye_pos = Eigen::Vector3f(0, 0, 2.5);
    Eigen::Matrix4f view;
    view << 1, 0, 0, -eye_pos[0],
            0, 1, 0, -eye_pos[1],
            0, 0, 1, -eye_pos[2],
            0, 0, 0, 1;

    // Set P
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

    // Set MVP matrix
    set_model(model);
    set_view(view);
    set_projection(perspective);
}

// Clear the frame buffer
void rst::rasterizer::clear()
{
    std::fill(frame_buf.begin(), frame_buf.end(), Vector3f(0, 0, 0));
    std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
}

// Set MVP matrix
void rst::rasterizer::set_model(const Matrix4f &m)
{
    model = m;
}

void rst::rasterizer::set_view(const Matrix4f &v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Matrix4f &p)
{
    projection = p;
}

// Homogeneous coordinates transformation
Vector4f rst::rasterizer::to_vec4(const Vector3f& v3, float w)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// Set a pixel
void rst::rasterizer::set_pixel(Vector2i point, Vector3f color)
{
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;

    auto ind = (height - point.y() - 1) * width + point.x();
    frame_buf[ind] = color;
}