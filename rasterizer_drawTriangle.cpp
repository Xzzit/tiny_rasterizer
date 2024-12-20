#include <iostream>
#include <cmath>
#include <random>

#include "rasterizer.hpp"
#include "triangle.hpp"
using namespace Eigen;

const float PI = 3.1415926f;

// Compute AABB
Vector4i rst::rasterizer::compute_AABB(Vector4f v[], int width, int height)
{
    Vector4i aabb = Vector4i::Zero();

    int left = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    int right = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    int bottom = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    int top = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    if (left >= width || right < 0 || bottom >= height || top < 0)
    {
        return aabb;
    }

    left = std::max(0, left);
    right = std::min(right, width - 1);
    bottom = std::max(0, bottom);
    top = std::min(top, height - 1);

    aabb << left, right, bottom, top;

    return aabb;
}

// Generate random color
Vector3f rst::rasterizer::random_color()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 255);

    return Vector3f(dis(gen), dis(gen), dis(gen));
}

// Check if a point is inside the triangle
bool rst::rasterizer::inside_triangle(Vector2f point, Vector4f v[])
{
    Vector2f AB = Vector2f(v[1].x() - v[0].x(), v[1].y() - v[0].y());
    Vector2f BC = Vector2f(v[2].x() - v[1].x(), v[2].y() - v[1].y());
    Vector2f CA = Vector2f(v[0].x() - v[2].x(), v[0].y() - v[2].y());

    Vector2f AP = Vector2f(point.x() - v[0].x(), point.y() - v[0].y());
    Vector2f BP = Vector2f(point.x() - v[1].x(), point.y() - v[1].y());
    Vector2f CP = Vector2f(point.x() - v[2].x(), point.y() - v[2].y());

    float r1 = AB.x() * AP.y() - AB.y() * AP.x();
    float r2 = BC.x() * BP.y() - BC.y() * BP.x();
    float r3 = CA.x() * CP.y() - CA.y() * CP.x();

    return (r1 >= 0 && r2 >= 0 && r3 >= 0) || (r1 <= 0 && r2 <= 0 && r3 <= 0);
}

// Barycentric interpolation
std::tuple<float, float, float> rst::rasterizer::computeBarycentric2D(float x, float y, const Vector4f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1, c2, c3};
}

// Draw the solid triangle
void rst::rasterizer::rasterize_triangle(std::vector<Triangle*> TriangleList)
{
    // Calculate the MVP matrix
    Matrix4f mvp = projection * view * model;

    // Loop through each triangle
    for (const auto& t : TriangleList)
    {
        // MVP transformation for each vertex
        Vector4f v[] =
        {
            mvp * t->v[0],
            mvp * t->v[1],
            mvp * t->v[2]
        };

        // Perspective division
        for (auto &vec : v)
        {
            vec /= vec.w();
        }

        // Viewport transformation
        float near = 0.01f, far = 50.0f;
        float f1 = (far - near) / 2.0f;
        float f2 = (far + near) / 2.0f;
        for (auto &vec : v)
        {
            vec.x() = ((vec.x() + 1.0f) * width - 1.0) * 0.5f; // [0, width-1]
            vec.y() = ((vec.y() + 1.0f) * height - 1.0) * 0.5f; // [0, height-1]
            vec.z() = f1 * vec.z() + f2; // [near, far]
        }

        // Frustum culling
        if (v[0].z() < near || v[0].z() > far ||
            v[1].z() < near || v[1].z() > far ||
            v[2].z() < near || v[2].z() > far)
        {
            continue;
        }

        // Compute AABB
        Vector4i aabb = compute_AABB(v, this->get_width(), this->get_height());
        
        // Return if the triangle is out of the screen
        if (aabb.y()==0 || aabb.w()==0)
        {
            continue;
        }

        Vector3f t_color = random_color();

        // Rasterization
        for (int x = aabb.x(); x <= aabb.y(); x++)
        {
            for (int y = aabb.z(); y <= aabb.w(); y++)
            {
                Vector2f point = Vector2f(x + 0.5f, y + 0.5f);
                
                if (inside_triangle(point, v))
                {
                    // Barycentric interpolation
                    auto [alpha, beta, gamma] = computeBarycentric2D(point.x(), point.y(), v);
                    float z_interpolated = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();

                    // Update depth buffer
                    int ind = (height - y - 1) * width + x;
                    if (z_interpolated < depth_buf[ind])
                    {
                        depth_buf[ind] = z_interpolated;
                        set_pixel(Vector2i(x, y), t_color);
                    }
                }
            }
        }
    }
}