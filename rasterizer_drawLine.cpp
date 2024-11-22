#include <iostream>
#include <cmath>
#include <random>

#include "rasterizer.hpp"
#include "triangle.hpp"
using namespace Eigen;

const float PI = 3.1415926f;

// Draw a line
void rst::rasterizer::draw_line(Vector3f begin, Vector3f end)
{
    int x1 = begin.x();
    int y1 = begin.y();
    int x2 = end.x();
    int y2 = end.y();

    // Check if two points are out of the screen, return if true
    if ((x1 < 0 && x2 < 0) || (x1 >= width && x2 >= width) ||
        (y1 < 0 && y2 < 0) || (y1 >= height && y2 >= height)) return;

    // Set the color of the line
    Eigen::Vector3f line_color = {255, 255, 255};

    // Make sure begin is on the left
    if (x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    int dx = x2 - x1;
    int dy = y2 - y1;

    if (dx == 0)
    {
        if (y1 > y2)
        {
            std::swap(y1, y2);
        }
        for (int y = y1; y <= y2; y++)
        {
            Vector2i point = Vector2i(x1, y);
            set_pixel(point, line_color);
        }
        return;
    }

    float k = float(dy) / float(dx);

    if (std::abs(k) > 1)
    {
        if (y1 > y2)
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        for (int y = y1; y <= y2; y++)
        {
            int x = (y - y1) / k + x1;
            Vector2i point = Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
    else
    {
        for (int x = x1; x <= x2; x++)
        {
            int y = k * (x - x1) + y1;
            Vector2i point = Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
}

// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line_bresenham(Vector3f begin, Vector3f end)
{
    int x1 = begin.x();
    int y1 = begin.y();
    int x2 = end.x();
    int y2 = end.y();

    // Check if two points are out of the screen, return if true
    if ((x1 < 0 && x2 < 0) || (x1 >= width && x2 >= width) ||
        (y1 < 0 && y2 < 0) || (y1 >= height && y2 >= height)) return;

    // Set the color of the line
    Eigen::Vector3f line_color = {255, 255, 255};

    // Bresenham's line algorithm
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        set_pixel(Vector2i(x1, y1), line_color);

        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}

// Draw the triangle
void rst::rasterizer::rasterize_line(std::vector<Triangle*> TriangleList)
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
            vec.x() = (vec.x() + 1.0f) * 0.5f * width; // [0, width]
            vec.y() = (vec.y() + 1.0f) * 0.5f * height; // [0, height]
            vec.z() = f1 * vec.z() + f2; // [near, far]
        }

        // Rasterization
        for (int i = 0; i < 3; i++)
        {
            if (i == 2)
            {
                Vector3f begin = {v[2].x(), v[2].y(), v[2].z()};
                Vector3f end = {v[0].x(), v[0].y(), v[0].z()};
                draw_line(begin, end);
                // draw_line_bresenham(begin, end);
            }
            else
            {
                Vector3f begin = {v[i].x(), v[i].y(), v[i].z()};
                Vector3f end = {v[i + 1].x(), v[i + 1].y(), v[i + 1].z()};
                draw_line(begin, end);
                // draw_line_bresenham(begin, end);
            }
        }
    }
}