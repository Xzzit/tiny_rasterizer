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
auto to_vec4(const Vector3f& v3, float w = 1.0f)
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

// Compute AABB
Vector4i compute_AABB(Vector4f v[], int width, int height)
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
Vector3f random_color()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 255);

    return Vector3f(dis(gen), dis(gen), dis(gen));
}

// Check if a point is inside the triangle
bool inside_triangle(Vector2f point, Vector4f v[])
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
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
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
            vec.x() = (vec.x() + 1.0f) * 0.5f * width; // [0, width]
            vec.y() = (vec.y() + 1.0f) * 0.5f * height; // [0, height]
            vec.z() = f1 * vec.z() + f2; // [near, far]
        }

        // Compute AABB
        Vector4i aabb = compute_AABB(v, this->get_width(), this->get_height());
        Vector3f t_color = random_color();

        // Rasterization
        for (int x = aabb.x(); x <= aabb.y(); x++)
        {
            for (int y = aabb.z(); y <= aabb.w(); y++)
            {
                Vector2f point = Vector2f(x + 0.5f, y + 0.5f);
                float z_interpolated = std::numeric_limits<float>::infinity();

                if (inside_triangle(point, v))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D(point.x(), point.y(), v);
                    z_interpolated = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();

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