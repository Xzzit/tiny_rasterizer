#include <iostream>
#include <cmath>

#include "rasterizer.hpp"
#include "triangle.hpp"
using namespace Eigen;

// Initialize the rasterizer
rst::rasterizer::rasterizer(int w, int h)
{
    width = w;
    height = h;
    frame_buf = std::vector<Vector3f>(w * h);
}

// Clear the frame buffer
void rst::rasterizer::clear()
{
    std::fill(frame_buf.begin(), frame_buf.end(), Vector3f(0, 0, 0));
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
    if (point.x() < 0 || point.x() > width ||
        point.y() < 0 || point.y() > height) return;

    auto ind = (height - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

// Draw a line
// Note: for close points (at x axis), the line may not be drawn
void rst::rasterizer::draw_line(Vector3f begin, Vector3f end)
{
    int x1 = begin.x();
    int y1 = begin.y();
    int x2 = end.x();
    int y2 = end.y();

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
void rst::rasterizer::draw_line_bresenham(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
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