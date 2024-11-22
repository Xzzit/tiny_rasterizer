#include <iostream>
#include <cmath>
#include <random>

#include "rasterizer.hpp"
#include "triangle.hpp"
using namespace Eigen;

const float PI = 3.1415926f;

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

void rst::rasterizer::rasterize_BlinnPhong(std::vector<Triangle*> TriangleList)
{
    // Set the light
    light l = {{10, 10, 10}, {500, 500, 500}};

    // Calculate eye position
    Eigen::Vector3f eye_pos = {0, 0, 2.5};

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

        // Compute view space coordinates
        std::array<Eigen::Vector4f, 3> mm {
            (view * model * t->v[0]),
            (view * model * t->v[1]),
            (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        // Store the view space coordinates of the triangle vertices
        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        // Compute normal
        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        // Create new triangle
        Triangle newtri = *t;
        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.set_vertex(i, v[i]);
        }
        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.set_normal(i, n[i].head<3>());
        }

        // Compute AABB
        Vector4i aabb = compute_AABB(v, this->get_width(), this->get_height());
        Vector3f t_color = random_color();
        // Vector3f t_color = {148, 121,92};

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

                        // Interpolate normal
                        Eigen::Vector3f interpolated_normal = interpolate(alpha, beta, gamma, newtri.normal[0], newtri.normal[1], newtri.normal[2], 1.0f).normalized();

                        // Interpolate view space position
                        Eigen::Vector3f interpolated_viewspace_pos = interpolate(alpha, beta, gamma, viewspace_pos[0], viewspace_pos[1], viewspace_pos[2], 1.0f);
                        
                        // Diffuse lighting
                        Eigen::Vector3f light_dir = (l.position - interpolated_viewspace_pos).normalized();
                        float diff = std::max(0.0f, interpolated_normal.dot(light_dir));
                        float r = (l.position - interpolated_viewspace_pos).norm();
                        Eigen::Vector3f Ld = t_color.cwiseProduct(l.intensity / std::pow(r, 2)) * diff;

                        // Specular lighting
                        Eigen::Vector3f view_dir = (eye_pos - interpolated_viewspace_pos).normalized();
                        Eigen::Vector3f h = (view_dir + light_dir).normalized();
                        float spec = std::pow(std::max(0.0f, interpolated_normal.dot(h)), 128);
                        Eigen::Vector3f Ls = t_color.cwiseProduct(l.intensity / std::pow(r, 2)) * spec;

                        // Ambient lighting
                        Eigen::Vector3f La = Eigen::Vector3f(10, 10, 10);

                        // Final color
                        Eigen::Vector3f color = Ls + Ld + La;

                        set_pixel(Vector2i(x, y), color);
                    }
                }
            }
        }
    }
}