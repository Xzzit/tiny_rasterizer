#ifndef RASTERIZER_RASTERIZER_H
#define RASTERIZER_RASTERIZER_H


#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include "triangle.hpp"

namespace rst
{
    class rasterizer
    {
        public:
            rasterizer(int w, int h, int sample_rate=1);

            void clear();

            void set_model(const Eigen::Matrix4f &m);
            void set_view(const Eigen::Matrix4f &v);
            void set_projection(const Eigen::Matrix4f &p);

            Eigen::Vector4f to_vec4(const Eigen::Vector3f& v3, float w = 1.0f);

            // Draw a line
            void set_pixel(Eigen::Vector2i point, Eigen::Vector3f color);
            void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
            void draw_line_bresenham(Eigen::Vector3f begin, Eigen::Vector3f end);
            void rasterize_line(std::vector<Triangle*> TriangleList);

            // Draw a solid triangle
            Eigen::Vector4i compute_AABB(Eigen::Vector4f v[], int width, int height);
            bool inside_triangle(Eigen::Vector2f point, Eigen::Vector4f v[]);
            Eigen::Vector3f random_color();
            static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Eigen::Vector4f* v);
            void rasterize_triangle(std::vector<Triangle*> TriangleList);

            // Blinn-Phong shading
            void rasterize_BlinnPhong(std::vector<Triangle*> TriangleList);

            std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }
            std::vector<float>& depth_buffer() { return depth_buf; }

            int get_width() { return width; }
            int get_height() { return height; }
            
        private:
            int width, height;

            std::vector<Eigen::Vector3f> frame_buf;
            std::vector<float> depth_buf;

            Eigen::Matrix4f model;
            Eigen::Matrix4f view;
            Eigen::Matrix4f projection;
    };
}

#endif //RASTERIZER_RASTERIZER_H