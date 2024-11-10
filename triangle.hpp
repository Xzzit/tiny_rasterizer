#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H


#include <eigen3/Eigen/Eigen>

class Triangle
{
    public:
        Triangle();

        Eigen::Vector4f v[3];
        Eigen::Vector3f color[3];
        Eigen::Vector3f normal[3];
        Eigen::Vector2f tex_coords[3];

        void set_vertex(int ind, Eigen::Vector4f ver);
        void set_color(int ind, Eigen::Vector3f col);
        void set_normal(int ind, Eigen::Vector3f nor);
        void set_tex_coords(int ind, Eigen::Vector2f tex);
};

#endif //RASTERIZER_TRIANGLE_H