#include "triangle.hpp"
using namespace Eigen;

// Initialize the triangle
Triangle::Triangle()
{
    for (int i = 0; i < 3; i++)
    {
        v[i] = Vector4f(0, 0, 0, 1);
        color[i] = Vector3f(0, 0, 0);
        normal[i] = Vector3f(0, 0, 0);
        tex_coords[i] = Vector2f(0, 0);
    }
}

// Load the vertex
void Triangle::set_vertex(int ind, Vector4f ver)
{
    v[ind] = ver;
}

// Load the color
void Triangle::set_color(int ind, Vector3f col)
{
    color[ind] = col;
}

// Load the normal
void Triangle::set_normal(int ind, Vector3f nor)
{
    normal[ind] = nor;
}

// Load the texture coordinates
void Triangle::set_tex_coords(int ind, Vector2f tex)
{
    tex_coords[ind] = tex;
}