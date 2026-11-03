#pragma once

#include "crobot/gui/camera.hpp"
#include "crobot/gui/texture.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::gui {

// Represents a triangle mesh with vertices (including uv, normals),
// triangular faces, and textures
class Mesh {
public:
  enum class ShadingType
  {
    // Color by linear interpolation from vertices, use vertex color as rgb
    vertex,
    // Color by texture, use color info as texture uv coordinates
    texture
  };

  // Construct uninitialized mesh with given number of vertices, triangles
  explicit Mesh(size_t num_verts = 0, size_t num_triangles = 0);
  // Construct from basic OBJ (see load_basic_obj)
  explicit Mesh(const std::string& path);
  // Construct with given points, faces, and (optionally) color data
  // faces: triangles. if not specified, assumes they are 0 1 2, 3 4 5 etc
  // rgb: optional per-vertex color (ignored if you call set_tex_coords later)
  // normals: normal vectors per point.
  // if not specified, will compute automatically on update
  explicit Mesh(
      const Eigen::Ref<const math::Points>& pos,
      const Eigen::Ref<const math::Triangles>& tri_faces,
      const Eigen::Ref<const math::Points>& rgb,
      const Eigen::Ref<const math::Points>& normals = math::Points());
  // Construct with given points, faces, and (optionally) rgb color for all
  // vertices r,g,b: the color given to all vertices (ignored if you call
  // set_tex_coords later) normals: normal vectors per point. if not
  // specified, will compute automatically on update
  explicit Mesh(
      const Eigen::Ref<const math::Points>& pos,
      const Eigen::Ref<const math::Triangles>& tri_faces = math::Triangles(),
      float r                                            = 1.f,
      float g                                            = 1.f,
      float b                                            = 1.f,
      const Eigen::Ref<const math::Points>& normals      = math::Points());
  ~Mesh();

  // Resize mesh (destroys current data)
  void resize(size_t num_verts, size_t num_triangles = 0);

  // Draw mesh to shader wrt camera
  void draw(math::Index shader_id, const Camera& camera);

  // Set the texture coordinates with given texture triangles
  // (shading mode will be set to texture)
  Mesh& set_tex_coords(
      const Eigen::Ref<const math::Points2D>& coords,
      const Eigen::Ref<const math::Triangles>& tri_faces);

  // Remove texture coordinates data
  // (shading mode will be set to vertex)
  Mesh& unset_tex_coords();

  // Add a texture. Arguments are passed to Texture constructor
  // returns self (for chaining)
  template <int Type = Texture::TYPE_DIFFUSE, typename... Args>
  Mesh& add_texture(Args&&... args) {
    textures[Type].emplace_back(std::forward<Args>(args)...);
    return *this;
  }

  // Set specular shininess parameter
  Mesh& set_shininess(float val);

  // * Transformations
  // Apply translation
  Mesh& translate(const Eigen::Ref<const math::Vector3f>& vec);
  // Set translation
  Mesh& set_translation(const Eigen::Ref<const math::Vector3f>& vec);

  // Apply rotation (3x3 rotation matrix)
  Mesh& rotate(const Eigen::Ref<const math::Matrix3f>& mat);

  // Apply scaling
  Mesh& scale(const Eigen::Ref<const math::Vector3f>& vec);
  // Apply uniform scaling
  Mesh& scale(float val);

  // Apply transform (4x4 affine transform matrix)
  Mesh& apply_transform(const Eigen::Ref<const math::Matrix4f>& mat);
  // Set transform (4x4 affine transform matrix)
  Mesh& set_transform(const Eigen::Ref<const math::Matrix4f>& mat);

  // Init or update VAO/VBO/EBO buffers from current vertex and triangle data
  // Must called before first draw for each GLFW context to ensure
  // textures are reconstructed.
  void update(bool force_init = false);

  // ADVANCED: Free buffers. Used automatically in destructor.
  void free_bufs();

  // * Accessors
  // Position data of vertices (#verts, 3).
  inline Eigen::Ref<math::Points> verts_pos() {
    return data.leftCols<3>();
  }

  // The optional RGB data of each vertex (#verts, 3).
  inline Eigen::Ref<math::Points> verts_rgb() {
    return data.middleCols<3>(3);
  }

  // ADVANCED: Normal vectors data (#verts, 3).
  // If called, this disables automatic normal computation.
  inline Eigen::Ref<math::Points> verts_norm() {
    _auto_normals = false;
    return data.rightCols<3>();
  }

  // Enable/disable object
  Mesh& enable(bool val = true);

  // * VERY basic OBJ I/O
  // - Only supports v and f types
  // - Each f row must consist of three integers (triangle indices)
  // - Each v row may consist of 3 OR 6 floats (position, or position+rgb
  // color)
  void save_basic_obj(const std::string& path) const;
  void load_basic_obj(const std::string& path);

  // * Example meshes
  // Triangle
  static Mesh Triangle(
      const Eigen::Ref<const math::Vector3f>& a,
      const Eigen::Ref<const math::Vector3f>& b,
      const Eigen::Ref<const math::Vector3f>& c);
  // Square centered at 0,0,0 with normal in z direction and side length 1
  static Mesh Square();

  // Cube centered at 0,0,0 with side length 1
  static Mesh Cube();

  // UV sphere centered at 0,0,0 with radius 1
  static Mesh Sphere(int rings = 30, int sectors = 30);

  // Shape (num_verts, 9)
  // 3 x vertex position
  // 3 x vertex color data (either uv coords, or rgb color, depending on
  // shading_type) 3 x normal
  // TODO: make some of the columns optional/only transfer partly
  // to save data transfer time and memory
  math::PointsRGBNormal data;

  // Shape (num_triangles, 3)
  // Triangle indices, empty if num_triangles = -1 (not using EBO)
  math::Triangles faces;

  // Whether this mesh is enabled; if false, does not draw anything
  bool enabled = true;

  // Textures
  std::array<std::vector<Texture>, Texture::__TYPE_COUNT> textures;

  // Shininess
  float shininess = 10.f;

  // Model local transfom
  math::Matrix4f transform;

  // Shading type : interpretation of middle 3 columns in verts
  // the right shader will be chosen by viewer (mesh does not have control)
  ShadingType shading_type = ShadingType::vertex;

private:
  // Generate a white 1x1 texture to blank_tex_id
  // used to fill maps if no texture provided
  void gen_blank_texture();

  // Vertex Array Object index
  math::Index VAO = -1;

  math::Index VBO = -1, EBO = -1;
  math::Index blank_tex_id = -1;

  // Texture data
  math::Points2D _tex_coords;
  math::Triangles _tex_faces;

  // Data, arranged in texture coord indexing
  math::PointsRGBNormal _data_tex;
  // Map from texture coord index -> vertex index
  Eigen::Matrix<math::Index, Eigen::Dynamic, 1> _tex_to_vert;

  // Whether to use automatic normal estimation (get normals automatically on
  // update)
  bool _auto_normals = true;
};

} // namespace crobot::gui

