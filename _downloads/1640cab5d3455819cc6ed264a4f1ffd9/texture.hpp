#pragma once

#include "crobot/math/matrix.hpp"

namespace crobot::gui {

// Represents a texture/material
struct Texture
{
  // Texture types
  enum
  {
    TYPE_DIFFUSE,
    TYPE_SPECULAR,
    // TYPE_NORMAL,
    // TYPE_HEIGHT,
    __TYPE_COUNT
  };
  // Texture type names (TYPE_DIFFUSE -> "diffuse" etc)
  static constexpr inline const char* type_to_name(int type) {
    switch (type) {
      case TYPE_SPECULAR:
        return "specular";
      // case TYPE_NORMAL: return "normal";
      // case TYPE_HEIGHT: return "height";
      default:
        return "diffuse";
    }
  }

  // Texture from path
  Texture(const std::string& path, bool flip = true);
  // Texture from solid color (1x1)
  Texture(float r, float g, float b);
  // Texture from an (rows, cols*3) row-major
  // red(1-channel)/RGB(3-channel)/RGBA(4-channel) f32 matrix
  Texture(const Eigen::Ref<const math::Image>& im, int n_channels = 1);

  ~Texture();

  // Load texture; need to be called before first use in each context
  // (called by Mesh::reset())
  void load();

  // ADVANCED: free buffers, called by destructor
  void free_bufs();

  // GL texture id; -1 if unavailable
  math::Index id = -1;

private:
  // File path (optional)
  std::string path;

  // Image data (optional)
  math::Image im_data;
  // Channels in above
  int n_channels;

  // Color to use if path empty OR failed to load the texture image
  math::Vector3f fallback_color;

  // Vertical flip on load?
  bool flip;
};

} // namespace crobot::gui

