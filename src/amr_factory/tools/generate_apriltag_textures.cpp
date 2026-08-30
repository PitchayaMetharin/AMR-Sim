#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "apriltag.h"
#include "common/image_u8.h"
#include "tag36h11.h"

int main(int argc, char ** argv) {
  if (argc != 2) {
    std::cerr << "usage: generate_apriltag_textures OUTPUT_DIRECTORY\n";
    return 2;
  }
  const std::filesystem::path output_directory(argv[1]);
  std::filesystem::create_directories(output_directory);
  const auto mesh_directory = output_directory.parent_path().parent_path() / "meshes";
  std::filesystem::create_directories(mesh_directory);
  apriltag_family_t * family = tag36h11_create();
  constexpr std::array<std::uint32_t, 7> ids{{10, 11, 12, 20, 101, 102, 103}};
  constexpr int pixels_per_module = 16;
  constexpr int white_margin_modules = 1;

  for (const auto id : ids) {
    image_u8_t * tag = apriltag_to_image(family, id);
    const int output_width =
      (tag->width + 2 * white_margin_modules) * pixels_per_module;
    image_u8_t * output = image_u8_create(output_width, output_width);
    std::fill(output->buf, output->buf + output->height * output->stride, 255U);
    for (int row = 0; row < tag->height; ++row) {
      for (int column = 0; column < tag->width; ++column) {
        const std::uint8_t value = tag->buf[row * tag->stride + column];
        const int output_row = (row + white_margin_modules) * pixels_per_module;
        const int output_column = (column + white_margin_modules) * pixels_per_module;
        for (int y = 0; y < pixels_per_module; ++y) {
          std::fill_n(
            output->buf + (output_row + y) * output->stride + output_column,
            pixels_per_module, value);
        }
      }
    }
    const auto output_path = output_directory / ("tag36h11_" + std::to_string(id) + ".pnm");
    if (image_u8_write_pnm(output, output_path.c_str()) != 0) {
      std::cerr << "failed to write " << output_path << '\n';
      image_u8_destroy(output);
      image_u8_destroy(tag);
      tag36h11_destroy(family);
      return 1;
    }
    const std::string base_name = "tag36h11_" + std::to_string(id);
    std::ofstream mesh(mesh_directory / (base_name + ".dae"));
    mesh << R"(<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="tag-image"><init_from>../materials/textures/)"
         << base_name << R"(.png</init_from></image></library_images>
  <library_effects><effect id="tag-effect"><profile_COMMON>
    <technique sid="standard"><phong>
      <emission><color>0 0 0 1</color></emission>
      <ambient><color>1 1 1 1</color></ambient>
      <diffuse><texture texture="tag-image" texcoord="CHANNEL0"><extra><technique profile="MAYA"><wrapU>FALSE</wrapU><wrapV>FALSE</wrapV><blend_mode>ADD</blend_mode></technique></extra></texture></diffuse>
      <specular><color>0 0 0 1</color></specular><shininess><float>1</float></shininess>
      <transparent opaque="RGB_ZERO"><color>1 1 1 1</color></transparent><transparency><float>0</float></transparency>
    </phong></technique>
  </profile_COMMON><extra><technique profile="GOOGLEEARTH"><double_sided>1</double_sided></technique></extra></effect></library_effects>
  <library_materials><material id="tag-material"><instance_effect url="#tag-effect"/></material></library_materials>
  <library_geometries><geometry id="tag-geometry"><mesh>
    <source id="positions"><float_array id="positions-array" count="12">-0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5 0</float_array><technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="normals"><float_array id="normals-array" count="6">0 0 1 0 0 -1</float_array><technique_common><accessor source="#normals-array" count="2" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="uv"><float_array id="uv-array" count="8">0 0 1 0 1 1 0 1</float_array><technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
    <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
    <triangles count="4" material="tag-material"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/><p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3 2 1 2 1 1 1 0 1 0 3 1 3 2 1 2 0 1 0</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node id="tag"><instance_geometry url="#tag-geometry"><bind_material><technique_common><instance_material symbol="tag-material" target="#tag-material"><bind_vertex_input semantic="CHANNEL0" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
)";
    if (!mesh) {
      std::cerr << "failed to write mesh for tag " << id << '\n';
      image_u8_destroy(output);
      image_u8_destroy(tag);
      tag36h11_destroy(family);
      return 1;
    }
    image_u8_destroy(output);
    image_u8_destroy(tag);
  }
  tag36h11_destroy(family);
  return 0;
}
