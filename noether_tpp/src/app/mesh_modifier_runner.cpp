/**
 * @file mesh_modifier_runner.cpp
 * @brief Applies the mesh modifiers of a tool path planning pipeline configuration to a mesh, one stage at a time,
 * and saves the output of every stage so the effect of each modifier can be inspected (e.g., in MeshLab)
 */
#include <boost/program_options.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/ply_io.h>

#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>

namespace
{
std::size_t countVertices(const pcl::PolygonMesh& mesh)
{
  return static_cast<std::size_t>(mesh.cloud.width) * static_cast<std::size_t>(mesh.cloud.height);
}

std::string stageFileName(const std::string& out_dir,
                          const std::size_t stage,
                          const std::string& name,
                          const std::size_t mesh_idx,
                          const std::size_t n_meshes)
{
  std::stringstream ss;
  ss << out_dir << "/" << std::setw(2) << std::setfill('0') << stage << "_" << name;
  if (n_meshes > 1)
    ss << "_" << mesh_idx;
  ss << ".ply";
  return ss.str();
}

void saveStage(const std::string& out_dir,
               const std::size_t stage,
               const std::string& name,
               const std::vector<pcl::PolygonMesh>& meshes)
{
  for (std::size_t i = 0; i < meshes.size(); ++i)
  {
    const std::string file = stageFileName(out_dir, stage, name, i, meshes.size());
    if (pcl::io::savePLYFileBinary(file, meshes[i]) < 0)
      throw std::runtime_error("Failed to save mesh to '" + file + "'");
  }
}

void printStage(const std::size_t stage,
                const std::string& name,
                const double time_ms,
                const std::vector<pcl::PolygonMesh>& meshes)
{
  std::size_t n_vertices = 0;
  std::size_t n_faces = 0;
  for (const auto& mesh : meshes)
  {
    n_vertices += countVertices(mesh);
    n_faces += mesh.polygons.size();
  }

  // clang-format off
  std::cout << std::setw(2) << std::setfill('0') << stage << std::setfill(' ') << "  "
            << std::left << std::setw(24) << name << std::right
            << std::setw(12) << std::fixed << std::setprecision(1) << time_ms << " ms"
            << std::setw(8) << meshes.size() << " mesh(es)"
            << std::setw(12) << n_vertices << " vertices"
            << std::setw(12) << n_faces << " faces" << std::endl;
  // clang-format on
}

void makeDirectory(const std::string& dir)
{
  struct stat info;
  if (stat(dir.c_str(), &info) == 0)
  {
    if (!(info.st_mode & S_IFDIR))
      throw std::runtime_error("Output path '" + dir + "' exists but is not a directory");
    return;
  }

  if (mkdir(dir.c_str(), 0755) != 0)
    throw std::runtime_error("Failed to create output directory '" + dir + "'");
}

}  // namespace

int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::options_description opts("Mesh modifier runner options");

  // clang-format off
  opts.add_options()
    ("help,h", "Produce help message")
    ("in_file,i", po::value<std::string>()->required(), "Input mesh file")
    ("config,c", po::value<std::string>()->required(), "Tool path planning pipeline YAML configuration file (only the 'mesh_modifiers' section is used)")
    ("out_dir,o", po::value<std::string>()->required(), "Output directory for the mesh of each stage");
  // clang-format on

  try
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, opts), vm);

    if (vm.count("help"))
    {
      std::cout << opts << std::endl;
      return 0;
    }

    po::notify(vm);

    const std::string in_file = vm.at("in_file").as<std::string>();
    const std::string config_file = vm.at("config").as<std::string>();
    const std::string out_dir = vm.at("out_dir").as<std::string>();

    // Load the input mesh
    pcl::PolygonMesh input_mesh;
    if (pcl::io::load(in_file, input_mesh) < 0)
      throw std::runtime_error("Failed to load mesh from '" + in_file + "'");

    // Load the mesh modifier configurations
    const YAML::Node config = YAML::LoadFile(config_file);
    const auto modifiers_config = YAML::getMember<YAML::Node>(config, "mesh_modifiers");
    if (!modifiers_config.IsSequence())
      throw std::runtime_error("'mesh_modifiers' in '" + config_file + "' must be a list");

    // Create all modifiers up front so that configuration errors are reported before any processing
    noether::Factory factory;
    std::vector<std::string> names;
    std::vector<noether::MeshModifier::ConstPtr> modifiers;
    for (const YAML::Node& entry : modifiers_config)
    {
      const auto name = YAML::getMember<std::string>(entry, "name");
      try
      {
        modifiers.push_back(factory.createMeshModifier(entry));
      }
      catch (const std::exception& ex)
      {
        throw std::runtime_error("Failed to create mesh modifier '" + name + "': " + ex.what());
      }
      names.push_back(name);
    }

    makeDirectory(out_dir);

    // Stage 0: the input mesh
    std::vector<pcl::PolygonMesh> meshes{ input_mesh };
    printStage(0, "input", 0.0, meshes);
    saveStage(out_dir, 0, "input", meshes);

    // Apply the modifiers one at a time
    double total_ms = 0.0;
    for (std::size_t i = 0; i < modifiers.size(); ++i)
    {
      const std::size_t stage = i + 1;
      std::vector<pcl::PolygonMesh> output;

      const auto start = std::chrono::steady_clock::now();
      try
      {
        for (const pcl::PolygonMesh& mesh : meshes)
        {
          std::vector<pcl::PolygonMesh> result = modifiers[i]->modify(mesh);
          output.insert(output.end(), std::make_move_iterator(result.begin()), std::make_move_iterator(result.end()));
        }
      }
      catch (const std::exception& ex)
      {
        throw std::runtime_error("Error in stage " + std::to_string(stage) + " (" + names[i] + "): " + ex.what());
      }
      const auto end = std::chrono::steady_clock::now();
      const double time_ms = std::chrono::duration<double, std::milli>(end - start).count();
      total_ms += time_ms;

      printStage(stage, names[i], time_ms, output);

      if (output.empty())
        throw std::runtime_error("Stage " + std::to_string(stage) + " (" + names[i] + ") produced no meshes");

      saveStage(out_dir, stage, names[i], output);
      meshes = std::move(output);
    }

    std::cout << "Total modifier time: " << std::fixed << std::setprecision(1) << total_ms << " ms" << std::endl;
    std::cout << "Saved stage meshes to '" << out_dir << "'" << std::endl;
  }
  catch (const std::exception& ex)
  {
    std::cout << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
