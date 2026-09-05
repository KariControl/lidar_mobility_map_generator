#include "lidar_mobility_map_generator/pointcloud_io.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{
namespace
{

std::string lower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

std::vector<std::string> split(const std::string & line)
{
  std::istringstream stream(line);
  std::vector<std::string> tokens;
  std::string token;
  while (stream >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

bool isIntensityName(const std::string & name)
{
  const std::string normalized = lower(name);
  return normalized == "intensity" || normalized == "reflectivity" ||
         normalized == "scalar_intensity" || normalized == "signal";
}

template<typename T>
T readLittleEndian(const std::uint8_t * data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(
    reinterpret_cast<std::uint8_t *>(&value),
    reinterpret_cast<std::uint8_t *>(&value) + sizeof(T));
#endif
  return value;
}

double readPcdScalar(
  const std::uint8_t * data, const char type, const std::size_t size)
{
  if (type == 'F' || type == 'f') {
    if (size == 4U) {return static_cast<double>(readLittleEndian<float>(data));}
    if (size == 8U) {return readLittleEndian<double>(data);}
  }
  if (type == 'I' || type == 'i') {
    if (size == 1U) {return static_cast<double>(readLittleEndian<std::int8_t>(data));}
    if (size == 2U) {return static_cast<double>(readLittleEndian<std::int16_t>(data));}
    if (size == 4U) {return static_cast<double>(readLittleEndian<std::int32_t>(data));}
    if (size == 8U) {return static_cast<double>(readLittleEndian<std::int64_t>(data));}
  }
  if (type == 'U' || type == 'u') {
    if (size == 1U) {return static_cast<double>(readLittleEndian<std::uint8_t>(data));}
    if (size == 2U) {return static_cast<double>(readLittleEndian<std::uint16_t>(data));}
    if (size == 4U) {return static_cast<double>(readLittleEndian<std::uint32_t>(data));}
    if (size == 8U) {return static_cast<double>(readLittleEndian<std::uint64_t>(data));}
  }
  throw std::runtime_error("unsupported PCD scalar type/size");
}

struct PcdField
{
  std::string name;
  std::size_t size{0};
  char type{'F'};
  std::size_t count{1};
  std::size_t byte_offset{0};
  std::size_t token_offset{0};
};

std::vector<PcdField> makePcdFields(
  const std::vector<std::string> & names,
  const std::vector<std::size_t> & sizes,
  const std::vector<char> & types,
  const std::vector<std::size_t> & counts)
{
  if (names.empty() || sizes.size() != names.size() || types.size() != names.size() ||
    counts.size() != names.size())
  {
    throw std::runtime_error("invalid PCD field specification");
  }
  std::vector<PcdField> fields;
  fields.reserve(names.size());
  std::size_t byte_offset = 0U;
  std::size_t token_offset = 0U;
  for (std::size_t index = 0U; index < names.size(); ++index) {
    fields.push_back(
      {names[index], sizes[index], types[index], counts[index], byte_offset, token_offset});
    byte_offset += sizes[index] * counts[index];
    token_offset += counts[index];
  }
  return fields;
}

const PcdField * findField(const std::vector<PcdField> & fields, const std::string & name)
{
  const std::string target = lower(name);
  for (const PcdField & field : fields) {
    if (lower(field.name) == target) {
      return &field;
    }
  }
  return nullptr;
}

const PcdField * findIntensityField(const std::vector<PcdField> & fields)
{
  for (const PcdField & field : fields) {
    if (isIntensityName(field.name)) {
      return &field;
    }
  }
  return nullptr;
}

const PcdField * findObservationCountField(const std::vector<PcdField> & fields)
{
  for (const PcdField & field : fields) {
    const std::string name = lower(field.name);
    if (name == "observation_count" || name == "observations") {
      return &field;
    }
  }
  return nullptr;
}

std::uint32_t observationCount(const double value)
{
  if (!std::isfinite(value) || value < 1.0) {
    return 1U;
  }
  return static_cast<std::uint32_t>(std::min(
      value, static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
}

std::size_t plyTypeSize(const std::string & type)
{
  const std::string normalized = lower(type);
  if (normalized == "char" || normalized == "int8" || normalized == "uchar" ||
    normalized == "uint8")
  {
    return 1U;
  }
  if (normalized == "short" || normalized == "int16" || normalized == "ushort" ||
    normalized == "uint16")
  {
    return 2U;
  }
  if (normalized == "int" || normalized == "int32" || normalized == "uint" ||
    normalized == "uint32" || normalized == "float" || normalized == "float32")
  {
    return 4U;
  }
  if (normalized == "double" || normalized == "float64" || normalized == "int64" ||
    normalized == "uint64")
  {
    return 8U;
  }
  throw std::runtime_error("unsupported PLY scalar type: " + type);
}

double readPlyScalar(const std::uint8_t * data, const std::string & type)
{
  const std::string normalized = lower(type);
  if (normalized == "char" || normalized == "int8") {
    return readLittleEndian<std::int8_t>(data);
  }
  if (normalized == "uchar" || normalized == "uint8") {
    return readLittleEndian<std::uint8_t>(data);
  }
  if (normalized == "short" || normalized == "int16") {
    return readLittleEndian<std::int16_t>(data);
  }
  if (normalized == "ushort" || normalized == "uint16") {
    return readLittleEndian<std::uint16_t>(data);
  }
  if (normalized == "int" || normalized == "int32") {
    return readLittleEndian<std::int32_t>(data);
  }
  if (normalized == "uint" || normalized == "uint32") {
    return readLittleEndian<std::uint32_t>(data);
  }
  if (normalized == "int64") {return static_cast<double>(readLittleEndian<std::int64_t>(data));}
  if (normalized == "uint64") {return static_cast<double>(readLittleEndian<std::uint64_t>(data));}
  if (normalized == "float" || normalized == "float32") {
    return readLittleEndian<float>(data);
  }
  if (normalized == "double" || normalized == "float64") {
    return readLittleEndian<double>(data);
  }
  throw std::runtime_error("unsupported PLY scalar type: " + type);
}

struct PlyProperty
{
  std::string type;
  std::string name;
  std::size_t byte_offset{0};
};

}  // namespace

std::vector<PointXYZI> loadPointCloudFile(const std::filesystem::path & path)
{
  const std::string extension = lower(path.extension().string());
  if (extension == ".pcd") {
    return loadPcdFile(path);
  }
  if (extension == ".ply") {
    return loadPlyFile(path);
  }
  throw std::runtime_error(
          "unsupported point cloud extension '" + extension + "'; use .pcd or .ply");
}

std::vector<PointXYZI> loadPcdFile(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("failed to open PCD file: " + path.string());
  }

  std::vector<std::string> names;
  std::vector<std::size_t> sizes;
  std::vector<char> types;
  std::vector<std::size_t> counts;
  std::size_t point_count = 0U;
  std::string data_mode;
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const std::vector<std::string> tokens = split(line);
    if (tokens.empty() || tokens[0].front() == '#') {
      continue;
    }
    const std::string key = lower(tokens[0]);
    if ((key == "fields" || key == "field") && tokens.size() > 1U) {
      names.assign(tokens.begin() + 1, tokens.end());
    } else if (key == "size" && tokens.size() > 1U) {
      sizes.clear();
      for (std::size_t index = 1U; index < tokens.size(); ++index) {
        sizes.push_back(static_cast<std::size_t>(std::stoull(tokens[index])));
      }
    } else if (key == "type" && tokens.size() > 1U) {
      types.clear();
      for (std::size_t index = 1U; index < tokens.size(); ++index) {
        types.push_back(tokens[index].empty() ? 'F' : tokens[index].front());
      }
    } else if (key == "count" && tokens.size() > 1U) {
      counts.clear();
      for (std::size_t index = 1U; index < tokens.size(); ++index) {
        counts.push_back(static_cast<std::size_t>(std::stoull(tokens[index])));
      }
    } else if (key == "points" && tokens.size() >= 2U) {
      point_count = static_cast<std::size_t>(std::stoull(tokens[1]));
    } else if (key == "width" && tokens.size() >= 2U && point_count == 0U) {
      point_count = static_cast<std::size_t>(std::stoull(tokens[1]));
    } else if (key == "data" && tokens.size() >= 2U) {
      data_mode = lower(tokens[1]);
      break;
    }
  }
  if (counts.empty()) {
    counts.assign(names.size(), 1U);
  }
  const std::vector<PcdField> fields = makePcdFields(names, sizes, types, counts);
  const PcdField * x_field = findField(fields, "x");
  const PcdField * y_field = findField(fields, "y");
  const PcdField * z_field = findField(fields, "z");
  const PcdField * intensity_field = findIntensityField(fields);
  const PcdField * observation_count_field = findObservationCountField(fields);
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    throw std::runtime_error("PCD must contain x, y and z fields");
  }

  std::vector<PointXYZI> points;
  points.reserve(point_count);
  if (data_mode == "ascii") {
    std::size_t read_points = 0U;
    while ((point_count == 0U || read_points < point_count) && std::getline(stream, line)) {
      const std::vector<std::string> tokens = split(line);
      if (tokens.empty()) {
        continue;
      }
      const std::size_t required = std::max({
        x_field->token_offset, y_field->token_offset, z_field->token_offset,
        intensity_field != nullptr ? intensity_field->token_offset : 0U,
        observation_count_field != nullptr ? observation_count_field->token_offset : 0U}) + 1U;
      if (tokens.size() < required) {
        continue;
      }
      PointXYZI point;
      point.x = std::stod(tokens[x_field->token_offset]);
      point.y = std::stod(tokens[y_field->token_offset]);
      point.z = std::stod(tokens[z_field->token_offset]);
      point.intensity = intensity_field != nullptr ?
        std::stod(tokens[intensity_field->token_offset]) : 0.0;
      if (observation_count_field != nullptr) {
        point.observation_count = observationCount(
          std::stod(tokens[observation_count_field->token_offset]));
      }
      if (point.finite()) {
        points.push_back(point);
      }
      ++read_points;
    }
  } else if (data_mode == "binary") {
    std::size_t point_step = 0U;
    for (const PcdField & field : fields) {
      point_step += field.size * field.count;
    }
    if (point_count == 0U || point_step == 0U) {
      throw std::runtime_error("PCD binary header has no points or zero point step");
    }
    std::vector<std::uint8_t> buffer(point_step);
    for (std::size_t index = 0U; index < point_count; ++index) {
      stream.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
      if (!stream) {
        throw std::runtime_error("unexpected end of binary PCD data");
      }
      PointXYZI point;
      point.x = readPcdScalar(buffer.data() + x_field->byte_offset, x_field->type, x_field->size);
      point.y = readPcdScalar(buffer.data() + y_field->byte_offset, y_field->type, y_field->size);
      point.z = readPcdScalar(buffer.data() + z_field->byte_offset, z_field->type, z_field->size);
      point.intensity = intensity_field != nullptr ?
        readPcdScalar(
        buffer.data() + intensity_field->byte_offset,
        intensity_field->type, intensity_field->size) : 0.0;
      if (observation_count_field != nullptr) {
        point.observation_count = observationCount(readPcdScalar(
            buffer.data() + observation_count_field->byte_offset,
            observation_count_field->type, observation_count_field->size));
      }
      if (point.finite()) {
        points.push_back(point);
      }
    }
  } else if (data_mode == "binary_compressed") {
    throw std::runtime_error("binary_compressed PCD is not supported; convert it to binary or ASCII PCD");
  } else {
    throw std::runtime_error("unsupported or missing PCD DATA mode: " + data_mode);
  }
  return points;
}

std::vector<PointXYZI> loadPlyFile(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("failed to open PLY file: " + path.string());
  }
  std::string line;
  if (!std::getline(stream, line) || lower(line) != "ply") {
    throw std::runtime_error("invalid PLY signature");
  }
  std::string format;
  std::size_t vertex_count = 0U;
  bool in_vertex_element = false;
  std::vector<PlyProperty> properties;
  std::size_t point_step = 0U;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const std::vector<std::string> tokens = split(line);
    if (tokens.empty()) {
      continue;
    }
    const std::string key = lower(tokens[0]);
    if (key == "format" && tokens.size() >= 2U) {
      format = lower(tokens[1]);
    } else if (key == "element" && tokens.size() >= 3U) {
      in_vertex_element = lower(tokens[1]) == "vertex";
      if (in_vertex_element) {
        vertex_count = static_cast<std::size_t>(std::stoull(tokens[2]));
      }
    } else if (key == "property" && in_vertex_element) {
      if (tokens.size() >= 2U && lower(tokens[1]) == "list") {
        throw std::runtime_error("list properties in PLY vertex elements are not supported");
      }
      if (tokens.size() >= 3U) {
        PlyProperty property{tokens[1], tokens[2], point_step};
        point_step += plyTypeSize(property.type);
        properties.push_back(property);
      }
    } else if (key == "end_header") {
      break;
    }
  }
  auto find_property = [&properties](const std::string & name) -> const PlyProperty * {
      for (const PlyProperty & property : properties) {
        if (lower(property.name) == lower(name)) {
          return &property;
        }
      }
      return nullptr;
    };
  const PlyProperty * x_property = find_property("x");
  const PlyProperty * y_property = find_property("y");
  const PlyProperty * z_property = find_property("z");
  const PlyProperty * intensity_property = nullptr;
  for (const PlyProperty & property : properties) {
    if (isIntensityName(property.name)) {
      intensity_property = &property;
      break;
    }
  }
  if (x_property == nullptr || y_property == nullptr || z_property == nullptr) {
    throw std::runtime_error("PLY must contain x, y and z vertex properties");
  }

  std::vector<PointXYZI> points;
  points.reserve(vertex_count);
  if (format == "ascii") {
    for (std::size_t index = 0U; index < vertex_count; ++index) {
      if (!std::getline(stream, line)) {
        throw std::runtime_error("unexpected end of ASCII PLY vertex data");
      }
      const std::vector<std::string> tokens = split(line);
      if (tokens.size() < properties.size()) {
        throw std::runtime_error("ASCII PLY vertex has fewer values than properties");
      }
      auto property_index = [&properties](const PlyProperty * property) {
          return static_cast<std::size_t>(property - properties.data());
        };
      PointXYZI point;
      point.x = std::stod(tokens[property_index(x_property)]);
      point.y = std::stod(tokens[property_index(y_property)]);
      point.z = std::stod(tokens[property_index(z_property)]);
      point.intensity = intensity_property != nullptr ?
        std::stod(tokens[property_index(intensity_property)]) : 0.0;
      if (point.finite()) {
        points.push_back(point);
      }
    }
  } else if (format == "binary_little_endian") {
    if (point_step == 0U) {
      throw std::runtime_error("binary PLY point step is zero");
    }
    std::vector<std::uint8_t> buffer(point_step);
    for (std::size_t index = 0U; index < vertex_count; ++index) {
      stream.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
      if (!stream) {
        throw std::runtime_error("unexpected end of binary PLY vertex data");
      }
      PointXYZI point;
      point.x = readPlyScalar(buffer.data() + x_property->byte_offset, x_property->type);
      point.y = readPlyScalar(buffer.data() + y_property->byte_offset, y_property->type);
      point.z = readPlyScalar(buffer.data() + z_property->byte_offset, z_property->type);
      point.intensity = intensity_property != nullptr ?
        readPlyScalar(buffer.data() + intensity_property->byte_offset, intensity_property->type) : 0.0;
      if (point.finite()) {
        points.push_back(point);
      }
    }
  } else if (format == "binary_big_endian") {
    throw std::runtime_error("binary_big_endian PLY is not supported");
  } else {
    throw std::runtime_error("unsupported or missing PLY format: " + format);
  }
  return points;
}

void savePcdBinary(const std::filesystem::path & path, const std::vector<PointXYZI> & points)
{
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  // Autoware staging uses a hard link for large PCDs. Writing the destination
  // inode in place would silently turn an old staged bundle into an
  // old-Lanelet/new-PCD mixture during regeneration. Write a sibling inode and
  // atomically replace the directory entry so existing hard links retain the
  // complete previous map until staging is refreshed.
  const std::filesystem::path temporary = path.string() + ".lmmg.tmp";
  std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("failed to create temporary PCD file: " + temporary.string());
  }
  stream << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity observation_count\n"
         << "SIZE 4 4 4 4 4\n"
         << "TYPE F F F F U\n"
         << "COUNT 1 1 1 1 1\n"
         << "WIDTH " << points.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << points.size() << "\n"
         << "DATA binary\n";
  for (const PointXYZI & point : points) {
    const std::array<float, 4> data{
      static_cast<float>(point.x), static_cast<float>(point.y),
      static_cast<float>(point.z), static_cast<float>(point.intensity)};
    stream.write(reinterpret_cast<const char *>(data.data()), sizeof(data));
    const std::uint32_t observations = std::max<std::uint32_t>(1U, point.observation_count);
    stream.write(reinterpret_cast<const char *>(&observations), sizeof(observations));
  }
  stream.close();
  if (!stream) {
    std::error_code cleanup_error;
    std::filesystem::remove(temporary, cleanup_error);
    throw std::runtime_error("failed to finish PCD file: " + temporary.string());
  }
  std::error_code rename_error;
  std::filesystem::rename(temporary, path, rename_error);
  if (rename_error) {
    std::error_code cleanup_error;
    std::filesystem::remove(temporary, cleanup_error);
    throw std::runtime_error(
            "failed to atomically install PCD file: " + path.string() +
            " (" + rename_error.message() + ")");
  }
}

}  // namespace lidar_mobility_map_generator
