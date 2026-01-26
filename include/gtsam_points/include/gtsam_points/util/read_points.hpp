// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstring>
#include <Eigen/Core>

namespace gtsam_points {

static std::vector<float> read_times(const std::string& path) {
  std::ifstream ifs(path, std::ios::binary | std::ios::ate);
  if (!ifs) {
    std::cerr << "error: failed to open " << path << std::endl;
    return std::vector<float>();
  }

  std::streamsize points_bytes = ifs.tellg();
  size_t num_data = points_bytes / sizeof(float);

  ifs.seekg(0, std::ios::beg);
  std::vector<float> times(num_data);
  ifs.read(reinterpret_cast<char*>(times.data()), sizeof(float) * num_data);

  return times;
}

static std::vector<Eigen::Vector3f> read_points(const std::string& path) 
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) 
  {
    std::cerr << "error: failed to open " << path << std::endl;
    return std::vector<Eigen::Vector3f>();
  }

  std::string line;
  size_t num_points = 0;
  bool is_binary = false;
  std::vector<std::string> fields;
  std::vector<size_t> sizes;
  std::vector<size_t> counts;
  std::vector<char> types;

  // Parse PCD header
  while (std::getline(ifs, line)) 
  {
    std::istringstream iss(line);
    std::string key;
    iss >> key;

    if (key == "FIELDS") 
    {
      std::string field;
      while (iss >> field) 
      {
        fields.push_back(field);
      }
    } 
    else if (key == "SIZE") 
    {
      size_t s;
      while (iss >> s) 
      {
        sizes.push_back(s);
      }
    }
    else if (key == "TYPE") 
    {
      char t;
      while (iss >> t) 
      {
        types.push_back(t);
      }
    }
    else if (key == "COUNT") 
    {
      size_t c;
      while (iss >> c) 
      {
        counts.push_back(c);
      }
    }
    else if (key == "POINTS") 
    {
      iss >> num_points;
    } 
    else if (key == "DATA") 
    {
      std::string format;
      iss >> format;
      is_binary = (format == "binary");
      break;
    }
  }

  // Calculate byte offset for each field and total point size
  std::vector<size_t> offsets(fields.size());
  size_t point_size = 0;
  int x_offset = -1, y_offset = -1, z_offset = -1;
  
  for (size_t i = 0; i < fields.size(); i++) 
  {
    offsets[i] = point_size;
    size_t field_size = (i < sizes.size() ? sizes[i] : 4);
    size_t field_count = (i < counts.size() ? counts[i] : 1);
    
    if (fields[i] == "x") x_offset = point_size;
    else if (fields[i] == "y") y_offset = point_size;
    else if (fields[i] == "z") z_offset = point_size;
    
    point_size += field_size * field_count;
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) 
  {
    std::cerr << "error: could not find x, y, z fields in " << path << std::endl;
    return std::vector<Eigen::Vector3f>();
  }

  std::vector<Eigen::Vector3f> points;
  points.reserve(num_points);

  if (is_binary) 
  {
    std::vector<char> buffer(point_size);
    
    for (size_t i = 0; i < num_points; i++) 
    {
      ifs.read(buffer.data(), point_size);
      if (!ifs) break;
      
      float x, y, z;
      std::memcpy(&x, buffer.data() + x_offset, sizeof(float));
      std::memcpy(&y, buffer.data() + y_offset, sizeof(float));
      std::memcpy(&z, buffer.data() + z_offset, sizeof(float));
      points.emplace_back(x, y, z);
    }
  } 
  else 
  {
    // ASCII format
    int x_idx = -1, y_idx = -1, z_idx = -1;
    for (size_t i = 0; i < fields.size(); i++) 
    {
      if (fields[i] == "x") x_idx = i;
      else if (fields[i] == "y") y_idx = i;
      else if (fields[i] == "z") z_idx = i;
    }
    
    while (std::getline(ifs, line) && points.size() < num_points) 
    {
      std::istringstream iss(line);
      std::vector<float> values(fields.size());
      for (size_t i = 0; i < fields.size(); i++) 
      {
        iss >> values[i];
      }
      points.emplace_back(values[x_idx], values[y_idx], values[z_idx]);
    }
  }

  return points;
}

static std::vector<Eigen::Vector4f> read_points4(const std::string& path) {
  std::ifstream ifs(path, std::ios::binary | std::ios::ate);
  if (!ifs) {
    std::cerr << "error: failed to open " << path << std::endl;
    return std::vector<Eigen::Vector4f>();
  }

  std::streamsize points_bytes = ifs.tellg();
  size_t num_points = points_bytes / (sizeof(Eigen::Vector4f));

  ifs.seekg(0, std::ios::beg);
  std::vector<Eigen::Vector4f> points(num_points);
  ifs.read(reinterpret_cast<char*>(points.data()), sizeof(Eigen::Vector4f) * num_points);

  return points;
}

}  // namespace gtsam_points