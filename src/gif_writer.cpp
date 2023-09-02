/**
 * @file gif.cpp
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "include/gif_writer.h"

GifWriter::GifWriter(std::string file_name) { file_name_ = file_name; }

GifWriter::~GifWriter() {}

void GifWriter::SaveToFile()
{
  std::ofstream file(file_name_);
  if (!file.is_open())
    {
      std::cout << "File could not be opened." << std::endl;
      return;
    }

  // Write header
  file << "GIF89a" << std::endl;

  // Write logical screen descriptor size = 16x16, 256 colors, no sort, 8 bits
  // per pixel
  file << "\x10\x00\x10\x00\x80\x00\x00";

  // Write global color table (256 colors)
  for (int i = 0; i < 256; i++)
    {
      file << (char)i << (char)i << (char)i;
    }
    

  file.close();
}

void GifWriter::AddFrame(const std::shared_ptr<Map> map)
{
  frames_.push_back(*map);
}

void GifWriter::InitGif(const GifParams &params) {}
