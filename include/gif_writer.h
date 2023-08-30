/**
 * @file gif.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Class for reading and writing gif files.
 * @version 0.1
 * @date 2023-08-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_GIF_WRITER_H_
#define INCLUDE_GIF_WRITER_H_

#include "include/utils.h"

#include <string>

struct GifParams
{
};

class GifWriter
{
public:
  GifWriter(std::string file_name);
  virtual ~GifWriter();

  void WriteToFile();
  void AddFrame();

private:
  std::string file_name_;
};

#endif /* INCLUDE_GIF_WRITER_H_ */