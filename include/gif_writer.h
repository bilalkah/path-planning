/**
 * @file gif.h
 * @author Bilal Kahraman (kahramannbilal@gmail.com)
 * @brief Class for writing gif files.
 * @version 0.1
 * @date 2023-08-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INCLUDE_GIF_WRITER_H_
#define INCLUDE_GIF_WRITER_H_

#include "include/utils.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

struct GifParams
{

};

class GifWriter
{
public:
  GifWriter(std::string file_name);
  virtual ~GifWriter();

  void SaveToFile();
  void AddFrame(const std::shared_ptr<Map> map);
  void InitGif(const GifParams &params);

private:
  std::string file_name_;
  std::ofstream file_;

  std::vector<Map> frames_;
};

#endif /* INCLUDE_GIF_WRITER_H_ */