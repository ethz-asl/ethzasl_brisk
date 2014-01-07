/*
 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IMAGE_IO_H_
#define IMAGE_IO_H_
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <brisk/brisk.h>
#include "./sort-comp.h"

int getfilelists(std::vector<std::string>& initialPaths,
                 std::vector<std::string>& imagepaths, bool sortlexical = false);

int getfilelists(std::string& initialPath, std::vector<std::string>& imagepaths,
                 bool sortlexical = false)
{
  std::vector<std::string> paths;
  paths.push_back(initialPath);
  return getfilelists(paths, imagepaths, sortlexical);
}

int getfilelists(std::vector<std::string>& initialPaths,
                 std::vector<std::string>& imagepaths, bool sortlexical)
{
  std::string initialPath;
  DIR *d;
  struct dirent *dir;
  for (size_t diridx = 0; diridx < initialPaths.size(); ++diridx)
  {
    initialPath = initialPaths.at(diridx);
    if (initialPath.find_last_of("/\\") != initialPath.size())
    {
      initialPath = initialPath + "/"; //add trailing slash
    }
    d = opendir(initialPath.c_str());
    if (d == NULL)
    {
      throw std::logic_error(initialPath + " results in d == NULL");
      return 1;
    }
    int i = 0;
    while ((dir = readdir(d)))
    {
      if (strcmp(dir->d_name, ".") == 0 || strcmp(dir->d_name, "..") == 0)
      {
        continue;
      }
      if(boost::filesystem::is_directory(dir->d_name)){
        continue;
      }
      if (dir == NULL)
        break;

      i++;
      imagepaths.push_back(initialPath + dir->d_name);
    }
  }
  if (sortlexical)
  {
    std::sort(imagepaths.begin(), imagepaths.end()); //normal lexical sort
  }
  else
  {
    std::sort(imagepaths.begin(), imagepaths.end(),
              boost::bind(&numeric_string_compare, _2, _1)); // sorts strictly by the number in the file name
  }
  return 0;
}

#endif /* IMAGE_IO_H_ */
