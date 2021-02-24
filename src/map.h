/**
 * map.h
 *
 * Created on: Feb 24, 2021
 * Author: Saeed Bonab
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>

class Map {
 public:  
  struct single_landmark_s {
    int id_i ; // Landmark ID
    float x_f; // Landmark x-position in the map (global coordinates)
    float y_f; // Landmark y-position in the map (global coordinates)
  };

  std::vector<single_landmark_s> landmark_list; // List of landmarks in the map
};

#endif  // MAP_H_
