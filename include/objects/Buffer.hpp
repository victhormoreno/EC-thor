/**
 * @file Buffer.hpp
 * @author Víctor Moreno (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Buffer class.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <ros/ros.h>

#include <deque>
#include <stdexcept>

/**
 * @brief A class that represents a buffer, implemented with a deque and able to
 * restrict its size with a maxSize.
 * 
 * @tparam BufferedType
 */
template <typename BufferedType>
class Buffer : private std::deque<BufferedType> {
 private:
  using Parent = std::deque<BufferedType>;

  /* -------------------------- Private Attributes -------------------------- */

  /**
   * @brief Represents the maximum size that the Buffer can have.
   */
  const int maxSize_;

 public:
  /* -------------------------- Public Constructor -------------------------- */

  Buffer(const int &maxSize) : maxSize_(maxSize) {}

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Whether or not the container does not have data.
   * 
   * @return true 
   * @return false 
   */
  bool empty() const { return Parent::empty(); }

  /**
   * @brief Returns the number of objects in the container.
   * 
   * @return size_t 
   */
  size_t size() const { return Parent::size(); }

  /**
   * @brief Returns the newest element.
   * 
   * @return const Elem& 
   */
  const BufferedType &newestElem() const {
    if (empty()) {
      throw std::runtime_error("Buffer is empty, cannot get element");
    }
    return Parent::back();
  }

   /**
   * @brief Emplaces a new element into the Buffer.
   * 
   * @param element 
   */
  void add(const BufferedType &element){
    Parent::emplace_back(element);

    // Readjust
    while (!empty() and size() > maxSize_) {
      Parent::pop_front();
    }
  }

  /**
   * @brief Returns a vector with the Buffer data
   * 
   * @return std::vector<BufferedType>
   */
  std::vector<BufferedType> toVector() const {
    std::vector<BufferedType> res;
    res.reserve(size());
    for (auto it = Parent::cbegin(); it != Parent::cend(); it++) {
      res.push_back(*it);
    }
    return res;
  }
};

#endif  
