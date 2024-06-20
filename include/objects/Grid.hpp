/**
 * @file Grid.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Grid object.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef GRID_HPP
#define GRID_HPP

#include <iostream>
#include <array>

/**
 * @brief The Grid object is .
 * It is responsible for:
 * - Creating a 2d grid with a fixed size
 * - Storing T values in the grid
 */

template <typename T>
class Grid {
    protected:
        
        std::vector<std::vector<T>> grid_;
        
        float cell_size_;
        float x_center_grid_, y_center_grid_;
        float x_grid_size_, y_grid_size_;
        int rows_, cols_;


    public:
    
        /* ---------------------------- Constructor ---------------------------- */

        Grid() = default;

        /**
         * @brief Construct a new Grid object
         * 
         * @param cell_size 
         * @param x_center_grid 
         * @param y_center_grid 
         * @param x_grid_size 
         * @param y_grid_size 
         * @param default_value 
         */
        Grid(float cell_size, float x_center_grid, float y_center_grid, float x_grid_size, float y_grid_size, T default_value) : cell_size_(cell_size), 
            x_center_grid_(x_center_grid), y_center_grid_(y_center_grid), x_grid_size_(x_grid_size), y_grid_size_(y_grid_size) {
            
            rows_ = static_cast<int>(x_grid_size_ / cell_size_);
            cols_ = static_cast<int>(y_grid_size_ / cell_size_);
            
            grid_.resize(rows_);
            for(int i = 0; i < rows_; i++) 
                grid_[i].resize(cols_, default_value);
        }

        bool insideGrid(int u, int v) {
            return u >= 0 && u < rows_ && v >= 0 && v < cols_;
        }

        /* ---------------------------- Getters ---------------------------- */

        /**
         * @brief Get the value of a cell in the grid
         * 
         * @param u 
         * @param v 
         * @return T 
         */
        T& operator()(int u, int v) {
            return grid_[u][v];
        }

        const T& operator()(int u, int v) const {
            return grid_[u][v];
        }


        inline int cell_x(float x) const {
            int u = static_cast<int>((x + (x_grid_size_ + x_center_grid_) / 2.0) / cell_size_);
            if (u < 0 || u >= rows_ ) return -1;
            else return static_cast<int>((x + (x_grid_size_ + x_center_grid_) / 2.0) / cell_size_);
        }

        inline int cell_y(float y) const {
            int v = static_cast<int>((y + (y_grid_size_ + y_center_grid_) / 2.0) / cell_size_);
            if (v < 0 || v >= cols_ ) return -1;
            else  return static_cast<int>((y + (y_grid_size_ + y_center_grid_) / 2.0) / cell_size_);
        }
        
        inline size_t rows() const { return rows_; }

        inline size_t cols() const { return cols_; }

        inline float getCellSize() const { return cell_size_; }

        typename std::vector<T>::iterator begin() {
            return grid_.begin()->begin();
        }

        typename std::vector<T>::iterator end() {
            return grid_.back().end();
        }
};

#endif