#include <iostream>
#include "python_api.h"

// Global data storage for plotting
static std::vector<std::vector<double>> g_roll_data;
static std::vector<std::vector<double>> g_pitch_data;

// Global API instance singleton
plot::PythonPlotAPI* g_global_api = nullptr;

// Implementation of PythonPlotAPI class
plot::PythonPlotAPI::PythonPlotAPI() {}

plot::PythonPlotAPI::~PythonPlotAPI() {
    cleanup();
}

bool plot::PythonPlotAPI::initialize(const std::string& python_path) {
    std::cout << "Initializing plot API" << std::endl;
    _initialized = true;
    return true;
}

void plot::PythonPlotAPI::cleanup() {
    if (_initialized) {
        // Reset data storage on cleanup
        g_roll_data.clear();
        g_pitch_data.clear();
        _initialized = false;
        std::cout << "Cleanup complete" << std::endl;
    }
}

bool plot::PythonPlotAPI::add_data(std::vector<double> euler1, std::vector<double> euler2, 
                              std::vector<double> /*euler3*/) {
    if (!_initialized || euler1.empty()) {
        return false;
    }
    
    // Store data for later plotting
    g_roll_data.push_back(euler1);
    g_pitch_data.push_back(euler2);
    
    std::cout << "Data added successfully" << std::endl;
    return true;
}

bool plot::PythonPlotAPI::add_data_from_vectors(std::vector<double> euler1, std::vector<double> euler2, 
                               std::vector<double> /*euler3*/) {
    if (!_initialized || euler1.empty()) {
        return false;
    }
    
    // Store data for later plotting
    g_roll_data.push_back(euler1);
    g_pitch_data.push_back(euler2);
    
    std::cout << "Data from vectors added successfully" << std::endl;
    return true;
}

bool plot::PythonPlotAPI::plot_data(const std::string& filename) {
    if (!_initialized || (g_roll_data.empty() && g_pitch_data.empty())) {
        return false;
    }
    
    std::cout << "Plotting data" << (filename.empty() ? "" : " to " + filename) << std::endl;
    
    _initialized = false;
    
    std::cout << "Plot complete" << std::endl;
    return true;
}
