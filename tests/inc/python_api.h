#ifndef PYTHON_API_H
#define PYTHON_API_H

#include <pybind11/embed.h>
#include <string>
#include <vector>
namespace py = pybind11;

namespace plot {

// Plotting API for Python integration
class PythonPlotAPI {
public:
    PythonPlotAPI();
    ~PythonPlotAPI();

    // Initialize the Python interpreter and load the plot module
    bool initialize(const std::string& python_path = "");

    // Clean up Python interpreter resources
    void cleanup();

    // Add Euler angle data to storage lists
    bool add_data(std::vector<double> euler1, std::vector<double> euler2, 
                  std::vector<double> euler3);

    // Generate and display the plot
    bool plot_data(const std::string& filename = "");

    // Add data from vectors (for use with C++ vectors)
    bool add_data_from_vectors(std::vector<double> euler1, std::vector<double> euler2, 
                               std::vector<double> euler3);

private:
    py::gil_scoped_acquire _g_acquire;
    bool _initialized = false;
    py::object g_python_module;
};

// Convenience function to get global API instance
inline PythonPlotAPI& get_python_plot_api() {
    static PythonPlotAPI api;
    return api;
}

} // namespace plot

#endif // PYTHON_API_H
