#include "python_api.h"
// AI use: created by Hermes Agent

#include <iostream>
#include <sstream>

// PythonPlotAPI implementation - wraps plot.py functions for C++ interop

PythonPlotAPI::PythonPlotAPI() {
}

PythonPlotAPI::~PythonPlotAPI() {
    cleanup();
}

bool PythonPlotAPI::initialize(const std::string& python_path) {
    if (_initialized) {
        std::cerr << "PythonPlotAPI already initialized" << std::endl;
        return true;
    }

    // Initialize Python interpreter
    try {
        py::initialize_interpreter();

        std::cout << "Python interpreter initialized" << std::endl;
    } catch (const py::error_already_set& e) {
        std::cerr << "Failed to initialize Python: " << e.what() << std::endl;
        return false;
    }

    // Add current directory to Python path so plot.py can be found
    try {
        py::module_ sys_module = py::module_::import("sys");
        sys_module.attr("path").attr("__add__")(python_path);

        std::cout << "Added Python path: " << python_path << std::endl;
    } catch (const py::error_already_set& e) {
        std::cerr << "Failed to add Python path: " << e.what() << std::endl;
        return false;
    }

    // Import the plot module
    try {
        g_python_module = py::module_::import("plot");

        // Verify we can access the functions we need
        auto attr1 = g_python_module.attr("add_data");
        auto attr2 = g_python_module.attr("plot_data");

        std::cout << "Successfully loaded plot module with add_data and plot_data" << std::endl;
    } catch (const py::error_already_set& e) {
        std::cerr << "Error importing plot module: " << e.what() << std::endl;
        // Cleanup on error
        py::finalize_interpreter();
        return false;
    }

    _initialized = true;
    return true;
}

void PythonPlotAPI::cleanup() {
    if (g_python_module.ptr()) {
        g_python_module = py::module(); // Release the module reference
    }

    _initialized = false;
}

bool PythonPlotAPI::add_data(const py::object& euler1, const py::object& euler2,
                             const py::object& euler3) {
    if (!_initialized) {
        std::cerr << "PythonPlotAPI not initialized" << std::endl;
        return false;
    }

    // Get the add_data function from the plot module
    auto add_data_func = g_python_module.attr("add_data");

    try {
        // Call add_data with the three Euler angle tuples/lists
        add_data_func(euler1, euler2, euler3);

        std::cout << "Successfully added data points" << std::endl;
        return true;
    } catch (const py::error_already_set& e) {
        std::cerr << "Error calling add_data: " << e.what() << std::endl;
        // Try to provide more detail if available
        PyErr_Print();
        return false;
    }
}

bool PythonPlotAPI::add_data_from_vectors(const py::list& euler1, const py::list& euler2,
                                          const py::list& euler3) {
    if (!_initialized) {
        std::cerr << "PythonPlotAPI not initialized" << std::endl;
        return false;
    }

    if (euler1.size() != euler2.size() || euler2.size() != euler3.size()) {
        std::cerr << "Error: All input lists must have the same size" << std::endl;
        return false;
    }

    // Get the add_data function
    auto add_data_func = g_python_module.attr("add_data");

    try {
        for (size_t i = 0; i < euler1.size(); ++i) {
            // Extract x, y, z from each tuple/list in the inputs
            const auto& e1 = euler1[i];
            const auto& e2 = euler2[i];
            const auto& e3 = euler3[i];

            add_data_func(e1, e2, e3);
        }

        std::cout << "Successfully added " << euler1.size() << " data points" << std::endl;
        return true;
    } catch (const py::error_already_set& e) {
        std::cerr << "Error in add_data_from_vectors: " << e.what() << std::endl;
        PyErr_Print();
        return false;
    }
}
