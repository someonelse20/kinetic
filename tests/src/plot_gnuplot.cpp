// plot_gnuplot.cpp - Professional gnuplot-based plotting using external gnuplot binary
// Usage: compile with CMake, call to generate plots via external gnuplot tool

#include "plot.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

namespace plot {

// ============ Gnuplot-based CSV generation ============

bool gnuplot::save_csv(const std::string& filename) {
    if (g_points.empty()) {
        std::cerr << "gnuplot::save_csv: No data points stored" << std::endl;
        return false;
    }
    
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Error: Cannot create " << filename << std::endl;
        return false;
    }
    
    // gnuplot CSV format with timestamps
    size_t max_time = 0;
    for (const auto& pt : g_points) {
        if (pt.time > max_time) max_time = pt.time;
    }
    
    out << std::fixed << std::setprecision(4);
    out << "# GNUPLOT CSV FORMAT\n";
    out << "# Column 1: Time, Columns 2+: Data values\n";
    out << std::endl;
    
    for (size_t i = 0; i < g_points.size(); ++i) {
        const auto& pt = g_points[i];
        out << pt.time << " ";
        out << pt.x << " ";
        out << pt.y << " ";
        out << pt.z;
        // Add placeholder columns for additional data series
        for (size_t j = 0; j < pt.values.size(); ++j) {
            out << " " << pt.values[j];
        }
        out << std::endl;
    }
    
    size_t num_points = g_points.empty() ? 0 : g_points.size();
    std::cout << "CSV data saved to " << filename << " (" 
              << num_points << " data points)" << std::endl;
    
    out.close();
    return true;
}

std::string gnuplot::call_gnuplot(const std::string& script, const std::string& input_file) {
    if (input_file.empty()) {
        return ""; // No stdin for now
    }
    
    std::string cmd = "gnuplot";
#ifdef _WIN32
    cmd = "gnuplot.exe";
#endif
    
    pid_t pid = fork();
    if (pid == 0) {
        // Child process - run gnuplot with script and input file
        execlp("gnuplot", "gnuplot", "-e", script.c_str(), NULL);
        exit(1); // Failed to execute
    } else if (pid > 0) {
        // Parent process - wait for child
        int status;
        waitpid(pid, &status, WNOHANG);
        
        std::string output = "";
        if (WIFEXITED(status)) {
            // Child completed - could read stderr if we captured it
            // For now, just acknowledge completion
        } else if (WIFSIGNALED(status)) {
            int sig = WTERMSIG(status);
            std::cerr << "Gnuplot terminated by signal " << sig << std::endl;
        }
    } else {
        std::cerr << "Error: Failed to fork gnuplot process" << std::endl;
    }
    
    return output;
}

bool gnuplot::generate_svg_script(const std::string& output_file) {
    if (g_points.empty()) {
        std::cerr << "gnuplot::generate_svg_script: No data to plot" << std::endl;
        return false;
    }
    
    // Generate SVG directly for the plot
    double time_min = g_points.front().time;
    double time_max = g_points.back().time;
    double range_time = (time_max - time_min > 1e-10) ? (time_max - time_min) : 1.0;
    
    int num_points = static_cast<int>(g_points.size());
    
    std::ostringstream oss;
    
    // SVG document
    oss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    oss << "<svg xmlns=\"http://www.w3.org/2000/svg\"\n";
    oss << "     width=\"800\" height=\"600\"\n";
    oss << "     viewBox=\"0 0 800 600\">\n\n";
    
    // White background
    oss << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n\n";
    
    // Title
    oss << "<text x=\"400\" y=\"35\" text-anchor=\"middle\" font-size=\"20\" font-weight=\"bold\">\n";
    oss << "  Kinetic IMU Sensor Fusion - Euler Angles\n";
    oss << "</text>\n\n";
    
    // Plot area parameters
    int axes_left = 60, axes_right = 740;
    int plot_height = 520;
    
    oss << "<g transform=\"translate(0, 55)\">\n";
    oss << "<text x=\"400\" y=\"20\" text-anchor=\"middle\" font-size=\"16\">X-Axis (Roll/Pitch/Yaw)</text>\n\n";
    
    // Plot background
    oss << "<rect x=\"" << axes_left << "\" y=\"30\"\n";
    oss << "      width=\"" << (axes_right - axes_left) << "\"\n";
    oss << "      height=\"" << plot_height << "\" fill=\"#f9f9f9\" stroke=\"#ccc\"/>\n\n";
    
    // First series: Roll angle (X-axis data)
    if (!g_points.empty()) {
        oss << "<polyline points=\"";
        for (size_t i = 0; i < num_points - 1 && i < g_points.size(); ++i) {
            const auto& pt = g_points[i];
            // X-axis mapping: use time vs roll angle
            double x_svg = axes_left + (pt.time - time_min) / range_time * 
                           (axes_right - axes_left);
            // Clamp and scale the angle value (-90 to 90 -> SVG coords)
            double val = std::abs(pt.x);
            double y_svg = plot_height + 30 - ((val + 45) / 90.0) * plot_height;
            
            if (i == 0) oss << " " << x_svg << "," << y_svg;
            else oss << ", " << x_svg << "," << y_svg;
        }
        oss << "\" fill=\"none\" stroke=\"#1f77b4\" stroke-width=\"2\"/>\n";
        
        // Add legend for this series
        oss << "<rect x=\"" << axes_left - 25 << "\" y=\"" 
            << plot_height + 30 - 10 << "\" width=\"12\" height=\"2\" fill=\"#1f77b4\"/>\n";
        oss << "<text x=\"" << axes_left - 35 << "\" y=\"" 
            << plot_height + 47 << "\" font-size=\"12\">Roll (True)</text>\n";
    }
    
    oss << "</g>\n\n";
    
    // X-axis labels and ticks
    oss << "<line x1=\"" << axes_left << "\" y1=\"" 
        << (plot_height + 30) << "\" x2=\"" << axes_right 
        << "\" y2=\"" << (plot_height + 30) << "\" stroke=\"#333\"/>\n";
    oss << "<text x=\"" << axes_left << "\" y=\"" 
        << (plot_height + 65) << "\" text-anchor=\"end\" font-size=\"12\">t_min</text>\n";
    
    oss << "<line x1=\"" << axes_right << "\" y1=\"" 
        << (plot_height + 30) << "\" x2=\"" << axes_right 
        << "\" y2=\"" << (plot_height + 65) << "\" stroke=\"#333\"/>\n";
    oss << "<text x=\"" << axes_right << "\" y=\"" 
        << (plot_height + 95) << "\" text-anchor=\"end\" font-size=\"12\">t_max</text>\n\n";
    
    // Y-axis labels and ticks
    oss << "<line x1=\"" << axes_left << "\" y1=\"" 
        << (plot_height + 30) << "\" x2=\"" << (axes_left - 8) 
        << "\" y2=\"" << (plot_height + 30) << "\" stroke=\"#333\"/>\n";
    oss << "<text x=\"" << (axes_left - 10) << "\" y=\"" 
        << (plot_height + 65) << "\" text-anchor=\"end\" font-size=\"12\">-45 deg</text>\n";
    oss << "<line x1=\"" << axes_left << "\" y1=\"" 
        << (plot_height + 30) << "\" x2=\"" << (axes_left - 18) 
        << "\" y2=\"" << (plot_height + 30) << "\" stroke=\"#333\"/>\n";
    oss << "<text x=\"" << (axes_left - 10) << "\" y=\"" 
        << (plot_height + 45) << "\" text-anchor=\"end\" font-size=\"12\">0 deg</text>\n";
    oss << "<line x1=\"" << axes_left << "\" y1=\"" 
        << (plot_height + 30) << "\" x2=\"" << (axes_left - 8) 
        << "\" y2=\"" << (plot_height + 30) << "\" stroke=\"#333\"/>\n";
    oss << "<text x=\"" << (axes_left - 10) << "\" y=\"" 
        << (plot_height + 25) << "\" text-anchor=\"end\" font-size=\"12\">+45 deg</text>\n\n";
    
    // Legend group
    oss << "<g transform=\"translate(580, 40)\">\n";
    oss << "<rect width=\"160\" height=\"70\" fill=\"#f5f5f5\" stroke=\"#ccc\" stroke-width=\"1\"/>\n";
    oss << "<text x=\"5\" y=\"-15\" font-size=\"12\" font-weight=\"bold\">Legend</text>\n\n";
    
    // Add legend entries for different data series if they exist
    oss << "<rect x=\"0\" y=\"5\" width=\"12\" height=\"2\" fill=\"#1f77b4\"/>\n";
    oss << "<text x=\"25\" y=\"11\" font-size=\"11\">Roll Angle</text>\n\n";
    
    // Add pitch and yaw placeholder legend entries
    if (g_points.size() > 0) {
        std::vector<std::string> colors = {"#1f77b4", "#2ca02c", "#d62728"};
        for (size_t i = 1; i <= 3 && i < g_points.size(); ++i) {
            double angle_val = std::abs(g_points[0].x); // Use roll as proxy
            oss << "<rect x=\"0\" y=\"" << (5 + i * 20) << "\" width=\"12\" height=\"2\" fill=\"" 
                << (i <= colors.size() ? colors[i-1] : "#666666") << "\"/>\n";
            std::string label = (i == 1) ? "Pitch Angle" : ((i == 2) ? "Yaw Angle" : "Additional Data");
            oss << "<text x=\"25\" y=\"" << (15 + i * 20) << "\" font-size=\"11\">" << label << "</text>\n";
        }
    }
    
    oss << "</g>\n\n";
    
    oss << "</svg>\n";
    
    std::ofstream out(output_file);
    if (out.is_open()) {
        out << oss.str();
        out.close();
        
        std::cout << "SVG plot saved to " << output_file << "\n";
        return true;
    }
    
    return false;
}

} // namespace gnuplot
