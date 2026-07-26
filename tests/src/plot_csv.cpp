// plot_csv.cpp - Pure C++ graphing via CSV + SVG generation (NO Python required!)
// Usage: compile with cmake, call to generate plots

#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>

namespace plot {

// Global data storage (mirrors your Python plot.py structure)
static std::vector<double> g_time;
static std::vector<double> g_data1_x, g_data1_y, g_data1_z;  // True angle XYZ
static std::vector<double> g_data2_x, g_data2_y, g_data2_z;  // Sensor noise XYZ
static std::vector<double> g_data3_x, g_data3_y, g_data3_z;  // Kalman filtered XYZ

const double TIME_INTERVAL = 1.0;

// Add a single data point for one axis (X, Y, or Z)
bool add_point(double t_val, 
               double x1, double y1, double z1,       // True angles
               double x2, double y2, double z2,       // Sensor noise
               double x3, double y3, double z3) {      // Kalman filtered
    g_time.push_back(t_val);
    g_data1_x.push_back(x1); g_data1_y.push_back(y1); g_data1_z.push_back(z1);
    g_data2_x.push_back(x2); g_data2_y.push_back(y2); g_data2_z.push_back(z2);
    g_data3_x.push_back(x3); g_data3_y.push_back(y3); g_data3_z.push_back(z3);
    
    return true;
}

// Add multiple points at once (convenience wrapper)
void add_points(const std::vector<double>& t1, 
                const std::vector<std::tuple<double,double,double>>& data1,
                const std::vector<std::tuple<double,double,double>>& data2,
                const std::vector<std::tuple<double,double,double>>& data3) {
    g_time.reserve(t1.size());
    for(size_t i = 0; i < t1.size(); ++i) {
        add_point(t1[i], 
            std::get<0>(data1[i]), std::get<1>(data1[i]), std::get<2>(data1[i]),
            std::get<0>(data2[i]), std::get<1>(data2[i]), std::get<2>(data2[i]),
            std::get<0>(data3[i]), std::get<1>(data3[i]), std::get<2>(data3[i]));
    }
}

// ============ CSV EXPORT FUNCTION ============

// Save data to CSV format (gnuplot/Matlab readable)
bool save_csv(const std::string& filename, const std::string& plot_type = "all") {
    std::ofstream out(filename);
    if(!out.is_open()) {
        std::cerr << "Error: Cannot open " << filename << std::endl;
        return false;
    }

    out << std::fixed << std::setprecision(4);

    // CSV header
    out << "# Time, data1 (True), data2 (Sensor), data3 (Kalman)\n";

    if(plot_type == "all" || plot_type == "") {
        for(size_t i = 0; i < g_time.size(); ++i) {
            // X-axis
            out << g_time[i] << ", ";
            out << g_data1_x[i] << ", ";
            out << g_data2_x[i] << ", ";
            out << g_data3_x[i] << "\n";

            // Y-axis
            out << g_time[i] << ", ";
            out << g_data1_y[i] << ", ";
            out << g_data2_y[i] << ", ";
            out << g_data3_y[i] << "\n";

            // Z-axis
            out << g_time[i] << ", ";
            out << g_data1_z[i] << ", ";
            out << g_data2_z[i] << ", ";
            out << g_data3_z[i] << "\n";
        }
    } else if(plot_type == "x") {
        for(size_t i = 0; i < g_time.size(); ++i) {
            out << g_time[i] << "," 
                << g_data1_x[i] << "," 
                << g_data2_x[i] << "," 
                << g_data3_x[i] << "\n";
        }
    } else if(plot_type == "y") {
        for(size_t i = 0; i < g_time.size(); ++i) {
            out << g_time[i] << "," 
                << g_data1_y[i] << "," 
                << g_data2_y[i] << "," 
                << g_data3_y[i] << "\n";
        }
    } else if(plot_type == "z") {
        for(size_t i = 0; i < g_time.size(); ++i) {
            out << g_time[i] << "," 
                << g_data1_z[i] << "," 
                << g_data2_z[i] << "," 
                << g_data3_z[i] << "\n";
        }
    }

    std::cout << "CSV data saved to " << filename << " (" 
              << g_time.size() << " data points)\n";
    return true;
}

// ============ SVG PLOT GENERATION (PURE C++) ============

bool plot_svg(const std::string& filename, int width = 800, int height = 600) {
    if(g_time.empty()) {
        std::cerr << "Error: No data to plot. Call add_point() or add_points() first.\n";
        return false;
    }

    int num_points = g_time.size();
    double time_min = *std::min_element(g_time.begin(), g_time.end());
    double time_max = *std::max_element(g_time.begin(), g_time.end());
    double range_time = (time_max - time_min > 1e-10) ? (time_max - time_min) : 1.0;

    // Create SVG file
    std::ofstream svg(filename);
    if(!svg.is_open()) {
        std::cerr << "Error: Cannot create " << filename << std::endl;
        return false;
    }

    // SVG document start
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\"\n";
    svg << "     width=\"" << width << "\" height=\"" << height << "\"\n";
    svg << "     viewBox=\"0 0 " << width << " " << height << "\">\n";

    // White background
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";

    // Main title
    svg << "<text x=\"" << (width/2) << "\" y=\"35\"\n";
    svg << "      text-anchor=\"middle\" font-size=\"20\" font-weight=\"bold\">\n";
    svg << "    Kinetic IMU Sensor Fusion - Euler Angles\n";
    svg << "</text>\n";

    // Plot parameters
    int axes_left = 60, axes_right = width - 40;
    int plot_height = height - 80;
    
    // Colors: Blue=True, Green=Sensor, Red=Kalman
    const std::string COLORS[3] = {"#1f77b4", "#2ca02c", "#d62728"};

    // Generate X-axis subplot
    svg << "<g transform=\"translate(0," << (height - 50) << "\")\">\n";
    
    // Subtitle
    svg << "<text x=\"" << (axes_left + axes_right)/2 
        << "\" y=\"20\" text-anchor=\"middle\" font-size=\"16\">"X-Axis</text>\n";

    // Draw plot area background
    svg << "<rect x=\"" << axes_left << "\" y=\"30\"\n";
    svg << "      width=\"" << (axes_right - axes_left) << "\"\n";
    svg << "      height=\"" << plot_height << "\" fill=\"#f9f9f9\" stroke=\"#ccc\"/>\n";

    // X-axis data lines (True/Sensor/Kalman)
    svg << "<polyline points=\"";
    for(size_t i = 0; i < num_points - 1; ++i) {
        double x_svg = axes_left + (g_time[i] - time_min) / range_time * 
                       (axes_right - axes_left);
        double y_svg = height - plot_height - 30 - 
                       ((std::abs(g_data1_x[i]) > std::abs(g_data2_x[i])) ? g_data1_x[i] : g_data2_x[i]) / 
                       (std::max(std::abs(g_data1_x[0]), std::max(std::abs(g_data2_x[0]), 1.0)) + 1e-10) * 
                       plot_height;
        if(i == 0) svg << " " << x_svg << "," << y_svg;
        else svg << ", " << x_svg << "," << y_svg;
    }
    svg << "</polyline>\n";

    svg << "<text x=\"" << axes_left - 15 << "\" y=\"" 
        << height - plot_height + 20 << "\" font-size=\"14\" fill=\"#1f77b4\">True</text>\n";

    svg << "</g>\n";

    // Generate Y-axis subplot
    svg << "<g transform=\"translate(0," << (height - 50 - plot_height/2) << "\")\">\n";
    svg << "<text x=\"" << (axes_left + axes_right)/2 
        << "\" y=\"20\" text-anchor=\"middle\" font-size=\"16\">Y-Axis</text>\n";

    // Y-axis data lines
    svg << "<polyline points=\"";
    for(size_t i = 0; i < num_points - 1; ++i) {
        double x_svg = axes_left + (g_time[i] - time_min) / range_time * 
                       (axes_right - axes_left);
        double y_svg = height - plot_height - 30 - 
                       ((std::abs(g_data1_y[i]) > std::abs(g_data2_y[i])) ? g_data1_y[i] : g_data2_y[i]) / 
                       (std::max(std::abs(g_data1_y[0]), std::max(std::abs(g_data2_y[0]), 1.0)) + 1e-10) * 
                       plot_height;
        if(i == 0) svg << " " << x_svg << "," << y_svg;
        else svg << ", " << x_svg << "," << y_svg;
    }
    svg << "</polyline>\n";

    svg << "<text x=\"" << axes_left - 15 << "\" y=\"" 
        << height - plot_height + 20 << "\" font-size=\"14\" fill=\"#1f77b4\">True</text>\n";

    svg << "</g>\n";

    // Generate Z-axis subplot
    svg << "<g transform=\"translate(0," << (height - 50 - plot_height) << "\")\">\n";
    svg << "<text x=\"" << (axes_left + axes_right)/2 
        << "\" y=\"20\" text-anchor=\"middle\" font-size=\"16\">Z-Axis</text>\n";

    // Z-axis data lines
    svg << "<polyline points=\"";
    for(size_t i = 0; i < num_points - 1; ++i) {
        double x_svg = axes_left + (g_time[i] - time_min) / range_time * 
                       (axes_right - axes_left);
        double y_svg = height - plot_height - 30 - 
                       ((std::abs(g_data1_z[i]) > std::abs(g_data2_z[i])) ? g_data1_z[i] : g_data2_z[i]) / 
                       (std::max(std::abs(g_data1_z[0]), std::max(std::abs(g_data2_z[0]), 1.0)) + 1e-10) * 
                       plot_height;
        if(i == 0) svg << " " << x_svg << "," << y_svg;
        else svg << ", " << x_svg << "," << y_svg;
    }
    svg << "</polyline>\n";

    svg << "<text x=\"" << axes_left - 15 << "\" y=\"" 
        << height - plot_height + 20 << "\" font-size=\"14\" fill=\"#1f77b4\">True</text>\n";

    svg << "</g>\n";

    // Legend
    svg << "<g transform=\"translate(" << (axes_left + axes_right)/2 - 60 << ", \" 
        << 80 << "\">\n";
    
    svg << "<rect x=\"0\" y=\"-15\" width=\"12\" height=\"2\" fill=\"#1f77b4\"/>\n";
    svg << "<text x=\"20\" y=\"-9\" font-size=\"14\">True Angle</text>\n";

    svg << "</g>\n";

    // X-axis tick marks and labels (simplified - just show start/end)
    svg << "<line x1=\"" << axes_left << "\" y1=\"" 
        << (height - plot_height + 20) << "\" x2=\"" << axes_right 
        << "\" y2=\"" << (height - plot_height + 20) << "\" stroke=\"#333\"/>\n";
    svg << "<text x=\"" << axes_left << "\" y=\"" << (height - plot_height + 40) 
        << "\" text-anchor=\"end\" font-size=\"12\">t = t_min</text>\n";

    svg << "<line x1=\"" << axes_right << "\" y1=\"" 
        << (height - plot_height + 20) << "\" x2=\"" << axes_right 
        << "\" y2=\"" << (height - plot_height + 45) << "\" stroke=\"#333\"/>\n";
    svg << "<text x=\"" << axes_right << "\" y=\"" << (height - plot_height + 65) 
        << "\" text-anchor=\"end\" font-size=\"12\">t = t_max</text>\n";

    // Y-axis tick marks (simplified)
    svg << "<line x1=\"" << axes_left << "\" y1=\"" 
        << (height - plot_height + 20) << "\" x2=\"" << (axes_left - 8) 
        << "\" y2=\"" << (height - plot_height + 20) << "\" stroke=\"#333\"/>\n";
    svg << "<text x=\"" << (axes_left - 10) << "\" y=\"" << (height - plot_height + 45) 
        << "\" text-anchor=\"end\" font-size=\"12\">0 deg</text>\n";

    svg << "</g>\n"; // SVG close
    svg.close();

    std::cout << "SVG plot saved to " << filename << " (" << width << "x" << height << ")\n";
    return true;
}

// ============ TXT ASCII PLOT (for terminal viewing) ============

bool plot_txt(const std::string& filename, const std::string& axis = "all") {
    if(g_time.empty()) {
        std::cerr << "Error: No data to plot.\n";
        return false;
    }

    int rows = 20;  // ASCII plot height
    int cols = 60;  // ASCII plot width

    std::ofstream out(filename);
    if(!out.is_open()) {
        std::cerr << "Error: Cannot create " << filename << "\n";
        return false;
    }

    double time_min = *std::min_element(g_time.begin(), g_time.end());
    double time_max = *std::max_element(g_time.begin(), g_time.end());
    double range_time = (time_max - time_min > 1e-10) ? (time_max - time_min) : 1.0;

    if(axis == "all" || axis == "") {
        out << "# X-Axis Plot\n";
        ascii_plot(rows, cols, g_data1_x, g_time, time_min, range_time, out);

        out << "# Y-Axis Plot\n";
        ascii_plot(rows, cols, g_data1_y, g_time, time_min, range_time, out);

        out << "# Z-Axis Plot\n";
        ascii_plot(rows, cols, g_data1_z, g_time, time_min, range_time, out);
    } else if(axis == "x") {
        ascii_plot(rows, cols, g_data1_x, g_time, time_min, range_time, out);
    } else if(axis == "y") {
        ascii_plot(rows, cols, g_data1_y, g_time, time_min, range_time, out);
    } else if(axis == "z") {
        ascii_plot(rows, cols, g_data1_z, g_time, time_min, range_time, out);
    }

    out.close();
    std::cout << "ASCII plot saved to " << filename << "\n";
    return true;
}

// Helper: generate ASCII plot
int ascii_plot(int rows, int cols, const std::vector<double>& data, 
               const std::vector<double>& time, double time_min, 
               double range_time, std::ostream& out) {
    
    int num_points = data.size();
    double data_min = *std::min_element(data.begin(), data.end());
    double data_max = *std::max_element(data.begin(), data.end());
    double range_data = (data_max - data_min > 1e-10) ? (data_max - data_min) : 1.0;

    out << "\n";

    // Print plot header
    out << "# X-Axis Euler Angle Data\n";
    out << "# Time: " << std::fixed << std::setprecision(2) 
        << time_min << " to " << time_max << " seconds\n";
    out << "# Range: " << std::fixed << std::setprecision(1) 
        << data_min << " to " << data_max << " degrees\n\n";

    // ASCII art coordinates
    for(int r = rows - 1; r >= 0; --r) {
        // Y-axis value at this row
        double y_val = data_min + (r / (rows - 1)) * range_data;
        out << "[" << std::fixed << std::setprecision(1) << y_val << " |   ";

        // X-axis labels at first and last position
        if(r == rows - 1) {
            out << " t_min";
        } else if(r == 0) {
            out << " t_max";
        }

        for(int c = 0; c < cols - 2; ++c) {
            // Calculate time position for this column
            double x_pos = time_min + (c / (cols - 2)) * range_time;
            
            // Find data point at this time or interpolate
            int idx = static_cast<int>((x_pos - time_min) / range_time * num_points);
            if(idx >= num_points) idx = num_points - 1;

            // Calculate Y position for this data value
            double y_pos = (data[idx] - data_min) / range_data;
            
            // Check if this point aligns with current row
            if(y_pos == r / (rows - 1)) {
                out << "*";
            } else if(y_pos > r / (rows - 1)) {
                out << ".";
            } else {
                out << " ";
            }
        }
        out << "]   " << std::flush;
    }

    // X-axis label
    for(int c = 0; c < cols - 2; ++c) {
        out << "-";
    }
    out << "\n       Time\n";

    return num_points;
}

// ============ PRINT TO TERMINAL (for quick viewing) ============

void print_ascii_plot(const std::string& title, 
                      const std::vector<double>& data, 
                      int rows = 15, int cols = 70) {
    
    if(data.empty()) {
        std::cerr << "No data to display\n";
        return;
    }

    double data_min = *std::min_element(data.begin(), data.end());
    double data_max = *std::max_element(data.begin(), data.end());
    double range_data = (data_max - data_min > 1e-10) ? (data_max - data_min) : 1.0;

    std::cout << "\n=== " << title << " ===\n";
    std::cout << "Range: " << std::fixed << std::setprecision(1) 
              << data_min << " to " << data_max << " degrees\n\n";

    for(int r = rows - 1; r >= 0; --r) {
        out "[" << std::fixed << std::setprecision(2) 
            << (data_min + (r / (rows - 1)) * range_data) << "]   ";

        for(int c = 0; c < cols - 3; ++c) {
            double x_pos = data_min + (c / (cols - 3)) * range_data;

            for(size_t i = 0; i < data.size(); ++i) {
                if(std::abs(data[i] - x_pos) < 1e-6) {
                    out << "*";
                    break;
                } else if(data[i] > x_pos && std::abs(data[i] - x_pos) < range_data / 10) {
                    out << ".";
                    break;
                }
            }
        }

        out << "\n\n      Values\n";
    }

    return;
}

// ============ COMMAND LINE UTILITY ============

bool plot_from_csv(const std::string& csv_file, const std::string& output_format = "svg") {
    if(output_format == "csv" || output_format == "") {
        // Already CSV, just acknowledge
        std::cout << "CSV file: " << csv_file << "\n";
        
    } else if(output_format == "svg") {
        std::cout << "Converting CSV to SVG...\n";
        
        // For now, generate a sample plot. In full implementation, 
        // you'd parse the CSV and populate g_data1_* arrays first.
        std::cout << "(SVG generation requires C++ code to be present)\n";
    }

    return true;
}

// ============ UTILITY FUNCTIONS ============

double get_min_max(const std::vector<double>& data, double& min_val, double& max_val) {
    if(data.empty()) return 0.0;
    min_val = *std::min_element(data.begin(), data.end());
    max_val = *std::max_element(data.begin(), data.end());
    return (max_val - min_val);
}

int get_data_size() {
    return static_cast<int>(g_time.size());
}

void clear_data() {
    g_time.clear();
    g_data1_x.clear(); g_data1_y.clear(); g_data1_z.clear();
    g_data2_x.clear(); g_data2_y.clear(); g_data2_z.clear();
    g_data3_x.clear(); g_data3_y.clear(); g_data3_z.clear();
}

// ============ RUN EXAMPLE FROM START ============

void run_plot_example() {
    using namespace std;

    clear_data();

    // Generate 100 data points like plot.py does
    int n_points = 100;
    double current_time = 0.0;

    auto r_uniform = []() { 
        static mt19937_64 rng(chrono::steady_clock::now().time_since_epoch().count());
        return uniform_real_distribution<double>(-45, 45); 
    }();

    for(int i = 0; i < n_points; ++i) {
        double t_now = current_time;
        
        // True angle (clean signal with drift)
        auto true_angle = [&]() -> tuple<double,double,double> {
            static int drift_counter = 0;
            return r_uniform() * 1.0 + r_uniform() * 0.5, 
                   r_uniform() * 0.8 + r_uniform() * 0.4, 
                   r_uniform() * 0.6 + r_uniform() * 0.3;
        }();

        // Sensor noise (high frequency)
        auto sensor_noise = [&]() -> tuple<double,double,double> {
            return (r_uniform() - 22.5) / 18.0 * 15,
                   (r_uniform() - 22.5) / 18.0 * 12,
                   (r_uniform() - 22.5) / 18.0 * 10;
        }();

        // Kalman filter approximation (smoothed)
        auto kalman = [&]() -> tuple<double,double,double> {
            return (true_angle.get<0>() + sensor_noise.get<0>()) / 2,
                   (true_angle.get<1>() + sensor_noise.get<1>()) / 2,
                   (true_angle.get<2>() + sensor_noise.get<2>()) / 2;
        }();

        add_point(t_now, 
            true_angle.get<0>(), true_angle.get<1>(), true_angle.get<2>,
            sensor_noise.get<0>(), sensor_noise.get<1>(), sensor_noise.get<2>,
            kalman.get<0>(), kalman.get<1>(), kalman.get<2>);

        current_time += TIME_INTERVAL;
    }

    // Output results
    cout << "Data generated: " << g_time.size() << " points\n";
    
    // Save CSV files (for gnuplot or other tools)
    save_csv("kinetic_euler_x.csv", "x");
    save_csv("kinetic_euler_y.csv", "y");
    save_csv("kinetic_euler_z.csv", "z");

    // Generate SVG visualization
    plot_svg("kinetic_euler_angles.svg", 1000, 750);

    // Print ASCII preview to terminal
    print_ascii_plot("X-Axis", g_data1_x);
    
    cout << "\n=== Output Files ===\n";
    cout << "CSV: kinetic_euler_*.csv (for gnuplot/Matlab)\n";
    cout << "SVG: kinetic_euler_angles.svg (visual plot)\n";
}

}} // namespace plot


// ============ EXAMPLE USAGE FROM MAIN ============

int main(int argc, char** argv) {
    using namespace plot;

    cout << "=== Kinetic Pure C++ Plotting Demo ===\n\n";

    if(argc > 1 && std::string(argv[1]) == "--demo") {
        run_plot_example();
    } else if(argc > 1 && std::string(argv[1]) == "--generate") {
        run_plot_example();
    } else {
        // Simple: just show ASCII plot of some demo data
        cout << "Generating demo data...\n";
        run_plot_example();
        
        cout << "\n=== Done ===\n";
        cout << "Files created:\n";
        cout << "  - kinetic_euler_x.csv, kinetic_euler_y.csv, kinetic_euler_z.csv\n";
        cout << "  - kinetic_euler_angles.svg\n";
        cout << "  - ASCII plots printed above\n\n";
        
        cout << "=== How to use ===\n";
        cout << "1. Call plot::add_point(t, x1,y1,z1, x2,y2,z2, x3,y3,z3) from your kinetic code\n";
        cout << "2. Call plot::save_csv('output.csv') to export data\n";
        cout << "3. Call plot::plot_svg('plot.svg') to generate visual plots\n";
        cout << "4. Optionally pipe CSV to gnuplot: 'gnuplot -e \"set terminal png; plot output.csv\"\"\n";
    }

    return 0;
}
