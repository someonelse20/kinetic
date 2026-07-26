#ifndef PLOT_H
#define PLOT_H

#include <string>
#include <vector>
#include <tuple>

namespace plot {

// Plotting modes - use GNUPLOT for professional-quality plots
enum class PlotMode {
    SVG,          // Generate SVG files
    TXT,          // ASCII/text plots
    CSV           // Export CSV data for external gnuplot
};

// Global plot manager instance
class PlotManager {
public:
    static PlotManager& getInstance() {
        static PlotManager instance;
        return instance;
    }
    
    // Initialize with optional gnuplot executable path
    bool initialize(const std::string& gnuplot_path = "gnuplot");
    
    // Save data to CSV for gnuplot consumption
    bool save_csv(const std::string& filename, int columns = 4) const;
    
    // Generate SVG plot file
    bool generate_svg(const std::string& filename, 
                      int width = 800, 
                      int height = 600,
                      double title_y = 35);
    
    // Clear stored data
    void clear();
    
    // Get current data size
    size_t get_data_size() const { return time.size(); }
    
private:
    PlotManager() = default;
    bool gnuplot_initialized = false;
    std::vector<double> time;
    std::tuple<std::vector<double>, std::vector<double>> data_x, data_y, data_z;  // (value, label) pairs
};

// Convenience global storage (for simple usage - mirrors your plot.py structure)
namespace gplot {

static std::vector<double> g_time_x, g_time_y, g_time_z;
static std::vector<double> g_data1_x, g_data1_y, g_data1_z;  // True angle XYZ
static std::vector<double> g_data2_x, g_data2_y, g_data2_z;  // Sensor noise XYZ
static std::vector<double> g_data3_x, g_data3_y, g_data3_z;  // Kalman filtered XYZ

const double TIME_INTERVAL = 1.0;

// Add a single data point for X axis
bool add_point_x(double t_val, 
                 double x1, double x2, double x3) {
    g_time_x.push_back(t_val);
    g_data1_x.push_back(x1);
    g_data2_x.push_back(x2);
    g_data3_x.push_back(x3);
    return true;
}

// Add a single data point for Y axis
bool add_point_y(double t_val,
                 double y1, double y2, double y3) {
    g_time_y.push_back(t_val);
    g_data1_y.push_back(y1);
    g_data2_y.push_back(y2);
    g_data3_y.push_back(y3);
    return true;
}

// Add a single data point for Z axis
bool add_point_z(double t_val,
                 double z1, double z2, double z3) {
    g_time_z.push_back(t_val);
    g_data1_z.push_back(z1);
    g_data2_z.push_back(z2);
    g_data3_z.push_back(z3);
    return true;
}

// Save all data to CSV (gnuplot readable format)
bool save_csv(const std::string& filename, const std::string& plot_type = "all");

// Generate SVG plot from stored data
bool generate_svg(const std::string& filename, int width = 800, int height = 600);

// Clear all data
void clear();

} // namespace gplot

} // namespace plot

#endif // PLOT_H
