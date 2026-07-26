#ifndef PLOT_GNUPLOT_H
#define PLOT_GNUPLOT_H

#include <string>
#include <vector>

namespace plot {

// Gnuplot-based professional plotting API
// Supports multiple datasets, line types, fills, legends, and export

class GnuplotPlotter {
public:
    GnuplotPlotter();
    ~GnuplotPlotter();
    
    // Set output format (svg, txt, png)
    void set_output_format(const std::string& format = "svg");
    
    // Add data series
    template<typename Container>
    bool add_series(const std::vector<double>& x, const std::vector<double>& y, 
                    const std::string& title = "", int line_type = 1) {
        if (x.empty() || x.size() != y.size()) {
            return false;
        }
        
        data_x.push_back(x);
        data_y.push_back(y);
        titles.push_back(title);
        line_types.push_back(line_type);
        num_points = static_cast<int>(x.size());
        return true;
    }
    
    // Add X-axis data (default first series)
    bool add_x_data(const std::vector<double>& x, const std::string& title = "") {
        if (data_x.empty()) {
            return false;
        }
        titles.push_back(title);
        line_types.push_back(1);
        return true;
    }
    
    // Generate output file
    bool generate(const std::string& filename, int width = 800, int height = 600);
    
    // Print gnuplot commands to stdout
    void print_commands() const;
    
    // Get gnuplot script as string
    std::string get_script() const;

private:
    std::vector<std::vector<double>> data_x, data_y;
    std::vector<std::string> titles;
    std::vector<int> line_types;
    int num_points = 0;
    std::string output_format = "svg";
};

// Quick helper functions using gnuplot-native CSV + external gnuplot call
namespace gnuplot {

// Add data point with timestamp
struct DataPoint {
    double time;
    double x, y, z;  // Euler angles
    std::vector<double> values;  // Additional values for multiple series
};

// Storage
static std::vector<DataPoint> g_points;

// Add a point
inline bool add_point(const std::vector<double>& t, const std::vector<double>& v1) {
    if (t.empty() || v1.empty()) return false;
    
    DataPoint pt;
    pt.time = t[0];
    pt.x = v1[0];
    pt.y = v1[1];
    pt.z = v1[2];
    pt.values.resize(v1.size() - 3, 0);
    
    g_points.push_back(pt);
    return true;
}

// Save to CSV file (gnuplot can read this directly)
bool save_csv(const std::string& filename);

// Call gnuplot with script and return output
std::string call_gnuplot(const std::string& script, const std::string& input_file = "");

// Generate gnuplot SVG script
bool generate_svg_script(const std::string& output_file);

} // namespace gnuplot

#endif // PLOT_GNUPLOT_H
