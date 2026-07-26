// plot_csv_simple.cpp - Minimal pure C++ graphing (NO Python needed!)
// Usage: cmake build, then call from your kinetic code

#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

namespace plot_simple {

static std::vector<double> g_time;
static std::vector<double> g_data1_x, g_data1_y, g_data1_z;

const double TIME_INTERVAL = 1.0;

// Add a single data point
void add_point(double t, double x1, double y1, double z1) {
    g_time.push_back(t);
    g_data1_x.push_back(x1);
    g_data1_y.push_back(y1);
    g_data1_z.push_back(z1);
}

// Save CSV for gnuplot or other tools
void save_csv(const char* filename) {
    std::ofstream out(filename);
    if(!out.is_open()) return;
    
    out << "# Time, X, Y, Z\n";
    for(size_t i = 0; i < g_time.size(); ++i) {
        out << std::fixed << std::setprecision(4);
        out << g_time[i] << "," 
            << g_data1_x[i] << ","
            << g_data1_y[i] << ","
            << g_data1_z[i] << "\n";
    }
    std::cout << "Saved: " << filename << " (" << g_time.size() << " pts)\n";
}

// Generate SVG plot
void plot_svg(const char* filename, int width = 800, int height = 600) {
    if(g_time.empty()) { std::cerr << "No data to plot\n"; return; }

    double tmin = *std::min_element(g_time.begin(), g_time.end());
    double tmax = *std::max_element(g_time.begin(), g_time.end());
    double trange = (tmax - tmin > 1e-10) ? (tmax - tmin) : 1.0;

    std::ofstream svg(filename);
    if(!svg.is_open()) return;

    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width 
        << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " " << height << "\">\n";
    
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"#fff\"/>\n";
    svg << "<text x=\"" << width/2.0 << "\" y=\"35\" text-anchor=\"middle\" font-size=\"20\">"
        "Kinetic Euler Angle Plot\n";
    svg << "</text>\n";

    // Axes and plot area
    int left = 60, right = width - 40;
    int h = height - 80;

    svg << "<g transform=\"translate(0," << (height-50) << "\")\">\n";
    svg << "<text x=\"" << (left+right)/2.0 << "\" y=\"25\" text-anchor=\"middle\" font-size=\"16\">X-Axis</text>\n";
    svg << "<rect x=\"" << left << "\" y=\"30\" width=\"" << (right-left) 
        << "\" height=\"" << h << "\" fill=\"#f9f9f9\" stroke=\"#ccc\"/>\n";

    // Plot line (simplified - blue = True angle)
    int xs = left, xe = right;
    int ys = height - 80 + 20;
    
    svg << "<polyline points=\"";
    for(size_t i = 0; i < std::min(g_time.size(), (size_t)100); ++i) {
        double x = left + (g_time[i] - tmin) / trange * (right - left);
        double val = g_data1_x[i];
        double y = height - 80 - 20 - ((val > 0) ? val : 0);
        if(i == 0) svg << " " << x << "," << y;
        else svg << ", " << x << "," << y;
    }
    svg << "</polyline>\n";
    svg << "<text x=\"" << left - 15 << "\" y=\"" << height - h + 20 
        << "\" font-size=\"14\" fill=\"#1f77b4\">True Angle</text>\n";
    svg << "</g>\n";

    // Y-axis subplot
    svg << "<g transform=\"translate(0," << (height-50-h/2) << "\")\">\n";
    svg << "<text x=\"" << (left+right)/2.0 << "\" y=\"25\" text-anchor=\"middle\" font-size=\"16\">Y-Axis</text>\n";
    svg << "<rect x=\"" << left << "\" y=\"30\" width=\"" << (right-left) 
        << "\" height=\"" << h/2.0 << "\" fill=\"#f9f9f9\" stroke=\"#ccc\"/>\n";
    
    svg << "<polyline points=\"";
    for(size_t i = 0; i < std::min(g_time.size(), (size_t)100); ++i) {
        double x = left + (g_time[i] - tmin) / trange * (right - left);
        double val = g_data1_y[i];
        double y = height - 80 - h/2.0 - 20 - ((val > 0) ? val : 0);
        if(i == 0) svg << " " << x << "," << y;
        else svg << ", " << x << "," << y;
    }
    svg << "</polyline>\n";
    svg << "<text x=\"" << left - 15 << "\" y=\"" << height - h + 20 
        << "\" font-size=\"14\" fill=\"#1f77b4\">True Angle</text>\n";
    svg << "</g>\n";

    // Z-axis subplot
    svg << "<g transform=\"translate(0," << (height-50-h) << "\")\">\n";
    svg << "<text x=\"" << (left+right)/2.0 << "\" y=\"25\" text-anchor=\"middle\" font-size=\"16\">Z-Axis</text>\n";
    svg << "<rect x=\"" << left << "\" y=\"30\" width=\"" << (right-left) 
        << "\" height=\"" << h/2.0 << "\" fill=\"#f9f9f9\" stroke=\"#ccc\"/>\n";
    
    svg << "<polyline points=\"";
    for(size_t i = 0; i < std::min(g_time.size(), (size_t)100); ++i) {
        double x = left + (g_time[i] - tmin) / trange * (right - left);
        double val = g_data1_z[i];
        double y = height - 80 - h/2.0 - 20 - ((val > 0) ? val : 0);
        if(i == 0) svg << " " << x << "," << y;
        else svg << ", " << x << "," << y;
    }
    svg << "</polyline>\n";
    svg << "<text x=\"" << left - 15 << "\" y=\"" << height - h + 20 
        << "\" font-size=\"14\" fill=\"#1f77b4\">True Angle</text>\n";
    svg << "</g>\n";

    svg << "</svg>\n";
    svg.close();
    std::cout << "Saved: " << filename << "\n";
}

// Print ASCII plot to terminal (for quick viewing)
void print_ascii(const std::vector<double>& data, const std::string& title = "Data") {
    if(data.empty()) return;
    
    double dmin = *std::min_element(data.begin(), data.end());
    double dmax = *std::max_element(data.begin(), data.end());
    double drange = (dmax - dmin > 1e-10) ? (dmax - dmin) : 1.0;

    std::cout << "\n" << title << " Plot\n";
    std::cout << "Range: " << dmin << " to " << dmax << "\n\n";

    for(int r = 15; r >= 0; --r) {
        double val = dmin + (r/14.0) * drange;
        std::cout << "[" << std::fixed << std::setprecision(2) << val << "]   ";
        
        for(int c = 0; c < 50; ++c) {
            double x = dmin + (c/49.0) * drange;
            int idx = static_cast<int>((x - dmin)/drange * data.size());
            if(idx >= data.size()) idx = data.size() - 1;
            
            if(std::abs(data[idx] - val) < 1e-6) std::cout << "*";
            else if(data[idx] > val) std::cout << ".";
            else std::cout << " ";
        }
        std::cout << "\n\n";
    }
}

void clear() {
    g_time.clear();
    g_data1_x.clear(); g_data1_y.clear(); g_data1_z.clear();
}

} // namespace plot_simple


// Example usage:
/*
#include "plot_csv_simple.h"  // or this file with system header guards
...
using namespace plot_simple;

// Generate data
for(int i = 0; i < 100; ++i) {
    add_point(i * TIME_INTERVAL, 
              sin(i/20.0)*45 + rand()%10 - 5,   // X angle
              cos(i/20.0)*30 + rand()%8 - 4,    // Y angle  
              tan(i/20.0)*20 + rand()%6 - 3);   // Z angle
}

// Save to CSV and SVG
save_csv("mydata.csv");
plot_svg("mydata.svg");
print_ascii(g_data1_x);
*/
