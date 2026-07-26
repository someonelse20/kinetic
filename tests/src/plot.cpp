// plot.cpp - Pure C++ graphing via CSV + GNUPLOT (NO Python required!)
// Usage: cmake build, call to generate plots saved as files or pipe to gnuplot terminal

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

// ============ GNUPLOT CSV EXPORT Implementation ============

namespace gplot {

bool save_csv(const std::string& filename, const std::string& plot_type) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		std::cerr << "Error: Cannot open " << filename << std::endl;
		return false;
	}

	// CSV header with column information
	out << "# Time, data1 (True Angle), data2 (Sensor Noise), data3 (Kalman Filtered)\n";

	if (plot_type == "all" || plot_type.empty()) {
		// Save all axes in interleaved format (one point per line)
		size_t max_size = std::max({g_time_x.size(), g_time_y.size(), g_time_z.size()});
		for (size_t i = 0; i < max_size; ++i) {
			out << std::fixed << std::setprecision(4);

			// X-axis data
			if (!g_time_x.empty() && i < g_time_x.size()) {
				out << g_time_x[i] << ","
				    << (g_data1_x.size() > i ? g_data1_x[i] : 0) << ","
				    << (g_data2_x.size() > i ? g_data2_x[i] : 0) << ","
				    << (g_data3_x.size() > i ? g_data3_x[i] : 0) << "\n";
			} else {
				out << std::endl; // Empty line for missing data
			}

			// Y-axis data
			if (!g_time_y.empty() && i < g_time_y.size()) {
				out << g_time_y[i] << ","
				    << (g_data1_y.size() > i ? g_data1_y[i] : 0) << ","
				    << (g_data2_y.size() > i ? g_data2_y[i] : 0) << ","
				    << (g_data3_y.size() > i ? g_data3_y[i] : 0) << "\n";
			} else {
				out << std::endl;
			}

			// Z-axis data
			if (!g_time_z.empty() && i < g_time_z.size()) {
				out << g_time_z[i] << ","
				    << (g_data1_z.size() > i ? g_data1_z[i] : 0) << ","
				    << (g_data2_z.size() > i ? g_data2_z[i] : 0) << ","
				    << (g_data3_z.size() > i ? g_data3_z[i] : 0) << "\n";
			} else {
				out << std::endl;
			}
		}
	} else if (plot_type == "x") {
		for (size_t i = 0; i < g_time_x.size(); ++i) {
			out << g_time_x[i] << ","
			    << g_data1_x[i] << ","
			    << g_data2_x[i] << ","
			    << g_data3_x[i] << "\n";
		}
	} else if (plot_type == "y") {
		for (size_t i = 0; i < g_time_y.size(); ++i) {
			out << g_time_y[i] << ","
			    << g_data1_y[i] << ","
			    << g_data2_y[i] << ","
			    << g_data3_y[i] << "\n";
		}
	} else if (plot_type == "z") {
		for (size_t i = 0; i < g_time_z.size(); ++i) {
			out << g_time_z[i] << ","
			    << g_data1_z[i] << ","
			    << g_data2_z[i] << ","
			    << g_data3_z[i] << "\n";
		}
	}

	size_t total_points = 0;
	if (!g_time_x.empty()) total_points = std::max(total_points, g_time_x.size());
	if (!g_time_y.empty()) total_points = std::max(total_points, g_time_y.size());
	if (!g_time_z.empty()) total_points = std::max(total_points, g_time_z.size());

	std::cout << "CSV data saved to " << filename << " ("
	          << total_points << " data points)\n";

	out.close();
	return true;
}

// Generate SVG plot using gnuplot-native style
bool generate_svg(const std::string& filename, int width, int height) {
	if (g_time_x.empty() && g_time_y.empty() && g_time_z.empty()) {
		std::cerr << "Error: No data to plot. Call add_point_x()/add_point_y()/add_point_z() first.\n";
		return false;
	}

	int num_points = 0;
	double time_min, time_max, range_time;

	if (!g_time_x.empty()) {
		time_min = *std::min_element(g_time_x.begin(), g_time_x.end());
		time_max = *std::max_element(g_time_x.begin(), g_time_x.end());
		range_time = (time_max - time_min > 1e-10) ? (time_max - time_min) : 1.0;
		num_points = g_time_x.size();
	} else if (!g_time_y.empty()) {
		time_min = *std::min_element(g_time_y.begin(), g_time_y.end());
		time_max = *std::max_element(g_time_y.begin(), g_time_y.end());
		range_time = (time_max - time_min > 1e-10) ? (time_max - time_min) : 1.0;
		num_points = g_time_y.size();
	} else {
		time_min = *std::min_element(g_time_z.begin(), g_time_z.end());
		time_max = *std::max_element(g_time_z.begin(), g_time_z.end());
		range_time = (time_max - time_min > 1e-10) ? (time_max - time_min) : 1.0;
		num_points = g_time_z.size();
	}

	// Create SVG file
	std::ofstream svg(filename);
	if (!svg.is_open()) {
		std::cerr << "Error: Cannot create " << filename << std::endl;
		return false;
	}

	// SVG document start - gnuplot-compatible style
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

	// Generate X-axis subplot (primary display)
	svg << "<g transform=\"translate(0,"
	    << (height - 50) << "\">\n";

	// Subtitle
	svg << "<text x=\"" << (axes_left + axes_right)/2
	    << "\" y=\"20\" text-anchor=\"middle\" font-size=\"16\">X-Axis</text>\n";

	// Draw plot area background
	svg << "<rect x=\"" << axes_left << "\" y=\"30\"\n";
	svg << "      width=\"" << (axes_right - axes_left) << "\"\n";
	svg << "      height=\"" << plot_height << "\" fill=\"#f9f9f9\" stroke=\"#ccc\"/>\n";

	// X-axis data lines (True angle only for simple view)
	svg << "<polyline points=\"";
	for (size_t i = 0; i < num_points - 1 && i < g_time_x.size(); ++i) {
		double x_svg = axes_left + (time_min > 0 ? g_time_x[i] : 0) /
		               (std::max(g_time_x[0], 1.0)) *
		               (axes_right - axes_left);
		double val = std::abs(g_data1_x[i]);
		double y_svg = height - plot_height - 30 - (val / 90.0) * plot_height; // normalize by ~90 deg range
		if (i == 0) svg << " " << x_svg << "," << y_svg;
		else svg << ", " << x_svg << "," << y_svg;
	}
	svg << "\" fill=\"none\" stroke=\"" << COLORS[0] << "\" stroke-width=\"2\"/>\n";

	svg << "<text x=\"" << axes_left - 15 << "\" y=\""
	    << height - plot_height + 20 << "\" font-size=\"14\" fill=\"" << COLORS[0] << "\">True Angle</text>\n";

	svg << "</g>\n";

	// Legend
	svg << "<g transform=\"translate("
	    << (axes_left + axes_right)/2 - 60 << ", "
	    << 80 << "\">\n";

	svg << "<rect x=\"0\" y=\"-15\" width=\"12\" height=\"2\" fill=\"" << COLORS[0] << "\"/>\n";
	svg << "<text x=\"20\" y=\"-9\" font-size=\"14\">True Angle</text>\n";

	svg << "</g>\n";

	// X-axis tick marks and labels
	svg << "<line x1=\"" << axes_left << "\" y1=\""
	    << (height - plot_height + 20) << "\" x2=\"" << axes_right
	    << "\" y2=\"" << (height - plot_height + 20) << "\" stroke=\"#333\"/>\n";
	svg << "<text x=\"" << axes_left << "\" y=\""
	    << (height - plot_height + 40) << "\" text-anchor=\"end\" font-size=\"12\">t = t_min</text>\n";

	svg << "<line x1=\"" << axes_right << "\" y1=\""
	    << (height - plot_height + 20) << "\" x2=\"" << axes_right
	    << "\" y2=\"" << (height - plot_height + 45) << "\" stroke=\"#333\"/>\n";
	svg << "<text x=\"" << axes_right << "\" y=\""
	    << (height - plot_height + 65) << "\" text-anchor=\"end\" font-size=\"12\">t = t_max</text>\n";

	// Y-axis tick marks
	svg << "<line x1=\"" << axes_left << "\" y1=\""
	    << (height - plot_height + 20) << "\" x2=\"" << (axes_left - 8)
	    << "\" y2=\"" << (height - plot_height + 20) << "\" stroke=\"#333\"/>\n";
	svg << "<text x=\"" << (axes_left - 10) << "\" y=\""
	    << (height - plot_height + 45) << "\" text-anchor=\"end\" font-size=\"12\">0 deg</text>\n";

	svg << "</g>\n"; // SVG close
	svg.close();

	std::cout << "SVG plot saved to " << filename << " ("
	          << width << "x" << height << ")\n";

	return true;
}

void clear() {
	g_time_x.clear(); g_time_y.clear(); g_time_z.clear();
	g_data1_x.clear(); g_data1_y.clear(); g_data1_z.clear();
	g_data2_x.clear(); g_data2_y.clear(); g_data2_z.clear();
	g_data3_x.clear(); g_data3_y.clear(); g_data3_z.clear();
}

// Generate gnuplot script file for external plotting
bool generate_gnuplot_script(const std::string& output_file) {
	if (g_time_x.empty() && g_time_y.empty() && g_time_z.empty()) {
		std::cerr << "Error: No data to plot. Call add_point_x()/add_point_y()/add_point_z() first.\n";
		return false;
	}

	std::ofstream out(output_file);
	if (!out.is_open()) {
		std::cerr << "Error: Cannot create " << output_file << std::endl;
		return false;
	}

	// Generate gnuplot script in gnuplot-native CSV format
	out << "# Gnuplot script for Kinetic IMU data\n";
	out << "set terminal svg size 800,600 enhanced font 'Arial,12'\n";
	out << "set output '" << output_file << ".gnuplot.svg'\n\n";

	// Set style and colors
	out << "# Color scheme: True=Blue, Sensor=Green, Kalman=Red\n";
	out << "set style line 1 lt solid lw 2 lc rgb '#1f77b4' pt none title 'True Angle (Roll)'\n";
	out << "set style line 2 lt solid lw 2 lc rgb '#2ca02c' pt none title 'Sensor Noise (Roll)'\n";
	out << "set style line 3 lt solid lw 2 lc rgb '#d62728' pt none title 'Kalman Filtered (Roll)'\n";
	out << "set key outside right top noreverse\n\n";

	// Set labels and grid
	out << "set xlabel 'Time (seconds)'\n";
	out << "set ylabel 'Angle (degrees)'\n";
	out << "set title 'Kinetic IMU Sensor Fusion - X-Axis Data'\n";
	out << "set grid\n";
	out << "set box width 0.85\n\n";

	// Save data for gnuplot to read
	size_t max_size = std::max({g_time_x.size(), g_time_y.size(), g_time_z.size()});

	// Generate CSV data block inline
	out << "plot ";
	if (!g_time_x.empty()) {
		double min_x = *std::min_element(g_time_x.begin(), g_time_x.end());
		double max_x = *std::max_element(g_time_x.begin(), g_time_x.end());
		double range_x = (max_x - min_x > 1e-10) ? (max_x - min_x) : 1.0;

		out << "<(echo \"TIME,DATA1,DATA2,DATA3\"; ";
		for (size_t i = 0; i < g_time_x.size(); ++i) {
			if (i > 0) out << " | ";
			out << std::fixed << std::setprecision(4)
			    << "\"" << min_x + range_x * i / (g_time_x.size() - 1) << "\" \""
			    << g_data1_x[i] << "\" \"" << g_data2_x[i] << "\" \"" << g_data3_x[i] << "\"";
		}
		out << ") using 1:2:(($2<0)?$2:-$2):($3>0?$3:0) ";
		out << "with lines ls 1,\n";
		out << "<(echo \"TIME,DATA1,DATA2,DATA3\"; ";
		for (size_t i = 0; i < g_time_x.size(); ++i) {
			if (i > 0) out << " | ";
			out << "\"" << min_x + range_x * i / (g_time_x.size() - 1) << "\" \""
			    << g_data2_x[i] << "\" 0 0";
		}
		out << ") using 1:2:($3>0?$3:0) ";
		out << "with linespoints ls 2,\n";
		out << "<(echo \"TIME,DATA1,DATA2,DATA3\"; ";
		for (size_t i = 0; i < g_time_x.size(); ++i) {
			if (i > 0) out << " | ";
			out << "\"" << min_x + range_x * i / (g_time_x.size() - 1) << "\" \""
			    << g_data3_x[i] << "\" 0 0";
		}
		out << ") using 1:2:($4>0?$4:0) ";
		out << "with linespoints ls 3";
	}

	out << "\n";
	out.close();

	std::cout << "Gnuplot script saved to " << output_file << std::endl;
	return true;
}

// Call gnuplot with a script and input data
std::string call_gnuplot(const std::string& script, const std::string& input_file) {
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
		execlp(cmd.c_str(), cmd.c_str(), "-e", script.c_str(), NULL);
		exit(1); // Failed to execute
	} else if (pid > 0) {
		// Parent process - wait for child
		int status;
		waitpid(pid, &status, WNOHANG);

		std::string output = "";
		if (WIFEXITED(status)) {
			// Child completed successfully
		} else if (WIFSIGNALED(status)) {
			int sig = WTERMSIG(status);
			std::cerr << "Gnuplot terminated by signal " << sig << std::endl;
		}
	} else {
		std::cerr << "Error: Failed to fork gnuplot process" << std::endl;
	}

	std::string output = "";
	return output;
}

// ============ GNUPLOT CSV Implementation using external gnuplot binary ============

bool save_gnuplot_csv(const std::string& filename) {
	if (g_time_x.empty() && g_time_y.empty() && g_time_z.empty()) {
		std::cerr << "Error: No data to save. Call add_point_x()/add_point_y()/add_point_z() first.\n";
		return false;
	}

	// Create gnuplot CSV file using gnuplot's native format
	std::ofstream out(filename);
	if (!out.is_open()) {
		std::cerr << "Error: Cannot create " << filename << std::endl;
		return false;
	}

	size_t max_size = std::max({g_time_x.size(), g_time_y.size(), g_time_z.size()});

	// gnuplot CSV format - one data series per block
	for (int axis = 0; axis < 3; ++axis) {
		if (axis == 0 && g_time_x.empty()) continue;
		if (axis == 1 && g_time_y.empty()) continue;
		if (axis == 2 && g_time_z.empty()) continue;

		std::vector<double>& time_data = (axis == 0) ? g_time_x :
		                                 (axis == 1) ? g_time_y : g_time_z;
		std::vector<double>& data_main = (axis == 0) ? g_data1_x :
		                                 (axis == 1) ? g_data1_y : g_data1_z;

		double min_t = *std::min_element(time_data.begin(), time_data.end());
		double max_t = *std::max_element(time_data.begin(), time_data.end());
		double range_t = (max_t - min_t > 1e-10) ? (max_t - min_t) : 1.0;

		out << "# " << ((axis == 0) ? "X" : ((axis == 1) ? "Y" : "Z")) << "-Axis Data\n";
		out << "# Column 1: Time, Columns 2+: VALUES\n";
		out << std::fixed << std::setprecision(4) << "\n";

		for (size_t i = 0; i < time_data.size(); ++i) {
			double t_interp = min_t + range_t * i / (time_data.size() - 1);
			out << t_interp << " ";
			if (axis == 0) out << data_main[i] << " ";
			else if (axis == 1) out << g_data1_y[i] << " ";
			else out << g_data1_z[i];
			out << std::endl;
		}

		if (axis > 0) out << "\n"; // Empty line between axes
	}

	size_t total_points = max_size;
	std::cout << "Gnuplot CSV saved to " << filename << " ("
	          << total_points << " data points)\n";

	out.close();
	return true;
}

} // namespace gplot

} // namespace plot
