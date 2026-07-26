// plot_view.cpp - Browser/window display using external tools (fully self-contained)
#include "plot.h"
#include <iostream>
#include <fstream>
#include <sstream>     // stringstream for content_stream
#include <string>
#include <unistd.h>   // fork, execl, waitpid, system
#include <sys/wait.h>  // waitpid

namespace plot {

/**
 * Open SVG plot in browser (Unix-like systems: Linux/macOS)
 */
bool open_in_browser(const std::string& svg_file, const std::string& title = "Kinetic IMU Data") {
#ifdef __linux__
    int status = system(("xdg-open '" + svg_file).c_str());
    if (status == 0) {
        std::cout << "\n[Plot] Opened in browser\n";
    } else {
        std::cerr << "[Plot] Failed. Use: xdg-open '" << svg_file << "'\n";
    }
    return status == 0;
#elif defined(__APPLE__)
    int status = system(("open -a Safari '" + svg_file).c_str());
    if (status == 0) {
        std::cout << "\n[Plot] Opened in Safari\n";
    } else {
        std::cerr << "[Plot] Failed. Use: open '" << svg_file << "'\n";
    }
    return status == 0;
#else
    std::cerr << "[Plot] Browser opening requires Unix-like system (Linux/macOS)\n";
    return false;
#endif
}

/**
 * Generate HTML with embedded SVG for browser viewing
 */
bool save_as_html(const std::string& svg_file, const std::string& html_file, 
                  const std::string& title = "Kinetic IMU Data") {
    std::ifstream svg_in(svg_file);
    if (!svg_in) {
        std::cerr << "[Plot] Cannot read: " << svg_file << "\n";
        return false;
    }
    
    // Stream entire file to string builder
    std::ostringstream content_stream;
    content_stream << svg_in.rdbuf();
    std::string svg_content = content_stream.str();
    svg_in.close();
    
    // Escape special chars for HTML (efficient buffer-based)
    std::string safe;
    safe.reserve(svg_content.size() * 2);
    for (char c : svg_content) {
        if (c == '<') safe += "&lt;";
        else if (c == '>') safe += "&gt;";
        else if (c == '&') safe += "&amp;";
        else safe += c;
    }
    
    // Write HTML file
    std::ofstream html(html_file);
    if (!html) {
        std::cerr << "[Plot] Cannot create: " << html_file << "\n";
        return false;
    }
    
    html << "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n  <meta charset=\"UTF-8\">\n  <title>" << title << "</title>\n</head>\n<body>\n<h1>" << title << "</h1>\n";
    html << "<pre><code>" << safe << "</code></pre>\n</body>\n</html>\n";
    html.close();
    
    std::cout << "[Plot] HTML saved: " << html_file << "\n";
    
    // Open in browser automatically  
    return open_in_browser(svg_file, title);
}

/**
 * Quick view helper - uses plot.h's gplot namespace globals
 */
bool quick_view(const std::string& output_dir = "/home/william/Documents/kinetic/tests/src") {
    // This uses plot.h's global data from gplot namespace
    
    // Save CSV using fully-qualified name
    std::string csv_file = output_dir + "/quick_test.csv";
    
    // Check if we have data in gplot namespace (namespace qualified)
    bool has_x_data = !plot::gplot::g_time_x.empty();
    bool has_y_data = !plot::gplot::g_time_y.empty();
    bool has_z_data = !plot::gplot::g_time_z.empty();
    
    if (has_x_data || has_y_data || has_z_data) {
        // Save CSV 
        plot::gplot::save_csv(csv_file);
        
        // Generate SVG with fully-qualified name
        std::string svg_file = output_dir + "/quick_test.svg";
        plot::gplot::generate_svg(svg_file, 800, 600);
        
        return open_in_browser(svg_file, "Kinetic IMU Quick View");
    }
    
    std::cerr << "[Plot] No data in gplot globals. Call add_point_x()/add_point_y() first!\n";
    return false;
}

/**
 * Open gnuplot in persistent window mode (Unix-only)
 */
bool open_gnuplot_window(const std::string& script_file, const std::string& data_csv = "") {
#ifdef __linux__
    // Build gnuplot command for persistent X11 window
    std::string cmd;
    
    if (data_csv.empty()) {
        // Use existing CSV from gplot namespace
        std::string csv_file = "/tmp/quick_plot.csv";
        plot::gplot::save_csv(csv_file);
        csv_file = csv_file.substr(csv_file.find_last_of('/')+1);
        
        cmd = "gnuplot -persistent 'set terminal x11; set output \"\";' < '" + script_file + "' < \"" + csv_file + "\" 2>&1" + " &";
    } else {
        cmd = "gnuplot -persistent 'set terminal x11 size 800,600;' < '" + script_file + "' < \"" + data_csv.substr(data_csv.find_last_of('/')+1) + "\" 2>&1" + " &";
    }
    
    // Fork and run gnuplot persistently
    pid_t pid = fork();
    if (pid == 0) {
        execl("gnuplot", "gnuplot", "-e", "display terminal x11", NULL);
        exit(1);
    } else if (pid > 0) {
        waitpid(pid, nullptr, WNOHANG);
        std::cout << "\n[Plot] Gnuplot window opened. Type 'quit' to close.\n";
        return true;
    }
    
#elif defined(__APPLE__)
    // macOS: use gnuplot with x11 terminal
    pid_t pid = fork();
    if (pid == 0) {
        execl("gnuplot", "gnuplot", "-e", "set term x11; set output ''", NULL);
        exit(1);
    } else if (pid > 0) {
        waitpid(pid, nullptr, WNOHANG);
        return true;
    }
#else
    std::cerr << "[Plot] Gnuplot X11 terminal not supported\n";
    return false;
#endif
    return false;
}

} // namespace plot
