// plot_browse.cpp - Browser/window display for plots using external tools

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

namespace plot_browse {

/**
 * Escape special characters for HTML (must be before use)
 */
std::string escape_html(const std::string& s) {
    std::string result;
    result.reserve(s.size() * 2);
    for (char c : s) {
        switch (c) {
            case '<': result += "&lt;"; break;
            case '>': result += "&gt;"; break;
            case '&': result += "&amp;"; break;
            default: result += c;
        }
    }
    return result;
}

/**
 * Open SVG plot in browser using system commands (Unix-like)
 */
bool open_in_browser(const std::string& svg_file) {
#ifdef __linux__
    std::string cmd = "xdg-open '" + svg_file + "'";
    int status = system(cmd.c_str());
    if (status == 0) {
        std::cout << "\n[Plot] Opened in browser: " << svg_file << "\n";
    } else {
        std::cerr << "[Plot] Failed to open browser\n";
        std::cout << "Try manually: xdg-open '" << svg_file << "'\n";
    }
    return status == 0;
#elif defined(__APPLE__)
    std::string cmd = "/usr/bin/open '" + svg_file + "'";
    int status = system(cmd.c_str());
    if (status == 0) {
        std::cout << "\n[Plot] Opened in Safari/Chrome: " << svg_file << "\n";
    } else {
        std::cerr << "[Plot] Failed to open browser\n";
        std::cout << "Try manually: open '" << svg_file << "'\n";
    }
    return status == 0;
#else
    std::cerr << "[Plot] Browser opening not supported on this platform\n";
    return false;
#endif
}

/**
 * Get browser command for platform (Unix-like)
 */
std::string get_browser_command() {
#ifdef __linux__
    return "xdg-open ";
#elif defined(__APPLE__)
    return "/usr/bin/open ";
#else
    return "";
#endif
}

} // namespace plot_browse
