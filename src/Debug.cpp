#include <labkit/debug.hh>

using namespace std;

namespace labkit 
{

void debugPrint(FILE* t_stream, const char* t_file, const char* t_function, 
    const char* t_msg, ...)
{
    // Print header with file and function
    std::string fname(t_file);
    size_t pos1 = fname.find_last_of('/');
    if (pos1 != std::string::npos)
        fname = fname.substr(pos1 +   1);
    size_t pos2 = fname.find_last_of('.');
    fname = fname.substr(0, pos2);
    fprintf(stderr, "%-18s: %20s() - ", fname.c_str(), t_function);

    // Compose message from variable arg list
    va_list args;
    va_start(args, t_msg);
    vfprintf(t_stream, t_msg, args);
    va_end(args);

    return;
}

void debugPrintStringData(FILE* t_stream, std::string t_data)
{
    std::string out {};
    bool ellipses_printed {false};
    for (size_t i = 0; i < t_data.size(); i++) {
        // Skip middle section if data is too long
        if ( (i > 40) && (i < t_data.size()-40) ) {
            if (!ellipses_printed) {
                out.append("[...]");
                ellipses_printed = true;
            }
            continue;
        }

        // Reformat ASCII special characters
        char c = t_data.at(i);
        switch (c) {
            case '\n':  out.append("\\n"); break;
            case '\r':  out.append("\\r"); break;
            case '\0':  out.append("\\0"); break;
            default: out.push_back(c);
        }

    }
    fprintf(t_stream, "'%s'\n", out.c_str());
    return;
}

void debugPrintByteData(FILE* t_stream, const uint8_t* t_data, size_t t_len)
{
    // Print string data, reformat special characters
    bool ellipses_printed {false};
    for (size_t i = 0; i < t_len; i++) {
        // Skip middle section if data is too long
        if ( (i > 8) && (i < t_len-8) ) {
            if (!ellipses_printed) {
                fprintf(t_stream, "[...] ");
                ellipses_printed = true;
            }
            continue;
        }
        fprintf(t_stream, "0x%02X ", t_data[i]);
    }
    fprintf(t_stream, "\n");
    return;
}

}