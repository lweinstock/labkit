#include <labkit/utils.hh>
#include <labkit/exceptions.hh>

#include <sstream>
#include <algorithm>
#include <stdint.h>

using namespace std;

namespace labkit
{

vector<string> split(string list, string delim, size_t max_size) {
    vector<string> ret;
    size_t last = 0, pos = 0;
    while ( (pos = list.find(delim.c_str(), last)) != string::npos) {
        ret.push_back( list.substr(last, pos - last) );
        last = pos + delim.size();
        // If max size is reached, append rest to the vector and stop!
        if (ret.size() == max_size-1) {
            ret.push_back( list.substr(last, string::npos) );
            break;
        }
    }
    // Push back the rest of the string as last entry
    string rest = list.substr(last);
	if (!rest.empty())
		ret.push_back(rest);
    return ret;
}

template <typename T> T convertTo(const std::string &t_val)
{
    std::istringstream iss(t_val);
    T ret;
    iss >> ret;
    bool success = !iss.fail();
    if (!success)
        throw ConversionError("Failed to convert '" + t_val + " to "
            + typeid(T).name());
    return ret;
}

template bool convertTo<bool>(const std::string &t_val);
template uint8_t convertTo<uint8_t>(const std::string &t_val);
template int convertTo<int>(const std::string &t_val);
template unsigned convertTo<unsigned>(const std::string &t_val);
template float convertTo<float>(const std::string &t_val);
template double convertTo<double>(const std::string &t_val);

std::string removeCtrlChars(const std::string &str)
{
    string ret = str;
    ret.erase(remove_if(ret.begin(), ret.end(), 
        [](char c) { return std::iscntrl(c); }), ret.end());
    return ret;
}

}