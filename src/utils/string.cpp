// SPDX-License-Identifier: MIT

#include <string>
#include <vector>

namespace utils
{
    std::string stringJoin(const std::vector<std::string>& items, const std::string& delim)
    {
        std::string result;
        for (const std::string& item : items)
        {
            if (0 < result.length())
            {
                result += delim;
            }
            result += item;
        }
        return result;
    }
}
