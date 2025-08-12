// SPDX-License-Identifier: MIT

#pragma once

#include <map>
#include <optional>
#include <vector>

namespace utils
{
    template <typename K, typename V>
    std::optional<V> mapGet(std::map<K, V> map, K key)
    {
        if (map.contains(key))
        {
            return std::make_optional<V>(map[key]);
        }
        return std::nullopt;
    }


    template <typename K, typename V>
    std::vector<K> mapGetKeys(const std::map<K, V>& map)
    {
        std::vector<K> result;
        result.reserve(map.size());

        for (const auto& pair : map)
        {
            result.push_back(pair.first);
        }
        return result;
    }


    template <typename K, typename V>
    std::vector<V> mapGetValues(const std::map<K, V>& map)
    {
        std::vector<V> result;
        result.reserve(map.size());

        for (const auto& pair : map)
        {
            result.push_back(pair.second);
        }
        return result;
    }
}
