#pragma once

// The QiString conversions scripts depend on: float formatting, word splitting and
// number parsing, matching the original's results.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace sh {

// QiString::formatFloat(v, digits = 5): "0.0" for 0; "%e" below 1e-6 or above 1e7 (absolute value);
// otherwise "%.*f" with digits - e decimals (e = log10|v| truncated, clamped to [-8, 0]), then trailing
// zeros trimmed down to one digit after the point.
inline std::string format_float(float v, int digits = 5) {
    char buf[128];
    if (v == 0.0f) return "0.0";
    const float a = std::fabs(v);
    if (a < 1e-6f || 1e7f < a) {
        std::snprintf(buf, sizeof buf, "%e", static_cast<double>(v));
        return buf;
    }
    int e = static_cast<int>(std::log10(a));
    if (e < -7) e = -8;
    else if (e >= 0) e = 0;
    std::snprintf(buf, sizeof buf, "%.*f", digits - e, static_cast<double>(v));
    const std::string s = buf;
    int last = static_cast<int>(s.size()) - 1;
    if (last > 1) {
        for (int i = static_cast<int>(s.size()) - 1;; --i) {
            if (s[i] != '0' || s[last - 1] == '.') break;
            --last;
            if (last == 1) break;
        }
    }
    return s.substr(0, last + 1);
}

// QiString::getWord(i): the i-th word, words being separated by runs of space, tab or newline; "" if none.
inline std::string get_word(const std::string& s, int index) {
    auto sep = [](char c) { return c == ' ' || c == '\t' || c == '\n'; };
    size_t p = 0;
    for (int w = 0; w <= index; ++w) {
        while (p < s.size() && sep(s[p])) ++p;
        if (p >= s.size()) return {};
        const size_t start = p;
        while (p < s.size() && !sep(s[p])) ++p;
        if (w == index) return s.substr(start, p - start);
    }
    return {};
}

// QiString::getWordCount: the number of words, with getWord's separators.
inline int word_count(const std::string& s) {
    int n = 0;
    while (!get_word(s, n).empty()) ++n;
    return n;
}

// QiString::toFloat (strtod, narrowed) and toInt (atoi).
inline float to_float(const std::string& s) { return static_cast<float>(std::strtod(s.c_str(), nullptr)); }
inline int to_int(const std::string& s) { return std::atoi(s.c_str()); }

}  // namespace sh
