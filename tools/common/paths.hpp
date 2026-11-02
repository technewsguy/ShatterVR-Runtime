#pragma once

#include <algorithm>
#include <filesystem>

namespace sh::tools {

// True when 'child' is 'parent' or lies beneath it.
inline bool is_inside(const std::filesystem::path& child, const std::filesystem::path& parent) {
    namespace fs = std::filesystem;
    // absolute() first: MSVC's weakly_canonical leaves a relative path relative when it does not exist yet.
    const auto c = fs::weakly_canonical(fs::absolute(child)), p = fs::weakly_canonical(fs::absolute(parent));
    auto [pi, ci] = std::mismatch(p.begin(), p.end(), c.begin(), c.end());
    return pi == p.end();
}

// Tools that write data derived from the game must not write into the repo.
inline bool refuse_source_tree(const std::filesystem::path& out) {
#ifdef SH_SOURCE_DIR
    return is_inside(out, std::filesystem::path(SH_SOURCE_DIR));
#else
    (void)out;
    return false;
#endif
}

}  // namespace sh::tools
