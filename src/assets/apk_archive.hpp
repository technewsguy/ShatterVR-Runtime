#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace sh {

// Read-only access to the files inside an APK (a zip archive).
class ApkArchive {
public:
    struct Entry {
        std::string name;  // path inside the archive, '/'-separated
        std::uint64_t size = 0;
        std::uint32_t index = 0;
    };

    explicit ApkArchive(const std::filesystem::path& path);
    ~ApkArchive();
    ApkArchive(const ApkArchive&) = delete;
    ApkArchive& operator=(const ApkArchive&) = delete;

    // Files only; directory entries are skipped.
    const std::vector<Entry>& entries() const { return entries_; }
    const Entry* find(std::string_view name) const;
    std::vector<std::uint8_t> read(const Entry& entry) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::vector<Entry> entries_;
};

}  // namespace sh
