#include "assets/apk_archive.hpp"

#include <miniz.h>

#include <stdexcept>

namespace sh {

struct ApkArchive::Impl {
    mz_zip_archive zip{};
};

ApkArchive::ApkArchive(const std::filesystem::path& path) : impl_(std::make_unique<Impl>()) {
    if (!mz_zip_reader_init_file(&impl_->zip, path.string().c_str(), 0)) {
        throw std::runtime_error("cannot open archive " + path.string() + ": " +
                                 mz_zip_get_error_string(mz_zip_get_last_error(&impl_->zip)));
    }
    const mz_uint count = mz_zip_reader_get_num_files(&impl_->zip);
    entries_.reserve(count);
    for (mz_uint i = 0; i < count; ++i) {
        mz_zip_archive_file_stat st{};
        if (!mz_zip_reader_file_stat(&impl_->zip, i, &st)) {
            throw std::runtime_error("cannot stat archive entry " + std::to_string(i));
        }
        if (st.m_is_directory) continue;
        entries_.push_back({st.m_filename, st.m_uncomp_size, i});
    }
}

ApkArchive::~ApkArchive() { mz_zip_reader_end(&impl_->zip); }

const ApkArchive::Entry* ApkArchive::find(std::string_view name) const {
    for (const auto& e : entries_) {
        if (e.name == name) return &e;
    }
    return nullptr;
}

std::vector<std::uint8_t> ApkArchive::read(const Entry& entry) const {
    std::vector<std::uint8_t> out(static_cast<size_t>(entry.size));
    if (!mz_zip_reader_extract_to_mem(&impl_->zip, entry.index, out.data(), out.size(), 0)) {
        throw std::runtime_error("cannot extract " + entry.name + ": " +
                                 mz_zip_get_error_string(mz_zip_get_last_error(&impl_->zip)));
    }
    return out;
}

}  // namespace sh
