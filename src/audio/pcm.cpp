#include "audio/pcm.hpp"

#include <cstdlib>

#define STB_VORBIS_HEADER_ONLY
#include <stb_vorbis.c>

namespace sh::audio {

std::shared_ptr<const Pcm> decode_ogg(const std::string& bytes) {
    int channels = 0, rate = 0;
    short* data = nullptr;
    const int frames = stb_vorbis_decode_memory(reinterpret_cast<const unsigned char*>(bytes.data()),
                                                static_cast<int>(bytes.size()), &channels, &rate, &data);
    if (frames <= 0 || channels <= 0 || !data) {
        std::free(data);
        return nullptr;
    }
    auto pcm = std::make_shared<Pcm>();
    // Channels beyond two are dropped; mono is duplicated.
    std::vector<std::int16_t> stereo(static_cast<size_t>(frames) * 2);
    for (int i = 0; i < frames; ++i) {
        const short l = data[static_cast<size_t>(i) * channels];
        const short r = channels > 1 ? data[static_cast<size_t>(i) * channels + 1] : l;
        stereo[2 * static_cast<size_t>(i)] = l, stereo[2 * static_cast<size_t>(i) + 1] = r;
    }
    std::free(data);
    if (rate == kRate) {
        pcm->samples = std::move(stereo);
    } else {
        const double step = static_cast<double>(rate) / kRate;
        const size_t out_frames = static_cast<size_t>(frames / step);
        pcm->samples.resize(out_frames * 2);
        for (size_t i = 0; i < out_frames; ++i) {
            const double src = static_cast<double>(i) * step;
            const size_t a = static_cast<size_t>(src), b = std::min(a + 1, static_cast<size_t>(frames - 1));
            const double t = src - static_cast<double>(a);
            for (int c = 0; c < 2; ++c)
                pcm->samples[2 * i + c] =
                    static_cast<std::int16_t>(stereo[2 * a + c] * (1.0 - t) + stereo[2 * b + c] * t);
        }
    }
    return pcm;
}

}  // namespace sh::audio
