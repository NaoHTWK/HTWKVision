#include "htwkrgbaimage.h"

namespace htwk {
namespace image {

void RgbaImage::saveAsPng(const PngImageSaverPtr& pngSaver, const std::string& filename) {
    pngSaver->saveRgbaToPngImage(filename, width, height, mem);
}

} // image
} // htwk
