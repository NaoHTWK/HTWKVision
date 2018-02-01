#include "htwkpngimagesaver.h"

#include <internal/htwkpngimagesaverlibpng.h>

namespace htwk {
namespace image  {

PngImageSaver::PngImageSaver()
{
}

PngImageSaverPtr getPngImageSaverInstace()
{
#ifdef LIBPNG_NOT_AVAILABLE
    return std::make_shared<PngImageSaverLodePng>();
#else
    return std::make_shared<PngImageSaverLibPng>();
#endif
}

} // image
} // htwk
