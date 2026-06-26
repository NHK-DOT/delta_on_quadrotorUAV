#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>

#include <libobsensor/ObSensor.hpp>

int main() {
    try {
        ob::Context context;
        auto devices = context.queryDeviceList();
        const uint32_t device_count = devices ? devices->deviceCount() : 0;
        std::cout << "device_count=" << device_count << std::endl;
        if(device_count == 0) {
            return 2;
        }

        auto device = devices->getDevice(0);
        auto info = device->getDeviceInfo();
        std::cout << "device name=" << info->name()
                  << " sn=" << info->serialNumber()
                  << " uid=" << info->uid() << std::endl;

        ob::Pipeline pipeline(device);
        auto profiles = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
        if(!profiles) {
            std::cerr << "no depth profile list" << std::endl;
            return 3;
        }

        std::shared_ptr<ob::VideoStreamProfile> chosen;
        std::cout << "depth_profiles:" << std::endl;
        for(uint32_t index = 0;; ++index) {
            try {
                auto profile = profiles->getProfile(index)->as<ob::VideoStreamProfile>();
                std::cout << "  [" << index << "] "
                          << profile->width() << "x" << profile->height()
                          << " fps=" << profile->fps()
                          << " format=" << profile->format() << std::endl;
                if(!chosen && profile->fps() >= 15) {
                    chosen = profile;
                }
            }
            catch(...) {
                break;
            }
        }

        if(!chosen) {
            chosen = profiles->getVideoStreamProfile();
        }

        auto config = std::make_shared<ob::Config>();
        config->enableStream(chosen);
        pipeline.start(config);

        std::shared_ptr<ob::FrameSet> frameset;
        for(int attempt = 0; attempt < 60; ++attempt) {
            frameset = pipeline.waitForFrames(1000);
            if(frameset && frameset->depthFrame()) {
                break;
            }
        }

        if(!frameset || !frameset->depthFrame()) {
            std::cerr << "no depth frame" << std::endl;
            pipeline.stop();
            return 4;
        }

        auto depth = frameset->depthFrame();
        const uint32_t width = depth->width();
        const uint32_t height = depth->height();
        const float scale = depth->getValueScale();
        const uint16_t *data = reinterpret_cast<const uint16_t *>(depth->data());
        const size_t count = static_cast<size_t>(width) * height;

        uint16_t min_depth = 65535;
        uint16_t max_depth = 0;
        uint64_t sum_depth = 0;
        size_t nonzero = 0;
        for(size_t index = 0; index < count; ++index) {
            const uint16_t value = data[index];
            if(value == 0) {
                continue;
            }
            nonzero++;
            min_depth = std::min(min_depth, value);
            max_depth = std::max(max_depth, value);
            sum_depth += value;
        }

        std::cout << "depth_frame width=" << width
                  << " height=" << height
                  << " scale=" << scale
                  << " nonzero=" << nonzero << "/" << count;
        if(nonzero > 0) {
            std::cout << " min_mm=" << (min_depth * scale)
                      << " mean_mm=" << (static_cast<double>(sum_depth) / nonzero * scale)
                      << " max_mm=" << (max_depth * scale);
        }
        std::cout << std::endl;

        pipeline.stop();
        return 0;
    }
    catch(const ob::Error &error) {
        std::cerr << "orbbec_error function=" << error.getName()
                  << " args=" << error.getArgs()
                  << " message=" << error.getMessage()
                  << " type=" << error.getExceptionType() << std::endl;
        return 10;
    }
    catch(const std::exception &error) {
        std::cerr << "std_error " << error.what() << std::endl;
        return 11;
    }
}

