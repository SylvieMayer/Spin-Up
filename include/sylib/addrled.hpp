#include "env.hpp"
#include "system.hpp"
#include "vex/v5_apitypes.h"
#include <cstdint>
#include <vector>

namespace sylib{
    class Addrled : private Device{
        private:
            static const std::vector<uint32_t> off_buffer;
            static std::uint32_t addrleds_existing;
            const std::uint8_t smart_port;
            const std::uint8_t adi_port;
            const std::uint8_t strip_length;
            std::vector<uint32_t> buffer;
            static std::vector<sylib::Addrled *> allAddrleds;
            const V5_DeviceT device;
            static int max_addrled_strips;
        public:
            static bool addrled_enabled;
            Addrled(const std::uint8_t smart_port, const std::uint8_t adi_port, const std::uint8_t strip_length, const std::vector<std::uint32_t> colors = std::vector<std::uint32_t>());
            void set_all(std::uint32_t color);
            void set_pixel(std::uint32_t color, std::uint8_t index);
            void set_buffer(std::vector<uint32_t> colors);
            static void set_max_led_strips(int strips);
            void update() override;
            void turn_off();
            uint32_t& operator[] (std::uint32_t index);
    };
}