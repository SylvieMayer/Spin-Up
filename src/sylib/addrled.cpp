#include "env.hpp"
#include "pros/adi.h"
#include "sylib/addrled.hpp"
#include "system.hpp"
#include "vex/v5_api.h"
#include "vex/v5_apitypes.h"
#include <cstdint>
#include <vector>


namespace sylib{
    Addrled::Addrled(const uint8_t smart_port, const uint8_t adi_port, const uint8_t strip_length, std::vector<uint32_t> colors) : 
                     Device(6*max_addrled_strips,addrleds_existing), smart_port(smart_port), adi_port(adi_port), strip_length(strip_length), device(vexDeviceGetByIndex(smart_port-1)){
        
        std::unique_lock<sylib::Mutex> _lock;
        if (sylib::millis() > 1) {
            _lock = std::unique_lock<sylib::Mutex>(sylib_port_mutexes[smart_port-1]);
        }
        addrleds_existing++;
        vexDeviceAdiPortConfigSet(device, adi_port-1, kAdiPortTypeDigitalOut);
        buffer.resize(strip_length);
        colors.resize(strip_length);
        for(int i = 0; i < strip_length; i++){
            buffer[i] = 0x000000;
        }
        for(int i = 0; i < colors.size(); i++){
            buffer[i] = colors[i];
        }
    }
    std::uint32_t Addrled::addrleds_existing = 0;
    int Addrled::max_addrled_strips = 1;
    std::uint32_t& Addrled::operator[] (std::uint32_t index){
        return buffer[index];
    }
    void Addrled::set_max_led_strips(int strips){
        if(strips < 1){
            strips = 1;
        }
        max_addrled_strips = strips;
    }
    void Addrled::update(){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        if(addrled_enabled){
            vexDeviceAdiAddrLedSet(device, adi_port-1, (std::uint32_t*)buffer.data(), 0, strip_length, 0);
        }
        else{
            vexDeviceAdiAddrLedSet(device, adi_port-1, (std::uint32_t*)off_buffer.data(), 0, 64, 0);
        } 
    }
    void Addrled::set_all(std::uint32_t color){
        for(int i = 0; i < buffer.size(); i++){
            buffer[i] = color;
        }
    }
    void Addrled::set_pixel(std::uint32_t color, std::uint8_t index){
        buffer[index] = color;
    }
    void Addrled::set_buffer(std::vector<std::uint32_t> colors){
        colors.resize(strip_length);
        buffer = colors;
    }
    void Addrled::turn_off(){
        for(int i = 0; i < buffer.size(); i++){
            buffer[i] = 0x000000;
        }
    }
    bool Addrled::addrled_enabled = true;
    const std::vector<uint32_t> Addrled::off_buffer = std::vector<uint32_t>(64,0x000000);
}