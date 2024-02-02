
#include <algorithm>

#include "drone_class.h"

namespace drone_cover {
    void DroneClass::print(){
        std::cerr<<"****************DroneClass*******************"<<std::endl;
        std::cerr<<"id: "<< id <<std::endl;
        std::cerr<<"range: "<< range <<std::endl;
        std::cerr<<"num_drones: "<< num_drones <<std::endl;
        std::cerr<<"duration: "<< duration <<std::endl;
        std::cerr<<"speed: "<< speed <<std::endl;
        std::cerr<<"****************DroneClass*******************"<<std::endl;
    }
}