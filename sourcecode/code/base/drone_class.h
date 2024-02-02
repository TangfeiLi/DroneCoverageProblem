
#ifndef DRONE_CLASS_H
#define DRONE_CLASS_H

#include <string>
#include <unordered_map>
#include <iostream>

namespace drone_cover {
    using SpeedCostMap = std::unordered_map<double, double>;

    struct DroneClass {
        /**
         * Drone class id
         */
        int id;

        /**
         * Cover Range
         */
        int range;

        /**
         * Number of drones available of this class.
         */
        int num_drones;

        /**
         * Duration of drones
        */
        int duration;

        /**
         * Speed achievable(spend #speed time moving to new target).
         */
        int speed;

        DroneClass(){}
        DroneClass(int id, int range, int num_drones, int duration, int speed):id(id), range(range), num_drones(num_drones), duration(duration), speed(speed){}

        bool operator==(const DroneClass& other) const { return id == other.id; }
        bool operator!=(const DroneClass& other) const { return !(*this == other); }
        
        void print();
    };

    inline std::ostream& operator<<(std::ostream& out, const DroneClass& vc) { out << vc.id; return out; }
}

#endif