#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "../../../Helpers/IO/Serialization.h"
#include "../../Geometry/Point.h"
#include "../../Intermediate/Entities/Stop.h"

namespace RAPTOR {

class Stop {
public:
    Stop(const std::string& name = "", const Geometry::Point& coordinates = Geometry::Point(),
        const int minTransferTime = 0, const int partition = 0)
        : name(name)
        , coordinates(coordinates)
        , minTransferTime(minTransferTime)
        , partition(partition)
    {
    }

    template <typename STOP_TYPE>
    Stop(const STOP_TYPE& s)
        : name(s.name)
        , coordinates(s.coordinates)
        , minTransferTime(s.minTransferTime)
    {
        partition = 0;
    }
    Stop(IO::Deserialization& deserialize)
    {
        this->deserialize(deserialize);
    }

    friend std::ostream& operator<<(std::ostream& out, const Stop& s)
    {
        return out << "Stop{" << s.name << ", " << s.coordinates << ", " << s.minTransferTime << ", " << s.partition
                   << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept
    {
        serialize(name, coordinates, minTransferTime, partition);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept
    {
        deserialize(name, coordinates, minTransferTime, partition);
    }

public:
    std::string name { "" };
    Geometry::Point coordinates {};
    int minTransferTime { 0 };
    int partition { 0 };
};

} // namespace RAPTOR
