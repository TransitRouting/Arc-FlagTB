#pragma once
#include <vector>

namespace TransferPattern {

class TimestampedAlreadySeen {
public:
    TimestampedAlreadySeen(const size_t numberOfElements = 0)
        : timestamps(numberOfElements, 0)
        , currentTimestamp(0)
    {
    }

    inline void clear()
    {
        ++currentTimestamp;

        if (currentTimestamp == 0) {
            std::fill(timestamps.begin(), timestamps.end(), 0);
        }
    }

    inline bool contains(const size_t elementId)
    {
        AssertMsg(elementId < timestamps.size(), "Element Id out of bounds!");
        return (timestamps[elementId] == currentTimestamp);
    }

    inline void insert(const size_t elementId)
    {
        AssertMsg(elementId < timestamps.size(), "Element Id out of bounds!");
        timestamps[elementId] = currentTimestamp;
    }

private:
    std::vector<uint16_t> timestamps;

    uint16_t currentTimestamp;
};
}
