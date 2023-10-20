#pragma once

#include "../../RAPTOR/Entities/RouteSegment.h"
#include "../../RAPTOR/Entities/StopEvent.h"
#include <vector>

namespace TransferPattern {
struct HaltsOfStopInLine {
    HaltsOfStopInLine(std::vector<RAPTOR::StopEvent> halts = {})
        : halts(halts)
    {
    }

    inline void serialize(IO::Serialization& serialize) const
    {
        serialize(halts);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(halts);
    }

    std::vector<RAPTOR::StopEvent> halts;
};

struct LookupOfLine {
    LookupOfLine(std::vector<HaltsOfStopInLine> stopsAlongLine = {})
        : stopsAlongLine(stopsAlongLine)
    {
    }

    inline void serialize(IO::Serialization& serialize) const
    {
        serialize(stopsAlongLine);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(stopsAlongLine);
    }

    std::vector<HaltsOfStopInLine> stopsAlongLine;
};

struct LineAndStopIndex {
    LineAndStopIndex(RouteId routeId = noRouteId, StopIndex stopIndex = noStopIndex)
        : routeId(routeId)
        , stopIndex(stopIndex)
    {
    }

    inline void serialize(IO::Serialization& serialize)
    {
        serialize(routeId, stopIndex);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(routeId, stopIndex);
    }

    RouteId routeId;
    StopIndex stopIndex;

    bool operator<(const LineAndStopIndex& a) const
    {
        return std::tie(routeId, stopIndex) < std::tie(a.routeId, a.stopIndex);
    }

    bool beforeOnSameLine(const LineAndStopIndex& a) const
    {
        return routeId == a.routeId && stopIndex < a.stopIndex;
    }
};

struct StopLookup {
    StopLookup(std::vector<RAPTOR::RouteSegment> incidentLines = {})
        : incidentLines(incidentLines)
    {
    }

    inline void serialize(IO::Serialization& serialize) const
    {
        serialize(incidentLines);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(incidentLines);
    }

    std::vector<RAPTOR::RouteSegment> incidentLines;
};
}
