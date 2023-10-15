#pragma once

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "../../DataStructures/CSA/Data.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "Profiler.h"
#include "TimeShits.h"

namespace CSA {

template <bool TRANSFERS_SECOND_CRIT = true, typename PROFILER = NoProfiler, bool PATH_RETRIEVAL = true>
class ProfileCSA {
public:
    constexpr static bool TransfersSecondCrit = TRANSFERS_SECOND_CRIT;
    constexpr static bool PathRetrieval = PATH_RETRIEVAL;
    using Profiler = PROFILER;
    using Type = ProfileCSA<TransfersSecondCrit, Profiler, PathRetrieval>;
    using TripFlag = bool;

private:
    struct Leg {
        Leg(const ConnectionId enter = ConnectionId(-1), const ConnectionId exit = ConnectionId(-1))
            : enter(enter)
            , exit(exit)
        {
        }

        ConnectionId enter;
        ConnectionId exit;
    };
    struct TripArrivalElement {
        TripArrivalElement(int arrivalTime = never, ConnectionId exit = ConnectionId(-1))
            : arrivalTime(arrivalTime)
            , exit(exit)
        {
        }

        int arrivalTime;
        ConnectionId exit;
    };

    struct ProfileElement {
        ProfileElement(int departureTime = never, int arrivalTime = never, ConnectionId enter = ConnectionId(-1), ConnectionId exit = ConnectionId(-1))
            : arrivalTime(arrivalTime)
            , departureTime(departureTime)
            , enter(enter)
            , exit(exit)
        {
        }

        inline bool dominates(ProfileElement other)
        {
            return dominates(other.getDepartureTime(), other.getArrivalTime());
        }

        inline bool dominates(int newDepartureTime, int newArrivalTime)
        {
            return departureTime >= newDepartureTime && arrivalTime <= newArrivalTime;
        }

        inline int getArrivalTime() const { return arrivalTime; }
        inline int getDepartureTime() const { return departureTime; }
        inline int getEnter() const { return enter; }
        inline int getExit() const { return exit; }

        int arrivalTime;
        int departureTime;
        ConnectionId enter;
        ConnectionId exit;
    };

    struct Profile {
        Profile()
            : elements(1, ProfileElement())
        {
            elements.reserve(64);
        };

        inline int size() { return elements.size(); }

        inline bool isDominated(ProfileElement newProfileElement)
        {
            // one element in the profile (call it P) domiantes currentProfile iff P.departureTime >= currentProfile.departureTime && P.arrivalTime <= currentProfile.arrivalTime
            // In other words: We don't need currentProfile if there is a ProfileElement 'P' that departs later (or equal) and arrives earlier (or equal) as currentProfile

            int i = findIndex(newProfileElement);

            return (elements[i].getArrivalTime() <= newProfileElement.getArrivalTime());
        }

        inline void incorporate(ProfileElement newProfileElement)
        {
            int i = findIndex(newProfileElement);
            auto iter = elements.begin() + i + 1;

            iter = elements.insert(iter, newProfileElement);

            // remove all dominated element that come later in the vector
            elements.erase(
                std::remove_if(std::next(iter), elements.end(),
                    [&](auto& other) {
                        return newProfileElement.dominates(other);
                    }),
                elements.end());
        }

        inline int findIndex(ProfileElement newProfileElement) noexcept
        {
            int i(size() - 1);

            while (elements[i].getDepartureTime() < newProfileElement.getDepartureTime())
                --i;
            return i;
        }

        void printElements() const
        {
            if (TransfersSecondCrit) {
                for (auto e : elements)
                    std::cout << String::secToString(getExactArrivalTime(e.getDepartureTime())) << "\t" << String::secToString(getExactArrivalTime(e.getArrivalTime())) << " @ " << getNumberOfTransfers(e.getArrivalTime()) << " [ " << e.enter << " -> " << e.exit << " ]\n";
            } else {
                for (auto e : elements)
                    std::cout << String::secToString(e.getDepartureTime()) << "\t" << String::secToString(e.getArrivalTime()) << " [ " << e.enter << " : " << e.exit << "]\n";
            }
        }

        std::vector<ProfileElement> elements;
    };

public:
    ProfileCSA(Data& data, const Profiler& profilerTemplate = Profiler())
        : data(data)
        , reverseTransferGraph(data.transferGraph)
        , sourceStop(noStop)
        , targetStop(noStop)
        , tripArrivalTime(data.numberOfTrips(), TripArrivalElement())
        , tripReached(data.numberOfTrips(), TripFlag())
        , profiles(data.numberOfStops(), Profile())
        , distanceToTarget(data.numberOfStops(), INFTY)
        , arrivalTimeToStop(data.numberOfStops(), never)
        , sourceDominationIndex(0)
        , profiler(profilerTemplate)
    {
        reverseTransferGraph.revert();
        if (TransfersSecondCrit) {
            for (auto& connection : data.connections) {
                connection.departureTime = shiftTime(connection.departureTime);
                connection.arrivalTime = shiftTime(connection.arrivalTime);
            }
        }
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        profiler.registerPhases({ PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN, PHASE_REACHABLE_EA_QUERY });
        profiler.registerMetrics({ METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER });
        profiler.initialize();
    }

    inline void run(const StopId source, const StopId target, int minDepartureTime = 0, int maxDepartureTime = 86400) noexcept
    {
        AssertMsg(data.isStop(source), "Source stop " << source << " is not a valid stop!");
        AssertMsg(data.isStop(target), "Target stop " << target << " is not a valid stop!");
        AssertMsg(minDepartureTime >= 0 && minDepartureTime < 24 * 60 * 60, "Min Departure Time is not in the first day!");
        AssertMsg(maxDepartureTime > 0 && maxDepartureTime <= 24 * 60 * 60, "Max Departure Time is not in the first day!");
        AssertMsg(minDepartureTime < maxDepartureTime, "Min Departure Time should be smaller than Max Departure Time!");

        profiler.start();

        minDepartureTime = transformTime(minDepartureTime);
        maxDepartureTime = transformTime(maxDepartureTime);
        profiler.startPhase();
        resetDistancesToTarget(target);
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceStop = source;
        targetStop = target;
        ConnectionId earliestConnection(0);
        ConnectionId latestConnection(data.connections.size());

        earliestConnection = firstReachableConnection(minDepartureTime);
        latestConnection = firstReachableConnection(maxDepartureTime + 1);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        runOneEAQuery(earliestConnection, latestConnection, minDepartureTime);
        profiler.donePhase(PHASE_REACHABLE_EA_QUERY);

        profiler.startPhase();
        scanConnections(earliestConnection, latestConnection);
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.done();
    }

    /* template <bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline Journey getJourney() const noexcept
    {
        return getJourney(targetStop);
    }

    template <bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline Journey getJourney(StopId stop) const noexcept
    {
        Journey journey;
        if (!reachable(stop))
            return journey;

        while (stop != sourceStop) {
            const ParentLabel& label = parentLabel[stop];
            if (label.reachedByTransfer) {
                const int travelTime = data.transferGraph.get(TravelTime, label.transferId);
                journey.emplace_back(label.parent, stop, arrivalTime[stop] - travelTime, arrivalTime[stop],
                    label.transferId);
            } else {
                journey.emplace_back(label.parent, stop, data.connections[tripReached[label.tripId]].departureTime,
                    arrivalTime[stop], label.tripId);
            }
            stop = label.parent;
        }
        Vector::reverse(journey);
        return journey;
    }


    inline std::vector<Vertex> getPath(const StopId stop) const noexcept
    {
        return journeyToPath(getJourney(stop));
    }

    inline std::vector<std::string> getRouteDescription(const StopId stop) const noexcept
    {
        return data.journeyToText(getJourney(stop));
    } */

    inline std::vector<Leg> getUsedConnections(StopId stop)
    {
        std::vector<Leg> journey;

        if (!reachable(stop))
            return journey;

        if (distanceToTarget[stop] < INFTY) [[unlikely]]
            return journey;

        // this currently grabs the last element => need to fix
        const ProfileElement& element = profiles[stop].elements.back();
        int arrivalTimeAtTarget = getExactArrivalTime(element.arrivalTime);

        journey.push_back({ element.enter, element.exit });
        stop = data.connections[element.exit].arrivalStopId;

        while (distanceToTarget[stop] == INFTY) {
            Profile& profil = profiles[stop];

            int i(0);

            while (i < profil.size() && getExactArrivalTime(profil.elements[i].arrivalTime) != arrivalTimeAtTarget)
                ++i;
            Assert(0 <= i && i < profil.size());
            auto& e = profil.elements[i];

            journey.push_back({ e.enter, e.exit });
            stop = data.connections[e.exit].arrivalStopId;

            std::cout << "Current Number of legs: " << journey.size() << "\n";
            std::cout << "Stop : " << stop << " distanceToTarget: " << distanceToTarget[stop] << "\n";
            profil.printElements();
        }

        return journey;
    }

    inline bool reachable(const StopId stop) const noexcept
    {
        if (TransfersSecondCrit)
            return getExactArrivalTime(profiles[stop].elements.back().arrivalTime) < never;
        return profiles[stop].elements.back().arrivalTime < never;
    }

    inline const Profiler& getProfiler() const noexcept
    {
        return profiler;
    }

    void printProfile(const StopId stop) const
    {
        profiles[stop].printElements();
    }

    int numberOfJourneys(const StopId stop)
    {
        return profiles[stop].size() - 1;
    }

private:
    inline void resetDistancesToTarget(const StopId newTarget)
    {
        if (targetStop != noStop) {
            distanceToTarget[targetStop] = INFTY;
            for (auto edge : reverseTransferGraph.edgesFrom(targetStop)) {
                distanceToTarget[reverseTransferGraph.get(ToVertex, edge)] = INFTY;
            }
        }

        distanceToTarget[newTarget] = 0;
        for (auto edge : reverseTransferGraph.edgesFrom(newTarget)) {
            distanceToTarget[reverseTransferGraph.get(ToVertex, edge)] = transformTime(reverseTransferGraph.get(TravelTime, edge));
        }
    }
    inline void clear()
    {
        sourceStop = noStop;
        targetStop = noStop;
        Vector::fill(profiles, Profile());
        Vector::fill(tripArrivalTime, TripArrivalElement());
        Vector::fill(tripReached, TripFlag());
        Vector::fill(arrivalTimeToStop, never);
        sourceDominationIndex = 0;
    }

    inline ConnectionId firstReachableConnection(const int departureTime) const noexcept
    {
        return ConnectionId(
            Vector::lowerBound(data.connections, departureTime, [](const Connection& connection, const int time) {
                return connection.departureTime < time;
            }));
    }

    inline void runOneEAQuery(const ConnectionId earliestConnection, const ConnectionId latestConnection, const int minDepartureTime) noexcept
    {
        arrivalTimeToStop[sourceStop] = minDepartureTime;
        relaxOutgoingEdges(sourceStop, minDepartureTime);

        for (ConnectionId i(earliestConnection); i < latestConnection; ++i) {
            const Connection& connection = data.connections[i];
            if (connectionIsReachable(connection)) {
                arrivalByTrip(connection.arrivalStopId, connection.arrivalTime);
            }
        }
    }

    inline void scanConnections(const ConnectionId earliestConnection, const ConnectionId latestConnection) noexcept
    {
        AssertMsg(earliestConnection < data.connections.size(), "earliestConnection out of bounds!\n");
        AssertMsg(latestConnection - 1 < data.connections.size(), "latestConnection out of bounds!\n");

        for (ConnectionId i(latestConnection); i > earliestConnection; --i) {
            const Connection& connection = data.connections[i - 1];

            /*
            // prefetch the next connections
            if (i >= earliestConnection + 4) {
                const Connection& nextConnection = data.connections[i - 1 - 3];
                if (tripReached[nextConnection.tripId]) {
                    __builtin_prefetch(&profiles[nextConnection.arrivalStopId]);
                    __builtin_prefetch(&tripArrivalTime[nextConnection.tripId]);
                }
            }
            */
            // Through the previous EA Query
            if (!tripReached[connection.tripId])
                continue;

            const int tau1 = connection.arrivalTime + distanceToTarget[connection.arrivalStopId];
            const int tau2 = tripArrivalTime[connection.tripId].arrivalTime;
            const int tau3 = earliestArrivalTimeInProfiles(connection.arrivalStopId, connection.arrivalTime) + (TransfersSecondCrit ? offset : 0);

            const int tauC = std::min(tau1, std::min(tau2, tau3));

            if (tauC == never) [[unlikely]]
                continue;

            if (tauC < tripArrivalTime[connection.tripId].arrivalTime) {
                tripArrivalTime[connection.tripId] = tauC;
                tripArrivalTime[connection.tripId].exit = i;
            }

            profiler.countMetric(METRIC_CONNECTIONS);
            Assert(tripArrivalTime[connection.tripId].exit != ConnectionId(-1));
            const ProfileElement currentProfile(connection.departureTime, tauC, i, tripArrivalTime[connection.tripId].exit);
            Profile& profileArrivalStop = profiles[connection.arrivalStopId];

            if (!profileArrivalStop.isDominated(currentProfile) && checkSourceDomination(currentProfile)) {
                profiler.countMetric(METRIC_STOPS_BY_TRIP);

                profileArrivalStop.incorporate(currentProfile);
                relaxIncommingEdges(connection.departureStopId, currentProfile);
            }
        }
    }

    inline bool checkSourceDomination(const ProfileElement currentProfile) noexcept
    {
        Profile& sourceProfile = profiles[sourceStop];

        while (sourceDominationIndex < (int)sourceProfile.size() - 1 && sourceProfile.elements[sourceDominationIndex].getDepartureTime() >= currentProfile.getDepartureTime())
            ++sourceDominationIndex;
        return sourceProfile.elements[sourceDominationIndex].getArrivalTime() > currentProfile.getArrivalTime();
    }

    inline int earliestArrivalTimeInProfiles(const StopId stop, const int arrivalTime) noexcept
    {
        auto stopProfile = profiles[stop];

        int i(stopProfile.elements.size() - 1);

        while (stopProfile.elements[i].getDepartureTime() < arrivalTime)
            --i;
        return stopProfile.elements[i].getArrivalTime();
    }

    inline void relaxIncommingEdges(const StopId stop, ProfileElement currentProfile) noexcept
    {
        for (auto edge : reverseTransferGraph.edgesFrom(stop)) {
            profiler.countMetric(METRIC_EDGES);
            const StopId fromStop = StopId(reverseTransferGraph.get(ToVertex, edge));
            const int newDepartureTime = currentProfile.getDepartureTime() - transformTime(reverseTransferGraph.get(TravelTime, edge));
            ProfileElement newElement(newDepartureTime, currentProfile.getArrivalTime(), currentProfile.enter, currentProfile.exit);
            Profile& profileFromStop(profiles[fromStop]);

            if (profileFromStop.isDominated(newElement))
                continue;
            profileFromStop.incorporate(newElement);
            profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        }
    }

    inline void relaxOutgoingEdges(const StopId stop, const int time) noexcept
    {
        for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
            const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
            const int newArrivalTime = time + transformTime(data.transferGraph.get(TravelTime, edge));
            arrivalByTransfer(toStop, newArrivalTime);
        }
    }

    inline bool connectionIsReachableFromStop(const Connection& connection) const noexcept
    {
        return arrivalTimeToStop[connection.departureStopId] <= connection.departureTime - data.minTransferTime(connection.departureStopId);
    }

    inline bool connectionIsReachableFromTrip(const Connection& connection) const noexcept
    {
        return tripReached[connection.tripId];
    }

    inline bool connectionIsReachable(const Connection& connection) noexcept
    {
        if (connectionIsReachableFromTrip(connection))
            return true;
        if (connectionIsReachableFromStop(connection)) {
            tripReached[connection.tripId] = true;
            return true;
        }
        return false;
    }

    inline void arrivalByTrip(const StopId stop, const int time) noexcept
    {
        if (arrivalTimeToStop[stop] <= time)
            return;
        arrivalTimeToStop[stop] = time;
        relaxOutgoingEdges(stop, time);
    }

    inline void arrivalByTransfer(const StopId stop, const int time) noexcept
    {
        if (arrivalTimeToStop[stop] <= time)
            return;
        arrivalTimeToStop[stop] = time;
    }

    inline int transformTime(int time) noexcept
    {
        if (!TransfersSecondCrit)
            return time;
        return shiftTime(time);
    }

private:
    Data& data;

    TransferGraph reverseTransferGraph;
    StopId sourceStop;
    StopId targetStop;

    std::vector<TripArrivalElement> tripArrivalTime;
    std::vector<TripFlag> tripReached;
    std::vector<Profile> profiles;
    std::vector<int> distanceToTarget;
    std::vector<int> arrivalTimeToStop;

    int sourceDominationIndex;
    Profiler profiler;
};
} // namespace CSA
