#pragma once
namespace CSA {
// time has 32 bits [32 ... 14 | 13 ... 9 | 8 ... 1] - rounded | number of legs | lower exact arr time bits

const int offset = (1 << 8);

inline int getRoundedArrivalTime(int time) noexcept
{
    return (((0xfFFFFE000) & (time)) >> 5);
}

inline int getExactArrivalTime(int time) noexcept
{
    return getRoundedArrivalTime(time) + (0xFF & time);
}

inline int getNumberOfTransfers(int time) noexcept
{
    return (((0b1111100000000) & (time)) >> 8);
}

inline int shiftTime(int time) noexcept
{
    return ((0xFF & (time)) + ((0xFFFFFF00 & (time)) << 5));
}

inline int increaseTransferCounter(int time) noexcept
{
    return time + offset;
}
}
