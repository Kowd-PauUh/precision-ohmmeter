#include <cstdint>
#include <cmath>
#include <algorithm>

enum class AggregationMode { MEAN, MEDIAN, MODE };

float computeMean(const float* buffer, uint8_t count) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += buffer[i];
    }
    return sum / count;
}

float computeMedian(const float* buffer, uint8_t count, float* scratch) {
    std::copy(buffer, buffer + count, scratch);
    std::sort(scratch, scratch + count);

    if (count % 2 == 0)
        return (scratch[count/2 - 1] + scratch[count/2]) / 2.0f;
    else
        return scratch[count/2];
}

float computeMode(const float* buffer, uint8_t count) {
    float most_common = buffer[0];
    int best_count = 1;

    for (uint8_t i = 0; i < count; i++) {
        int cnt = 1;
        for (uint8_t j = i + 1; j < count; j++) {
            if (std::fabs(buffer[i] - buffer[j]) < 1e-6f) {
                cnt++;
            }
        }
        if (cnt > best_count) {
            best_count = cnt;
            most_common = buffer[i];
        }
    }
    return most_common;
}

float aggregateBuffer(const float* buffer, uint8_t count, AggregationMode mode, float* scratch) {
    if (count == 0) return NAN;
    if (count == 1) return buffer[0];

    switch (mode) {
        case AggregationMode::MEAN:   return computeMean(buffer, count);
        case AggregationMode::MEDIAN: return computeMedian(buffer, count, scratch);
        case AggregationMode::MODE:   return computeMode(buffer, count);
        default:                      return NAN;
    }
}
