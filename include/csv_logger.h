#pragma once

#include <Arduino.h>
#include "ekf_nav.h"

struct CsvAccumulator {
  uint32_t sampleCount;
  double ekfLatDegSum;
  double ekfLonDegSum;
};

void resetAccumulator(CsvAccumulator &acc);

void addSampleToAccumulator(CsvAccumulator &acc, const EkfOutput &ekf);

void printAveragedCsvRow(Stream &out, const CsvAccumulator &acc);

void printCSVHeader(Stream &out);
