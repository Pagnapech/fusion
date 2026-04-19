#pragma once

#include <Arduino.h>
#include <stddef.h>

#define MAX_WAYPOINTS 50

struct Waypoint {
  double lat;
  double lon;
  bool valid;
};

extern Waypoint waypoints[MAX_WAYPOINTS];
extern int waypointCount;
extern int currentWaypoint;
extern bool navigationEnabled;

void waypointNavInit();

/** Parse one text command line (no \\r\\n). Replies via ESP-NOW where applicable. */
void waypointNavProcessCommand(const char *line);

/** Call every loop: advances waypoints, drives motors when navigating. */
void runNavigationTick();

/** Format multi-line status into buf (NUL-terminated if room). */
void waypointNavFormatStatus(char *buf, size_t bufLen);
