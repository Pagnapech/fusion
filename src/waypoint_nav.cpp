#include "waypoint_nav.h"
#include "nav_state.h"
#include "geo_math.h"
#include "motor_nav.h"
#include "mission_espnow.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;
int currentWaypoint = 0;
bool navigationEnabled = false;

static const double WAYPOINT_RADIUS_M = 3.0;

static void clearAllWaypoints() {
  for (int i = 0; i < MAX_WAYPOINTS; ++i) {
    waypoints[i].lat = 0.0;
    waypoints[i].lon = 0.0;
    waypoints[i].valid = false;
  }
  waypointCount = 0;
  currentWaypoint = 0;
  navigationEnabled = false;
  stopMotors();
}

static bool hasAnyValidWaypoint() {
  for (int i = 0; i < waypointCount; ++i) {
    if (waypoints[i].valid) {
      return true;
    }
  }
  return false;
}

static void skipInvalidForward() {
  while (currentWaypoint < waypointCount && !waypoints[currentWaypoint].valid) {
    currentWaypoint++;
  }
}

void waypointNavInit() { clearAllWaypoints(); }

static int countCommas(const char *s) {
  int n = 0;
  while (*s) {
    if (*s == ',') {
      n++;
    }
    s++;
  }
  return n;
}

void waypointNavProcessCommand(const char *line) {
  if (line == nullptr || line[0] == '\0') {
    return;
  }

  char upper[192];
  size_t len = strnlen(line, sizeof(upper) - 1);
  for (size_t i = 0; i < len; ++i) {
    upper[i] = (char)toupper((unsigned char)line[i]);
  }
  upper[len] = '\0';

  if (strcmp(upper, "CLEAR") == 0) {
    clearAllWaypoints();
    missionNotify("CLEARED");
    return;
  }

  if (strcmp(upper, "STOP") == 0) {
    navigationEnabled = false;
    stopMotors();
    missionNotify("STOPPED");
    return;
  }

  if (strcmp(upper, "START") == 0) {
    if (!hasAnyValidWaypoint()) {
      missionNotify("ERR,NO_WAYPOINTS");
      return;
    }
    currentWaypoint = 0;
    skipInvalidForward();
    if (currentWaypoint >= waypointCount) {
      missionNotify("ERR,NO_WAYPOINTS");
      return;
    }
    navigationEnabled = true;
    missionNotify("OK,START");
    return;
  }

  if (strcmp(upper, "STATUS") == 0) {
    char posLine[128];
    char navLine[160];
    snprintf(posLine, sizeof(posLine), "POS,%.7f,%.7f", g_ekf_lat, g_ekf_lon);
    if (navigationEnabled && currentWaypoint < waypointCount && waypoints[currentWaypoint].valid) {
      const Waypoint &wp = waypoints[currentWaypoint];
      const double dist =
          geo_haversine_m(g_ekf_lat, g_ekf_lon, wp.lat, wp.lon);
      const double brg = geo_bearing_deg(g_ekf_lat, g_ekf_lon, wp.lat, wp.lon);
      double err = 0.0;
      if (g_ekf_heading_valid) {
        err = geo_normalize_angle_180_deg(brg - (double)g_ekf_heading_deg);
      }
      snprintf(navLine, sizeof(navLine), "NAV,WP=%d,DIST=%.2f,BRG=%.2f,ERR=%.2f", currentWaypoint, dist, brg, err);
    } else {
      snprintf(navLine, sizeof(navLine), "NAV,OFF");
    }
    missionNotify(posLine);
    missionNotify(navLine);
    return;
  }

  if (strcmp(upper, "SAVE") == 0) {
    missionNotify("OK,SAVE");
    return;
  }

  if (strncmp(upper, "WP,", 3) == 0) {
    if (countCommas(upper) < 3) {
      missionNotify("ERR,BAD_WP");
      return;
    }
    int idx = -1;
    double lat = 0.0;
    double lon = 0.0;
    if (sscanf(upper + 3, "%d,%lf,%lf", &idx, &lat, &lon) != 3) {
      missionNotify("ERR,BAD_WP");
      return;
    }
    if (idx < 0 || idx >= MAX_WAYPOINTS) {
      missionNotify("ERR,BAD_INDEX");
      return;
    }
    waypoints[idx].lat = lat;
    waypoints[idx].lon = lon;
    waypoints[idx].valid = true;
    if (idx + 1 > waypointCount) {
      waypointCount = idx + 1;
    }
    missionNotify("OK,WP");
    return;
  }

  missionNotify("ERR,UNKNOWN");
}

void waypointNavFormatStatus(char *buf, size_t bufLen) {
  if (bufLen == 0) {
    return;
  }
  char line1[96];
  char line2[160];
  snprintf(line1, sizeof(line1), "POS,%.7f,%.7f", g_ekf_lat, g_ekf_lon);
  if (navigationEnabled && currentWaypoint < waypointCount && waypoints[currentWaypoint].valid) {
    const Waypoint &wp = waypoints[currentWaypoint];
    const double dist =
        geo_haversine_m(g_ekf_lat, g_ekf_lon, wp.lat, wp.lon);
    const double brg = geo_bearing_deg(g_ekf_lat, g_ekf_lon, wp.lat, wp.lon);
    double err = 0.0;
    if (g_ekf_heading_valid) {
      err = geo_normalize_angle_180_deg(brg - (double)g_ekf_heading_deg);
    }
    snprintf(line2, sizeof(line2), "NAV,WP=%d,DIST=%.2f,BRG=%.2f,ERR=%.2f", currentWaypoint, dist, brg, err);
  } else {
    snprintf(line2, sizeof(line2), "NAV,OFF");
  }
  snprintf(buf, bufLen, "%s|%s", line1, line2);
}

void runNavigationTick() {
  if (!navigationEnabled) {
    return;
  }

  skipInvalidForward();
  if (currentWaypoint >= waypointCount) {
    navigationEnabled = false;
    stopMotors();
    missionNotify("DONE");
    return;
  }

  const Waypoint &wp = waypoints[currentWaypoint];
  if (!wp.valid) {
    currentWaypoint++;
    return;
  }

  const double dist = geo_haversine_m(g_ekf_lat, g_ekf_lon, wp.lat, wp.lon);
  const double brg = geo_bearing_deg(g_ekf_lat, g_ekf_lon, wp.lat, wp.lon);

  if (dist <= WAYPOINT_RADIUS_M) {
    currentWaypoint++;
    skipInvalidForward();
    if (currentWaypoint >= waypointCount) {
      navigationEnabled = false;
      stopMotors();
      missionNotify("DONE");
    }
    return;
  }

  double heading_err = 0.0;
  if (g_ekf_heading_valid) {
    heading_err = geo_normalize_angle_180_deg(brg - (double)g_ekf_heading_deg);
  }
  navigateTowardWaypoint(dist, heading_err);
}
