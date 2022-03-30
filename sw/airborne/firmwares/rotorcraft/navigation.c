/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/navigation.c
 *
 * Rotorcraft navigation functions.
 */


#define NAV_C

#include "firmwares/rotorcraft/navigation.h"

#include "pprz_debug.h"
#include "subsystems/gps.h" // needed by auto_nav from the flight plan
#include "subsystems/ins.h"
#include "state.h"

#include "autopilot.h"
#include "generated/modules.h"
#include "generated/flight_plan.h"

/* for default GUIDANCE_H_USE_REF */
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "math/pprz_algebra_int.h"

#include "subsystems/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"



/** default nav_circle_radius in meters */
#ifndef DEFAULT_CIRCLE_RADIUS
#define DEFAULT_CIRCLE_RADIUS 5.
#endif

#ifndef NAV_CLIMB_VSPEED
#define NAV_CLIMB_VSPEED 0.5
#endif

#ifndef NAV_DESCEND_VSPEED
#define NAV_DESCEND_VSPEED -0.8
#endif

/** minimum horizontal distance to waypoint to mark as arrived */
#ifndef ARRIVED_AT_WAYPOINT
#define ARRIVED_AT_WAYPOINT 3.0
#endif

/** Maximum distance from HOME waypoint before going into failsafe mode */
#ifndef FAILSAFE_MODE_DISTANCE
#define FAILSAFE_MODE_DISTANCE (1.5*MAX_DIST_FROM_HOME)
#endif

#define CLOSE_TO_WAYPOINT (15 << INT32_POS_FRAC)
#define CARROT_DIST (12 << INT32_POS_FRAC)

bool force_forward = false;

struct FloatVect2 line_vect, to_end_vect;

const float max_dist_from_home = MAX_DIST_FROM_HOME;
const float max_dist2_from_home = MAX_DIST_FROM_HOME * MAX_DIST_FROM_HOME;
float failsafe_mode_dist2 = FAILSAFE_MODE_DISTANCE * FAILSAFE_MODE_DISTANCE;
float dist2_to_home;
bool too_far_from_home;

float dist2_to_wp;

struct EnuCoor_i navigation_target;
struct EnuCoor_i navigation_carrot;

struct EnuCoor_i nav_last_point;

uint8_t last_wp UNUSED;

bool exception_flag[10] = {0}; //exception flags that can be used in the flight plan

uint8_t horizontal_mode;

int32_t nav_leg_progress;
uint32_t nav_leg_length;

bool nav_survey_active;

int32_t nav_roll, nav_pitch;
int32_t nav_heading;
int32_t nav_cmd_roll, nav_cmd_pitch, nav_cmd_yaw;
float nav_radius;
float nav_climb_vspeed, nav_descend_vspeed;

uint8_t vertical_mode;
uint32_t nav_throttle;
int32_t nav_climb, nav_altitude, nav_flight_altitude;
float flight_altitude;

/* nav_circle variables */
struct EnuCoor_i nav_circle_center;
int32_t nav_circle_radius, nav_circle_qdr, nav_circle_radians;

/* nav_route variables */
struct EnuCoor_i nav_segment_start, nav_segment_end;

static inline void nav_set_altitude(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

void set_exception_flag(uint8_t flag_num)
{
  exception_flag[flag_num] = 1;
}

static void send_segment(struct transport_tx *trans, struct link_device *dev)
{
  float sx = POS_FLOAT_OF_BFP(nav_segment_start.x);
  float sy = POS_FLOAT_OF_BFP(nav_segment_start.y);
  float ex = POS_FLOAT_OF_BFP(nav_segment_end.x);
  float ey = POS_FLOAT_OF_BFP(nav_segment_end.y);
  pprz_msg_send_SEGMENT(trans, dev, AC_ID, &sx, &sy, &ex, &ey);
}

static void send_circle(struct transport_tx *trans, struct link_device *dev)
{
  float cx = POS_FLOAT_OF_BFP(nav_circle_center.x);
  float cy = POS_FLOAT_OF_BFP(nav_circle_center.y);
  float r = POS_FLOAT_OF_BFP(nav_circle_radius);
  pprz_msg_send_CIRCLE(trans, dev, AC_ID, &cx, &cy, &r);
}

static void send_nav_status(struct transport_tx *trans, struct link_device *dev)
{
  float dist_home = sqrtf(dist2_to_home);
  float dist_wp = sqrtf(dist2_to_wp);
  pprz_msg_send_ROTORCRAFT_NAV_STATUS(trans, dev, AC_ID,
                                      &block_time, &stage_time,
                                      &dist_home, &dist_wp,
                                      &nav_block, &nav_stage,
                                      &horizontal_mode);
  if (horizontal_mode == HORIZONTAL_MODE_ROUTE) {
    send_segment(trans, dev);
  } else if (horizontal_mode == HORIZONTAL_MODE_CIRCLE) {
    send_circle(trans, dev);
  }
}

static void send_wp_moved(struct transport_tx *trans, struct link_device *dev)
{
  static uint8_t i;
  i++;
  if (i >= nb_waypoint) { i = 0; }
  pprz_msg_send_WP_MOVED_ENU(trans, dev, AC_ID,
                             &i,
                             &(waypoints[i].enu_i.x),
                             &(waypoints[i].enu_i.y),
                             &(waypoints[i].enu_i.z));
}
#endif

void nav_init(void)
{
  waypoints_init();

  nav_block = 0;
  nav_stage = 0;
  nav_altitude = POS_BFP_OF_REAL(SECURITY_HEIGHT);
  nav_flight_altitude = nav_altitude;
  flight_altitude = SECURITY_ALT;
  VECT3_COPY(navigation_target, waypoints[WP_HOME].enu_i);
  VECT3_COPY(navigation_carrot, waypoints[WP_HOME].enu_i);

  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  vertical_mode = VERTICAL_MODE_ALT;

  nav_roll = 0;
  nav_pitch = 0;
  nav_heading = 0;
  nav_cmd_roll = 0;
  nav_cmd_pitch = 0;
  nav_cmd_yaw = 0;
  nav_radius = DEFAULT_CIRCLE_RADIUS;
  nav_climb_vspeed = NAV_CLIMB_VSPEED;
  nav_descend_vspeed = NAV_DESCEND_VSPEED;
  nav_throttle = 0;
  nav_climb = 0;
  nav_leg_progress = 0;
  nav_leg_length = 1;

  too_far_from_home = false;
  dist2_to_home = 0;
  dist2_to_wp = 0;

  FLOAT_VECT2_ZERO(line_vect);
  FLOAT_VECT2_ZERO(to_end_vect);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTORCRAFT_NAV_STATUS, send_nav_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WP_MOVED, send_wp_moved);
#endif

  // generated init function
  auto_nav_init();
}

static inline void UNUSED nav_advance_carrot(void)
{
  struct EnuCoor_i *pos = stateGetPositionEnu_i();
  /* compute a vector to the waypoint */
  struct Int32Vect2 path_to_waypoint;
  VECT2_DIFF(path_to_waypoint, navigation_target, *pos);

  /* saturate it */
  VECT2_STRIM(path_to_waypoint, -(1 << 15), (1 << 15));

  int32_t dist_to_waypoint = int32_vect2_norm(&path_to_waypoint);

  if (dist_to_waypoint < CLOSE_TO_WAYPOINT) {
    VECT2_COPY(navigation_carrot, navigation_target);
  } else {
    struct Int32Vect2 path_to_carrot;
    VECT2_SMUL(path_to_carrot, path_to_waypoint, CARROT_DIST);
    VECT2_SDIV(path_to_carrot, path_to_carrot, dist_to_waypoint);
    VECT2_SUM(navigation_carrot, path_to_carrot, *pos);
  }
}

void nav_run(void)
{

#if GUIDANCE_H_USE_REF
  // if GUIDANCE_H_USE_REF, CARROT_DIST is not used
  VECT2_COPY(navigation_carrot, navigation_target);
#else
  nav_advance_carrot();
#endif

  nav_set_altitude();
}


bool nav_approaching_from(struct EnuCoor_i *wp, struct EnuCoor_i *from, int16_t approaching_time)
{
  float dist_to_point;
  struct Int32Vect2 diff;
  struct EnuCoor_i *pos = stateGetPositionEnu_i();

  /* if an approaching_time is given, estimate diff after approching_time secs */
  if (approaching_time > 0) {
    struct Int32Vect2 estimated_pos;
    struct Int32Vect2 estimated_progress;
    struct EnuCoor_i *speed = stateGetSpeedEnu_i();
    VECT2_SMUL(estimated_progress, *speed, approaching_time);
    INT32_VECT2_RSHIFT(estimated_progress, estimated_progress, (INT32_SPEED_FRAC - INT32_POS_FRAC));
    VECT2_SUM(estimated_pos, *pos, estimated_progress);
    VECT2_DIFF(diff, *wp, estimated_pos);
  }
  /* else use current position */
  else {
    VECT2_DIFF(diff, *wp, *pos);
  }
  /* compute distance of estimated/current pos to target wp
   * POS_FRAC resolution
   * convert to float to compute the norm without overflow in 32bit
   */
  struct FloatVect2 diff_f = {POS_FLOAT_OF_BFP(diff.x), POS_FLOAT_OF_BFP(diff.y)};
  dist_to_point = float_vect2_norm(&diff_f);

  /* return TRUE if we have arrived */
  if (dist_to_point < ARRIVED_AT_WAYPOINT) {
    return true;
  }

  /* if coming from a valid waypoint */
  if (from != NULL) {
    /* return TRUE if normal line at the end of the segment is crossed */
    struct Int32Vect2 from_diff;
    VECT2_DIFF(from_diff, *wp, *from);
    struct FloatVect2 from_diff_f = {POS_FLOAT_OF_BFP(from_diff.x), POS_FLOAT_OF_BFP(from_diff.y)};
    return (diff_f.x * from_diff_f.x + diff_f.y * from_diff_f.y < 0);
  }

  return false;
}

bool nav_check_wp_time(struct EnuCoor_i *wp, uint16_t stay_time)
{
  uint16_t time_at_wp;
  float dist_to_point;
  static uint16_t wp_entry_time = 0;
  static bool wp_reached = false;
  static struct EnuCoor_i wp_last = { 0, 0, 0 };
  struct Int32Vect2 diff;

  if ((wp_last.x != wp->x) || (wp_last.y != wp->y)) {
    wp_reached = false;
    wp_last = *wp;
  }

  VECT2_DIFF(diff, *wp, *stateGetPositionEnu_i());
  struct FloatVect2 diff_f = {POS_FLOAT_OF_BFP(diff.x), POS_FLOAT_OF_BFP(diff.y)};
  dist_to_point = float_vect2_norm(&diff_f);
  if (dist_to_point < ARRIVED_AT_WAYPOINT) {
    if (!wp_reached) {
      wp_reached = true;
      wp_entry_time = autopilot.flight_time;
      time_at_wp = 0;
    } else {
      time_at_wp = autopilot.flight_time - wp_entry_time;
    }
  } else {
    time_at_wp = 0;
    wp_reached = false;
  }
  if (time_at_wp > stay_time) {
    INT_VECT3_ZERO(wp_last);
    return true;
  }
  return false;
}

static inline void nav_set_altitude(void)
{
  static int32_t last_nav_alt = 0;
  if (abs(nav_altitude - last_nav_alt) > (POS_BFP_OF_REAL(0.2))) {
    nav_flight_altitude = nav_altitude;
    last_nav_alt = nav_altitude;
  }
}


/** Reset the geographic reference to the current GPS fix */
void nav_reset_reference(void)
{
  ins_reset_local_origin();
  /* update local ENU coordinates of global waypoints */
  waypoints_localize_all();
}

void nav_reset_alt(void)
{
  ins_reset_altitude_ref();
  waypoints_localize_all();
}

void nav_init_stage(void)
{
  VECT3_COPY(nav_last_point, *stateGetPositionEnu_i());
  stage_time = 0;
  nav_circle_radians = 0;
}

#include <stdio.h>
void nav_periodic_task(void)
{
  RunOnceEvery(NAV_FREQ, { stage_time++;  block_time++; });

  nav_survey_active = false;

  dist2_to_wp = 0;

  /* from flight_plan.h */
  auto_nav();

  /* run carrot loop */
  nav_run();
}

void navigation_update_wp_from_speed(uint8_t wp, struct Int16Vect3 speed_sp, int16_t heading_rate_sp)
{
  //  MY_ASSERT(wp < nb_waypoint); FIXME
  int32_t s_heading, c_heading;
  PPRZ_ITRIG_SIN(s_heading, nav_heading);
  PPRZ_ITRIG_COS(c_heading, nav_heading);
  // FIXME : scale POS to SPEED
  struct Int32Vect3 delta_pos;
  VECT3_SDIV(delta_pos, speed_sp, NAV_FREQ); /* fixme :make sure the division is really a >> */
  INT32_VECT3_RSHIFT(delta_pos, delta_pos, (INT32_SPEED_FRAC - INT32_POS_FRAC));
  waypoints[wp].enu_i.x += (s_heading * delta_pos.x + c_heading * delta_pos.y) >> INT32_TRIG_FRAC;
  waypoints[wp].enu_i.y += (c_heading * delta_pos.x - s_heading * delta_pos.y) >> INT32_TRIG_FRAC;
  waypoints[wp].enu_i.z += delta_pos.z;
  int32_t delta_heading = heading_rate_sp / NAV_FREQ;
  delta_heading = delta_heading >> (INT32_SPEED_FRAC - INT32_POS_FRAC);
  nav_heading += delta_heading;

  INT32_COURSE_NORMALIZE(nav_heading);
  RunOnceEvery(10, DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp,
               &(waypoints[wp].enu_i.x),
               &(waypoints[wp].enu_i.y),
               &(waypoints[wp].enu_i.z)));
}

bool nav_detect_ground(void)
{
  if (!autopilot.ground_detected) { return false; }
  autopilot.ground_detected = false;
  return true;
}

bool nav_is_in_flight(void)
{
  return autopilot_in_flight();
}

/** Home mode navigation */
void nav_home(void)
{
  horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
  VECT3_COPY(navigation_target, waypoints[WP_HOME].enu_i);

  vertical_mode = VERTICAL_MODE_ALT;
  nav_altitude = waypoints[WP_HOME].enu_i.z;
  nav_flight_altitude = nav_altitude;

  dist2_to_wp = dist2_to_home;

  /* run carrot loop */
  nav_run();
}

/** Set manual roll, pitch and yaw without stabilization
 *
 * @param[in] roll command in pprz scale (int32_t)
 * @param[in] pitch command in pprz scale (int32_t)
 * @param[in] yaw command in pprz scale (int32_t)
 *
 * This function allows to directly set commands from the flight plan,
 * if in nav_manual mode.
 * This is for instance useful for helicopters during the spinup
 */
void nav_set_manual(int32_t roll, int32_t pitch, int32_t yaw)
{
  horizontal_mode = HORIZONTAL_MODE_MANUAL;
  nav_cmd_roll = roll;
  nav_cmd_pitch = pitch;
  nav_cmd_yaw = yaw;
}

/** Returns squared horizontal distance to given point */
float get_dist2_to_point(struct EnuCoor_i *p)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  struct FloatVect2 pos_diff;
  pos_diff.x = POS_FLOAT_OF_BFP(p->x) - pos->x;
  pos_diff.y = POS_FLOAT_OF_BFP(p->y) - pos->y;
  return pos_diff.x * pos_diff.x + pos_diff.y * pos_diff.y;
}

/** Returns squared horizontal distance to given waypoint */
float get_dist2_to_waypoint(uint8_t wp_id)
{
  return get_dist2_to_point(&waypoints[wp_id].enu_i);
}

/** Computes squared distance to the HOME waypoint potentially sets
 * #too_far_from_home
 */
void compute_dist2_to_home(void)
{
  dist2_to_home = get_dist2_to_waypoint(WP_HOME);
  too_far_from_home = dist2_to_home > max_dist2_from_home;
#ifdef InGeofenceSector
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  too_far_from_home = too_far_from_home || !(InGeofenceSector(pos->x, pos->y));
#endif
}

/** Set nav_heading in radians. */
void nav_set_heading_rad(float rad)
{
  nav_heading = ANGLE_BFP_OF_REAL(rad);
  INT32_COURSE_NORMALIZE(nav_heading);
}

/** Set nav_heading in degrees. */
void nav_set_heading_deg(float deg)
{
  nav_set_heading_rad(RadOfDeg(deg));
}

/** Set heading to point towards x,y position in local coordinates */
void nav_set_heading_towards(float x, float y)
{
  struct FloatVect2 target = {x, y};
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
  // don't change heading if closer than 0.5m to target
  if (VECT2_NORM2(pos_diff) > 0.25) {
    float heading_f = atan2f(pos_diff.x, pos_diff.y);
    nav_heading = ANGLE_BFP_OF_REAL(heading_f);
  }
}

/** Set heading in the direction of a waypoint */
void nav_set_heading_towards_waypoint(uint8_t wp)
{
  nav_set_heading_towards(WaypointX(wp), WaypointY(wp));
}

/** Set heading in the direction of the target*/
void nav_set_heading_towards_target(void)
{
  nav_set_heading_towards(POS_FLOAT_OF_BFP(navigation_target.x),
                          POS_FLOAT_OF_BFP(navigation_target.y));
}

/** Set heading to the current yaw angle */
void nav_set_heading_current(void)
{
  nav_heading = stateGetNedToBodyEulers_i()->psi;
}

void nav_set_failsafe(void)
{
  autopilot_set_mode(AP_MODE_FAILSAFE);
}


/***********************************************************
 * built in navigation routines
 **********************************************************/

void nav_circle(struct EnuCoor_i *wp_center, int32_t radius)
{
  if (radius == 0) {
    VECT2_COPY(navigation_target, *wp_center);
    dist2_to_wp = get_dist2_to_point(wp_center);
  } else {
    struct Int32Vect2 pos_diff;
    VECT2_DIFF(pos_diff, *stateGetPositionEnu_i(), *wp_center);
    // go back to half metric precision or values are too large
    //INT32_VECT2_RSHIFT(pos_diff,pos_diff,INT32_POS_FRAC/2);
    // store last qdr
    int32_t last_qdr = nav_circle_qdr;
    // compute qdr
    nav_circle_qdr = int32_atan2(pos_diff.y, pos_diff.x);
    // increment circle radians
    if (nav_circle_radians != 0) {
      int32_t angle_diff = nav_circle_qdr - last_qdr;
      INT32_ANGLE_NORMALIZE(angle_diff);
      nav_circle_radians += angle_diff;
    } else {
      // Smallest angle to increment at next step
      nav_circle_radians = 1;
    }

    // direction of rotation
    int8_t sign_radius = radius > 0 ? 1 : -1;
    // absolute radius
    int32_t abs_radius = abs(radius);
    // carrot_angle
    int32_t carrot_angle = ((CARROT_DIST << INT32_ANGLE_FRAC) / abs_radius);
    Bound(carrot_angle, (INT32_ANGLE_PI / 16), INT32_ANGLE_PI_4);
    carrot_angle = nav_circle_qdr - sign_radius * carrot_angle;
    int32_t s_carrot, c_carrot;
    PPRZ_ITRIG_SIN(s_carrot, carrot_angle);
    PPRZ_ITRIG_COS(c_carrot, carrot_angle);
    // compute setpoint
    VECT2_ASSIGN(pos_diff, abs_radius * c_carrot, abs_radius * s_carrot);
    INT32_VECT2_RSHIFT(pos_diff, pos_diff, INT32_TRIG_FRAC);
    VECT2_SUM(navigation_target, *wp_center, pos_diff);
  }
  nav_circle_center = *wp_center;
  nav_circle_radius = radius;
  horizontal_mode = HORIZONTAL_MODE_CIRCLE;
}


void nav_route(struct EnuCoor_i *wp_start, struct EnuCoor_i *wp_end)
{
  struct Int32Vect2 wp_diff, pos_diff, wp_diff_prec;
  VECT2_DIFF(wp_diff, *wp_end, *wp_start);
  VECT2_DIFF(pos_diff, *stateGetPositionEnu_i(), *wp_start);
  // go back to metric precision or values are too large
  VECT2_COPY(wp_diff_prec, wp_diff);
  INT32_VECT2_RSHIFT(wp_diff, wp_diff, INT32_POS_FRAC);
  INT32_VECT2_RSHIFT(pos_diff, pos_diff, INT32_POS_FRAC);
  uint32_t leg_length2 = Max((wp_diff.x * wp_diff.x + wp_diff.y * wp_diff.y), 1);
  nav_leg_length = int32_sqrt(leg_length2);
  nav_leg_progress = (pos_diff.x * wp_diff.x + pos_diff.y * wp_diff.y) / (int32_t)nav_leg_length;
  int32_t progress = Max((CARROT_DIST >> INT32_POS_FRAC), 0);
  nav_leg_progress += progress;
  int32_t prog_2 = nav_leg_length;
  Bound(nav_leg_progress, 0, prog_2);
  struct Int32Vect2 progress_pos;
  VECT2_SMUL(progress_pos, wp_diff_prec, ((float)nav_leg_progress) / nav_leg_length);
  VECT2_SUM(navigation_target, *wp_start, progress_pos);

  nav_segment_start = *wp_start;
  nav_segment_end = *wp_end;
  horizontal_mode = HORIZONTAL_MODE_ROUTE;

  dist2_to_wp = get_dist2_to_point(wp_end);
}


/************** Oval Navigation **********************************************/

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

enum oval_status { OR12, OC2, OR21, OC1 };
enum oval_status oval_status;
uint8_t nav_oval_count;

void nav_oval_init(void)
{
  oval_status = OC2;
  nav_oval_count = 0;
}

/**
 * Navigation along a figure O. One side leg is defined by waypoints [p1] and [p2].
 * The navigation goes through 4 states: OC1 (half circle next to [p1]),
 * OR21 (route [p2] to [p1], OC2 (half circle next to [p2]) and OR12 (opposite leg).
 * Initial state is the route along the desired segment (OC2).
 */
void nav_oval(uint8_t p1, uint8_t p2, float radius)
{
  radius = - radius; /* Historical error ? */
  int32_t alt = waypoints[p1].enu_i.z;
  waypoints[p2].enu_i.z = alt;

  float p2_p1_x = waypoints[p1].enu_f.x - waypoints[p2].enu_f.x;
  float p2_p1_y = waypoints[p1].enu_f.y - waypoints[p2].enu_f.y;
  float d = sqrtf(p2_p1_x * p2_p1_x + p2_p1_y * p2_p1_y);

  /* Unit vector from p1 to p2 */
  int32_t u_x = POS_BFP_OF_REAL(p2_p1_x / d);
  int32_t u_y = POS_BFP_OF_REAL(p2_p1_y / d);

  /* The half circle centers and the other leg */
  struct EnuCoor_i p1_center = { waypoints[p1].enu_i.x + radius * -u_y,
           waypoints[p1].enu_i.y + radius * u_x,
           alt
  };
  struct EnuCoor_i p1_out = { waypoints[p1].enu_i.x + 2 * radius * -u_y,
           waypoints[p1].enu_i.y + 2 * radius * u_x,
           alt
  };

  struct EnuCoor_i p2_in = { waypoints[p2].enu_i.x + 2 * radius * -u_y,
           waypoints[p2].enu_i.y + 2 * radius * u_x,
           alt
  };
  struct EnuCoor_i p2_center = { waypoints[p2].enu_i.x + radius * -u_y,
           waypoints[p2].enu_i.y + radius * u_x,
           alt
  };

  int32_t qdr_out_2 = INT32_ANGLE_PI - int32_atan2_2(u_y, u_x);
  int32_t qdr_out_1 = qdr_out_2 + INT32_ANGLE_PI;
  if (radius < 0) {
    qdr_out_2 += INT32_ANGLE_PI;
    qdr_out_1 += INT32_ANGLE_PI;
  }
  int32_t qdr_anticipation = ANGLE_BFP_OF_REAL(radius > 0 ? -15 : 15);

  switch (oval_status) {
    case OC1 :
      nav_circle(&p1_center, POS_BFP_OF_REAL(-radius));
      if (NavQdrCloseTo(INT32_DEG_OF_RAD(qdr_out_1) - qdr_anticipation)) {
        oval_status = OR12;
        InitStage();
        LINE_START_FUNCTION;
      }
      return;

    case OR12:
      nav_route(&p1_out, &p2_in);
      if (nav_approaching_from(&p2_in, &p1_out, CARROT)) {
        oval_status = OC2;
        nav_oval_count++;
        InitStage();
        LINE_STOP_FUNCTION;
      }
      return;

    case OC2 :
      nav_circle(&p2_center, POS_BFP_OF_REAL(-radius));
      if (NavQdrCloseTo(INT32_DEG_OF_RAD(qdr_out_2) - qdr_anticipation)) {
        oval_status = OR21;
        InitStage();
        LINE_START_FUNCTION;
      }
      return;

    case OR21:
      nav_route(&waypoints[p2].enu_i, &waypoints[p1].enu_i);
      if (nav_approaching_from(&waypoints[p1].enu_i, &waypoints[p2].enu_i, CARROT)) {
        oval_status = OC1;
        InitStage();
        LINE_STOP_FUNCTION;
      }
      return;

    default: /* Should not occur !!! Doing nothing */
      return;
  }
}
/*
#ifdef TRAFFIC_INFO
#include "modules/multi/traffic_info.h"

void nav_follow(uint8_t ac_id, uint32_t distance, uint32_t height)
{
    struct EnuCoor_i* target = acInfoGetPositionEnu_i(ac_id);


    float alpha = M_PI / 2 - acInfoGetCourse(ac_id);
    float ca = cosf(alpha), sa = sinf(alpha);
    target->x += - distance * ca;
    target->y += - distance * sa;
    target->z = (Max(target->z + height, SECURITY_HEIGHT)); // todo add ground height to check

    ENU_OF_TO_NED(navigation_target, *target);
}
#else*/
void nav_follow(uint8_t  __attribute__((unused)) _ac_id, uint32_t  __attribute__((unused)) distance,
                uint32_t  __attribute__((unused)) height) {}

/**
 * BEGIN FOR NO FLY ZONES
 **/

int intersect_two_lines_absolute(float *x_i, float *y_i, float ax0, float ay0, float ax1, float ay1, float bx0, float by0, float bx1, float by1) {
  float mb = (by1 - by0) / (bx1 - bx0), ma = (ay1 - ay0) / (ax1 - ax0);
  if(ma == mb) {
    return 0;
  }
  float denom = (ay1 - ay0) * (bx1 - bx0) - (by1 - by0) * (ax1 - ax0);
  float num_a = (by0 - ay0) * (bx1 - bx0) - (bx0 - ax0) * (by1 - by0);
  float num_b = (by0 - ay0) * (ax1 - ax0) - (bx0 - ax0) * (ay1 - ay0);
  float fac_a = num_a / denom, fac_b = num_b / denom;
  if(fac_a <= 0 || fac_a >= 1) {
    return 0;
  }
  if(fac_b <= 0 || fac_b >= 1) {
    return 0;
  }

  *x_i = ax0 + fac_a * (ax1 - ax0);
  *y_i = ay0 + fac_a * (ay1 - ay0);
  return 1;
}

//formula from Wikipedia:
//https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
float * centroid(int num_verts, coords *verts) {
  int i = 0;
  float *C = calloc(2, sizeof(float));
  float A = verts[num_verts-1][0] * verts[0][1] - verts[0][0] * verts[num_verts-1][1];
  float Cx = (verts[num_verts-1][0] + verts[0][0]) * (verts[num_verts-1][0]*verts[0][1] - verts[0][0]*verts[num_verts-1][1]),
    Cy = (verts[num_verts-1][1] + verts[0][1]) * (verts[num_verts-1][0]*verts[0][1] - verts[0][0]*verts[num_verts-1][1]);
  for(i=0; i < num_verts-1; i++) {
    A += verts[i][0]*verts[i+1][1] - verts[i+1][0]*verts[i][1];
    Cx += (verts[i][0] + verts[i+1][0]) * (verts[i][0]*verts[i+1][1] - verts[i][1]*verts[i+1][0]);
    Cy += (verts[i][1] + verts[i+1][1]) * (verts[i][0]*verts[i+1][1] - verts[i][1]*verts[i+1][0]);
  }
  A /= 2;
  Cx /= (6 * A);
  Cy /= (6 * A);
  C[0] = Cx;
  C[1] = Cy;
  return C;
}

//TODO come up with a way to make the ratio shrink as the NFZ grows; for now use area
coords *buffer_zone(int num_verts, int *verts) {
  const float RATIO = 1.25;
  coords verts_as_coords[num_verts];
  for(int i = 0; i < num_verts; i++) {
    verts_as_coords[i][0] = (float)waypoints[verts[i]].enu_i.x;
    verts_as_coords[i][1] = (float)waypoints[verts[i]].enu_i.y;
  }
  float area = 0;
  for(int i = 0; i < num_verts; i++) {
    area += verts_as_coords[i][0] * verts_as_coords[(i+1)%num_verts][1] - verts_as_coords[(i+1)%num_verts][0] * verts_as_coords[i][1];
  }
  if(0 > area) area = -area;
  area /= 2;
  float *c = centroid(num_verts, verts_as_coords);
  coords *bz = (coords *)calloc(sizeof(coords), num_verts);
  for(int i = 0; i < num_verts; i++) {
    bz[i][0] = c[0] + RATIO * (verts_as_coords[i][0] - c[0]);
    bz[i][1] = c[1] + RATIO * (verts_as_coords[i][1] - c[1]);
  }
  return bz;
}

//The following enum and structs are now defined in the generated flight plan,
//but it's useful to have them as comments here for reference
/*enum VISIT_STATUS {UNVISITED, VISITING, VISITED};

//int node_id_gen = 0;

struct vis_node {
  int num_neighbors;
  int capacity;
  struct vis_node **neighbors;
  int *weights;
  float x, y;
  int node_id;
  enum VISIT_STATUS status;
  };*/

/*struct path_node {
  struct vis_node *wp;
  struct path_node *next;
  };

struct astar_node {
  struct vis_node *wp;
  struct astar_node *next;
  float priority;
  float dist_so_far;
  struct astar_node *parent; //NOT prev! not a doubly linked list!
  }*/

struct vis_node *init_vis_node(float x_in, float y_in, int capacity) {
  static int node_id_gen = 0;
  node_id_gen++;
  struct vis_node ret = {0, capacity, calloc(capacity, sizeof(struct vis_node*)), calloc(capacity, sizeof(int)), x_in, y_in, node_id_gen, UNVISITED};
  struct vis_node *ptr = (struct vis_node*)calloc(1, sizeof(struct vis_node));
  memcpy(ptr, &ret, sizeof(ret));
  return ptr;
}

struct path_node *init_path_node(struct vis_node *vn, struct path_node *prev) {
  struct path_node *ret = calloc(1, sizeof(struct path_node));
  if(NULL != prev) {
    prev->next = ret;
  }
  ret->wp = vn;
  ret->next = NULL;
  return ret;
}

struct astar_node *init_first_astar_node(struct vis_node *first, struct vis_node *target) {
  struct astar_node *ret = calloc(1, sizeof(struct astar_node));
  ret->wp = first;
  ret->priority = sqrt(DistanceSquare(first->x, first->y, target->x, target->y));
  ret->dist_so_far = 0.0f;
  ret->next = NULL;
  ret->parent = NULL;
  return ret;
}

struct astar_node *init_astar_node(struct astar_node *parent, struct vis_node *next, struct vis_node *target, float dist_so_far) {
  struct astar_node *ret = calloc(1, sizeof(struct astar_node));
  ret->wp = next;
  ret->next = NULL;
  ret->priority = dist_so_far + sqrt(DistanceSquare(next->x, next->y, target->x, target->y));
  ret->dist_so_far = dist_so_far;
  ret->parent = parent;
  return ret;
}

struct astar_node *astar_pop(struct astar_node **head) {
  if(NULL == head || NULL == *head) return NULL;
  struct astar_node *ret = *head;
  *head = (*head)->next;
  return ret;
}

struct astar_node *astar_insert(struct astar_node *head, struct astar_node *next) {
  if(!head) return next;
  if(!next) return head;
  if(head->priority > next->priority) {
    next->next = head;
    return next;
  }
  struct astar_node *prev = head;
  while(NULL != prev->next && prev->next->priority <= next->priority) prev = prev->next;
  next->next = prev->next;
  prev->next = next;
  return head;
}

void astar_free(struct astar_node *start) {
  struct astar_node *next = NULL;
  for(struct astar_node *p = start; NULL != p; p = next) {
    next = p->next;
    free(p);
  }
}

//return:
//1 if the neighbor was successfully added
//0 if it was not successfully added
//-1 if it was already a neighbor
int add_neighbor(struct vis_node *node, struct vis_node *new_neighbor, int weight) {
  //make sure we don't add the same neighbor twice
  for(int i = 0; i < node->num_neighbors; i++) {
    if(node->neighbors[i]->x == new_neighbor->x &&
       node->neighbors[i]->y == new_neighbor->y) {
      return -1;
    }
  }
  //add the neighbor and return "true" if possible
  if(node->num_neighbors < node->capacity) {
    node->neighbors[node->num_neighbors] = new_neighbor;
    node->weights[node->num_neighbors] = weight;
    node->num_neighbors++;
    return 1;
  }
  //return "false" if unable to add the new neighbor
  return 0;
}

int is_visible(struct vis_node *p1, struct vis_node *p2, int num_bzs, struct vis_node ***bzs, int *bz_sizes) {
  float ipx, ipy;
  for(int i = 0; i < num_bzs; i++) {
    for(int j = 0; j < bz_sizes[i]; j++) {
      if (intersect_two_lines_absolute(&ipx, &ipy,
				      p1->x, p1->y, p2->x, p2->y,
				      bzs[i][j]->x, bzs[i][j]->y,
				      bzs[i][(j+1)%(bz_sizes[i])]->x,
				       bzs[i][(j+1)%(bz_sizes[i])]->y)) {
	return 0;
      }
      else {
      }
    }
  }
  return 1;
}

//based solely on the existing visibility graph
int is_visible2(struct vis_node *p1, struct vis_node *p2) {
  float ipx, ipy;
  for(int i = 0; i < num_nfzs; i++) {
    for(int j = 0; j < nfz_sizes[i]; j++) {
      if(intersect_two_lines_absolute(&ipx, &ipy,
			     p1->x, p1->y, p2->x, p2->y,
			     vis_graph_ref[nfz_borders[i][j]]->x, vis_graph_ref[nfz_borders[i][j]]->y,
			     vis_graph_ref[nfz_borders[i][(j+1)%nfz_sizes[i]]]->x,
			     vis_graph_ref[nfz_borders[i][(j+1)%nfz_sizes[i]]]->y)) {
	return 0;
      }
    }
  }
  for(int i = 0; i < NB_NOFLYPOINT; i++) {
    for(int j = 0; j < 8; j++) {
      if(intersect_two_lines_absolute(&ipx, &ipy, p1->x, p1->y, p2->x, p2->y,
				      vis_graph_ref[NB_WAYPOINT + 8*i + j]->x,
				      vis_graph_ref[NB_WAYPOINT + 8*i + j]->y,
				      vis_graph_ref[NB_WAYPOINT + 8*i + (j+1)%8]->x,
				      vis_graph_ref[NB_WAYPOINT + 8*i + (j+1)%8]->y)) {
	return 0;
      }
    }
  }
  return 1;
}

int num_nodes;
struct vis_node **nodes;

bool is_nfz_corner(const int index, int num_nfzs, int **nfzs, int *nfz_sizes) {
  for(int i = 0; i < num_nfzs; i++) {
    for(int j = 0; j < nfz_sizes[i]; j++) {
      if(index == nfzs[i][j]) return true;
    }
  }
  return false;
}

//Remove all edges in the visibility graph and recalculate them. Should be used every time a waypoint is moved.

void reconstruct_visibility_graph() {
  //clear the connections
  for(int i = 1; i < vis_graph_size; i++) {
    vis_graph_ref[i]->num_neighbors = 0;
  }
  //get the indices of the non-NFZ waypoints
  int non_nfz_wp_ct = 0;
  int non_nfz_wps[NB_WAYPOINT-1];
  for(int i = 1; i < vis_graph_size; i++) {
    bool isCorner = is_nfz_corner(i, num_nfzs, nfz_borders, nfz_sizes);
    if(!isCorner) {
      non_nfz_wps[non_nfz_wp_ct] = i;
      non_nfz_wp_ct++;
    }
  }
  //get and connect all the buffer zones
  for(int i = 0; i < num_nfzs; i++) {
    //get the buffer zone for the current NFZ
    coords *bfz = buffer_zone(nfz_sizes[i], nfz_borders[i]);
    //move the existing visibility graph nodes to their new spots, if necessary
    for(int j = 0; j < nfz_sizes[i]; j++) {
      vis_graph_ref[nfz_borders[i][j]]->x = bfz[j][0];
      vis_graph_ref[nfz_borders[i][j]]->y = bfz[j][1];
    }
    //connect each NFZ to itself
    for(int j = 0; j < nfz_sizes[i]; j++) {
      if(add_neighbor(vis_graph_ref[nfz_borders[i][j]], vis_graph_ref[nfz_borders[i][(j+1)%nfz_sizes[i]]], 1) &&
	 add_neighbor(vis_graph_ref[nfz_borders[i][(j+1)%nfz_sizes[i]]], vis_graph_ref[nfz_borders[i][j]], 1)) {
	//successfully connected this vertex with the next; carry on
      }
      else {
	printf("ERROR: Failed to reconnect no-fly zone %d\n", i);
	exit(1);
      }
    }
  }
  //now redo all the no-fly points
  float denom = (float)(1 + sqrt(2));
  for(int i = 0; i < NB_NOFLYPOINT; i++) {
    float x = noflypoints[i].enu_f.x, y = noflypoints[i].enu_f.y, rad = noflypoints[i].radius;
    vis_graph_ref[NB_WAYPOINT + 8*i]->x = x - rad/denom;
    vis_graph_ref[NB_WAYPOINT + 8*i]->y = y + rad;
    vis_graph_ref[NB_WAYPOINT + 8*i + 1]->x = x + rad/denom;
    vis_graph_ref[NB_WAYPOINT + 8*i + 1]->y = y + rad;
    vis_graph_ref[NB_WAYPOINT + 8*i + 2]->x = x + rad;
    vis_graph_ref[NB_WAYPOINT + 8*i + 2]->y = y + rad/denom;
    vis_graph_ref[NB_WAYPOINT + 8*i + 3]->x = x + rad;
    vis_graph_ref[NB_WAYPOINT + 8*i + 3]->y = y - rad/denom;
    vis_graph_ref[NB_WAYPOINT + 8*i + 4]->x = x + rad/denom;
    vis_graph_ref[NB_WAYPOINT + 8*i + 4]->y = y - rad;
    vis_graph_ref[NB_WAYPOINT + 8*i + 5]->x = x - rad/denom;
    vis_graph_ref[NB_WAYPOINT + 8*i + 5]->y = y - rad;
    vis_graph_ref[NB_WAYPOINT + 8*i + 6]->x = x - rad;
    vis_graph_ref[NB_WAYPOINT + 8*i + 6]->y = y - rad/denom;
    vis_graph_ref[NB_WAYPOINT + 8*i + 7]->x = x - rad;
    vis_graph_ref[NB_WAYPOINT + 8*i + 7]->y = y + rad/denom;
    for(int j = 0; j < 8; j++) {
      const int index = NB_WAYPOINT + 8*i + j;
      const int next_pt = NB_WAYPOINT + 8*i + (j+1)%8;
      add_neighbor(vis_graph_ref[index], vis_graph_ref[next_pt], 1);
      add_neighbor(vis_graph_ref[next_pt], vis_graph_ref[index], 1);
    }
  }
  //connect all the no-fly zones to each other
  for(int i = 0; i < NB_NOFLYPOINT; i++) {
    for(int j = 0; j < 8; j++) {
      const int index = NB_WAYPOINT + 8*i + j;
      if(is_visible2(vis_graph_ref[index], HOME_NODE)) {
	add_neighbor(HOME_NODE, vis_graph_ref[index], 0);
	add_neighbor(vis_graph_ref[index], HOME_NODE, 0);
      }
      for(int k = i+1; k < NB_NOFLYPOINT; k++) {
	for(int l = 0; l < 8; l++) {
	  const int index2 = NB_WAYPOINT + 8*k + l;
	  if(is_visible2(vis_graph_ref[index], vis_graph_ref[index2])) {
	    add_neighbor(vis_graph_ref[index], vis_graph_ref[index2], 0);
	    add_neighbor(vis_graph_ref[index2], vis_graph_ref[index], 0);
	  }
	}
      }
    }
  }
  for(int i = 0; i < num_nfzs; i++) {
    for(int j = 0; j < nfz_sizes[i]; j++) {
      //connect to home wp
      if(is_visible2(HOME_NODE, vis_graph_ref[nfz_borders[i][j]])) {
	add_neighbor(HOME_NODE, vis_graph_ref[nfz_borders[i][j]], 0);
	add_neighbor(vis_graph_ref[nfz_borders[i][j]], HOME_NODE, 0);
      }
      for(int k = 0; k < num_nfzs; k++) {
	if(k == i) continue;
	for(int l = 0; l < nfz_sizes[k]; l++) {
	  //connect to other NFZ
	  if(is_visible2(vis_graph_ref[nfz_borders[i][j]], vis_graph_ref[nfz_borders[k][l]])) {
	    add_neighbor(vis_graph_ref[nfz_borders[i][j]], vis_graph_ref[nfz_borders[k][l]], 0);
	  }
	}
      }
      for(int k = 0; k < NB_NOFLYPOINT; k++) {
	for(int l = 0; l < 8; l++) {
	  const int index = NB_WAYPOINT + 8*k + l;
	  if(is_visible2(vis_graph_ref[index], vis_graph_ref[nfz_borders[i][j]])) {
	    add_neighbor(vis_graph_ref[index], vis_graph_ref[nfz_borders[i][j]], 0);
	    add_neighbor(vis_graph_ref[nfz_borders[i][i]], vis_graph_ref[index], 0);
	  }
	}
      }
    }
  }
  //reconnect all the non-NFZ waypoints
  for(int i = 0; i < non_nfz_wp_ct; i++) {
    if(WP_HOME == non_nfz_wps[i]) continue;
    //connect to home
    if(is_visible2(vis_graph_ref[WP_HOME], vis_graph_ref[non_nfz_wps[i]])) {
      add_neighbor(vis_graph_ref[WP_HOME], vis_graph_ref[non_nfz_wps[i]], 0);
      add_neighbor(vis_graph_ref[non_nfz_wps[i]], vis_graph_ref[WP_HOME], 0);
    }
    //connect to the no-fly zones
    for(int j = 0; j < num_nfzs; j++) {
      for(int k = 0; k < nfz_sizes[j]; k++) {
	if(is_visible2(vis_graph_ref[non_nfz_wps[i]], vis_graph_ref[nfz_borders[j][k]])) {
	  add_neighbor(vis_graph_ref[non_nfz_wps[i]], vis_graph_ref[nfz_borders[j][k]], 0);
	  add_neighbor(vis_graph_ref[nfz_borders[j][k]], vis_graph_ref[non_nfz_wps[i]], 0);
	}
      }
    }
    for(int j = 0; j < NB_NOFLYPOINT; j++) {
      for(int k = 0; k < 8; k++) {
	const int index = NB_WAYPOINT + 8*j + k;
	if(is_visible2(vis_graph_ref[index], vis_graph_ref[non_nfz_wps[i]])) {
	  add_neighbor(vis_graph_ref[index], vis_graph_ref[non_nfz_wps[i]], 0);
	  add_neighbor(vis_graph_ref[non_nfz_wps[i]], vis_graph_ref[index], 0);
	}
      }
    }
    //connect to the other non-NFZ waypoints
    for(int j = 0; j < i; j++) {
      if(WP_HOME == j) continue;
      if(is_visible2(vis_graph_ref[non_nfz_wps[i]], vis_graph_ref[non_nfz_wps[j]])) {
	add_neighbor(vis_graph_ref[non_nfz_wps[i]], vis_graph_ref[non_nfz_wps[j]], 0);
	add_neighbor(vis_graph_ref[non_nfz_wps[j]], vis_graph_ref[non_nfz_wps[i]], 0);
      }
    }
  }
}

//Free the visibility graph.
void free_visibility_graph(struct vis_node *home) {
  home->status = VISITING;
  for(int i = 0; i < home->num_neighbors; i++) {
    if(UNVISITED == home->neighbors[i]->status) {
      free_visibility_graph(home->neighbors[i]);
    }
  }
  free(home);
}

//Create the visibility graph.
struct vis_node *create_visibility_graph() {
  int **nfzs = nfz_borders;
  //Before even creating any points, need to filter the waypoints
  int non_nfz_wp_ct = 0;
  int non_nfz_wps[NB_WAYPOINT-1];
  //note: there is an invisible dummy waypoint at index 0 that might get in the way
  for(int i = 1; i < NB_WAYPOINT; i++) {
    bool isCorner = is_nfz_corner(i, num_nfzs, nfzs, nfz_sizes);
    if(!isCorner) {
      non_nfz_wps[non_nfz_wp_ct] = i;
      non_nfz_wp_ct++;
    }
  }
  //first, add home - need a reference point
  struct vis_node *home = init_vis_node((float)waypoints[WP_HOME].enu_i.x, (float)waypoints[WP_HOME].enu_i.y, vis_graph_size-1);
  vis_graph_ref[WP_HOME] = home;
  struct vis_node ***buffer_zones = (struct vis_node***)calloc(num_nfzs, sizeof(struct vis_node**));
  //now, create a representation of each no-fly zone's buffer zone
  for(int i = 0; i < num_nfzs; i++) {
    coords *bfz = buffer_zone(nfz_sizes[i], nfzs[i]);
    buffer_zones[i] = (struct vis_node**)calloc(nfz_sizes[i], sizeof(struct vis_node*));
    //create the nodes for the buffer zone vertices
    for(int j = 0; j < nfz_sizes[i]; j++) {
      buffer_zones[i][j] = init_vis_node(bfz[j][0], bfz[j][1], vis_graph_size - nfz_sizes[i] + 2);
      vis_graph_ref[nfzs[i][j]] = buffer_zones[i][j];
    }
    //connect them
    for(int j = 0; j < nfz_sizes[i]; j++) {
      if(add_neighbor(buffer_zones[i][j], buffer_zones[i][(j+1)%(nfz_sizes[i])], 1) &&
	 add_neighbor(buffer_zones[i][(j+1)%(nfz_sizes[i])], buffer_zones[i][j], 1)) {
	//successfully connected this vertex with the next; carry on
      }
      else {
	printf("ERROR: Failed to connect no-fly zone %d\n", i);
	return NULL;
      }
    }
  }
  //Now create no-fly zones for the no-fly points
  float denom = (float)(1 + sqrt(2));
  for(int i = 0; i < NB_NOFLYPOINT; i++) {
    struct vis_node **cur_nfz = (struct vis_node **)calloc(8, sizeof(struct vis_node*));
    int rad = noflypoints[i].radius;
    float x = (float)noflypoints[i].enu_i.x, y = (float)noflypoints[i].enu_i.y;
    cur_nfz[0] = init_vis_node(x - rad/denom, y + rad, vis_graph_size - 1);
    cur_nfz[1] = init_vis_node(x + rad/denom, y + rad, vis_graph_size - 1);
    cur_nfz[2] = init_vis_node(x + rad, y + rad/denom, vis_graph_size - 1);
    cur_nfz[3] = init_vis_node(x + rad, y - rad/denom, vis_graph_size - 1);
    cur_nfz[4] = init_vis_node(x + rad/denom, y - rad, vis_graph_size - 1);
    cur_nfz[5] = init_vis_node(x - rad/denom, y - rad, vis_graph_size - 1);
    cur_nfz[6] = init_vis_node(x - rad, y - rad/denom, vis_graph_size - 1);
    cur_nfz[7] = init_vis_node(x - rad, y + rad/denom, vis_graph_size - 1);
    for(int j = 0; j < 8; j++) {
      vis_graph_ref[NB_WAYPOINT + 8*i + j] = cur_nfz[j];
      add_neighbor(cur_nfz[i], cur_nfz[(i+1)%8], 1);
      add_neighbor(cur_nfz[(i+1)%8], cur_nfz[i], 1);
    }
    free(cur_nfz);
  }
  //now, connect all the no-fly zones to each other
  for(int i = 0; i < num_nfzs; i++) {
    for(int j = 0; j < nfz_sizes[i]; j++) {
      for(int k = 0; k < num_nfzs; k++) {
	if(k == i) continue;
	for(int l = 0; l < nfz_sizes[k]; l++) {
	  if(is_visible2(buffer_zones[i][j], buffer_zones[k][l])) {
	    //don't need to add both ways here since it'll be done by the loops
	    add_neighbor(buffer_zones[i][j], buffer_zones[k][l], 0);
	  }
	}
      }
      for(int k = 0; k < NB_NOFLYPOINT; k++) {
	for(int l = 0; l < 8; l++) {
	  if(is_visible2(buffer_zones[i][j], vis_graph_ref[NB_WAYPOINT + 8*k + l])) {
	    add_neighbor(buffer_zones[i][j], vis_graph_ref[NB_WAYPOINT + 8*k + l], 0);
	    add_neighbor(vis_graph_ref[NB_WAYPOINT + 8*k + l], buffer_zones[i][j], 0);
	  }
	}
      }
    }
  }
  for(int i = 0; i < NB_NOFLYPOINT; i++) {
    for(int j = 0; j < 8; j++) {
      const int index = NB_WAYPOINT + 8*i + j;
      if(is_visible2(home, vis_graph_ref[index])) {
	add_neighbor(home, vis_graph_ref[index], 0);
	add_neighbor(vis_graph_ref[index], home, 0);
      }
      for(int k = i+1; k < NB_NOFLYPOINT; k++) {
	for(int l = 0; l < 8; l++) {
	  const int index2 = NB_WAYPOINT + 8*k + l;
	  if(is_visible2(vis_graph_ref[index], vis_graph_ref[index2])) {
	    add_neighbor(vis_graph_ref[index], vis_graph_ref[index2], 0);
	    add_neighbor(vis_graph_ref[index2], vis_graph_ref[index], 0);
	  }
	}
      }
    }
  }
  for(int i = 0; i < num_nfzs; i++) {
    for(int j = 0; j < nfz_sizes[i]; j++) {
      if(is_visible2(home, buffer_zones[i][j])) {
	add_neighbor(buffer_zones[i][j], home, 0);
	add_neighbor(home, buffer_zones[i][j], 0);
      }
    }
  }
  //finally, add all the other waypoints to the graph
  struct vis_node **wp_nodes = (struct vis_node**)calloc(non_nfz_wp_ct, sizeof(struct vis_node*));
  for(int i = 0; i < non_nfz_wp_ct; i++) {
    struct EnuCoor_i cur_wp = waypoints[non_nfz_wps[i]].enu_i;
    //if it's home, skip
    if(WP_HOME == non_nfz_wps[i]) {
      continue;
    }
    wp_nodes[i] = init_vis_node((float)cur_wp.x, (float)cur_wp.y, vis_graph_size-2);
    vis_graph_ref[non_nfz_wps[i]] = wp_nodes[i];
    //connect to home if appropriate
    if(is_visible2(home, wp_nodes[i])) {
      add_neighbor(home, wp_nodes[i], 0);
      add_neighbor(wp_nodes[i], home, 0);
    }
    //next, check against all the buffer zones
    for(int j = 0; j < num_nfzs; j++) {
      for(int k = 0; k < nfz_sizes[j]; k++) {
	if(is_visible2(wp_nodes[i], buffer_zones[j][k])) {
	  add_neighbor(wp_nodes[i], buffer_zones[j][k], 0);
	  add_neighbor(buffer_zones[j][k], wp_nodes[i], 0);
	}
      }
    }
    for(int j = 0; j < NB_NOFLYPOINT; j++) {
      for(int k = 0; k < 8; k++) {
	const int index = NB_WAYPOINT + 8*j + k;
	if(is_visible2(vis_graph_ref[index], wp_nodes[i])) {
	  add_neighbor(vis_graph_ref[index], wp_nodes[i], 0);
	  add_neighbor(wp_nodes[i], vis_graph_ref[index], 0);
	}
      }
    }
    //have to check against all other waypoints too
    for(int j = 0; j < i; j++) {
      struct EnuCoor_f added_wp = waypoints[non_nfz_wps[j]].enu_f;
      //if it's home, skip
      if((added_wp.x == waypoints[WP_HOME].enu_f.x) && (added_wp.y == waypoints[WP_HOME].enu_f.y)) {
	continue;
      }
      if(is_visible(wp_nodes[i], wp_nodes[j], num_nfzs, buffer_zones, nfz_sizes)) {
	add_neighbor(wp_nodes[i], wp_nodes[j], 0);
	add_neighbor(wp_nodes[j], wp_nodes[i], 0);
      }
    }
  }
  //print_visibility_graph(home, 0);
  return home;
}

//resetting visit status, depth-first
void reset_visit_statuses(struct vis_node *home) {
  home->status = VISITING;
  for(int i = 0; i < home->num_neighbors; i++) {
    if(VISITED == home->neighbors[i]->status) {
      reset_visit_statuses(home->neighbors[i]);
    }
    //this line should not be necessary but it seems to be
    if(VISITING == home->neighbors[i]->status) home->neighbors[i]->status = UNVISITED;
  }
  home->status = UNVISITED;
}

//depth-first traversal
void print_visibility_graph(struct vis_node *home, int depth) {
  home->status = VISITING;
  if(0 == home->num_neighbors) {
    printf("Depth %d: Node %d has no neighbors.\n", depth, home->node_id);
  }
  else if(1 == home->num_neighbors) {
    printf("Depth %d: Node %d has one neighbor: %d(%.1f, %.1f)\n", depth, home->node_id, home->neighbors[0]->node_id, home->neighbors[0]->x, home->neighbors[0]->y);
  }
  else {
    printf("Depth %d: Node %d (%.1f, %.1f) has %d neighbors: ", depth, home->node_id, home->x, home->y, home->num_neighbors);
    for(int i = 0; i < home->num_neighbors; i++) {
      printf("%d(%.1f, %.1f)%s", home->neighbors[i]->node_id, home->neighbors[i]->x, home->neighbors[i]->y, ((home->num_neighbors == i+1)?("\n"):(", ")));
    }
  }
  for(int i = 0; i < home->num_neighbors; i++) {
    if(UNVISITED == home->neighbors[i]->status) {
      print_visibility_graph(home->neighbors[i], depth+1);
    }
  }
  home->status = VISITED;
  //make sure the state changes back so we can traverse the graph again
  if(0 == depth) {
    reset_visit_statuses(home);
  }
}

struct vis_node *best_neighbor(struct vis_node *current, struct vis_node *dest) {
  float min_dist = FLT_MAX;
  struct vis_node *ret = NULL;
  for(int i = 0; i < current->num_neighbors; i++) {
    if(UNVISITED == current->neighbors[i]->status) {
      float cur_dist = sqrt(DistanceSquare(current->x, current->y, current->neighbors[i]->x, current->neighbors[i]->y)) + sqrt(DistanceSquare(dest->x, dest->y, current->neighbors[i]->x, current->neighbors[i]->y));
      if(min_dist > cur_dist) {
	min_dist = cur_dist;
	ret = current->neighbors[i];
      }
    }
  }
  if(NULL == ret) {
    printf("All neighbors have been visited??\n");
  }
  return ret;
}

struct vis_node *best_neighbor_xy(struct vis_node *current, float dest_x, float dest_y) {
  float min_dist = FLT_MAX;
  struct vis_node *ret = NULL;
  for(int i = 0; i < current->num_neighbors; i++) {
    float cur_dist = sqrt(DistanceSquare(current->x, current->y, current->neighbors[i]->x, current->neighbors[i]->y)) + sqrt(DistanceSquare(dest_x, dest_y, current->neighbors[i]->x, current->neighbors[i]->y));
    if(min_dist > cur_dist) {
      min_dist = cur_dist;
      ret = current->neighbors[i];
    }
  }
  return ret;
}

struct vis_node *closest_node_helper(struct vis_node *home, float target_x, float target_y, float closest_distsq_so_far) {
  home->status = VISITING;
  if(home->num_neighbors < 1) return home;
  if((home->x == target_x) && (home->y == target_y)) return home;
  struct vis_node *closest = NULL;
  float xdist = home->x - target_x, ydist = home->y - target_y;
  float home_distsq = xdist*xdist + ydist*ydist;
  if(home_distsq < closest_distsq_so_far) {
    closest = home;
    closest_distsq_so_far = home_distsq;
  }
  for(int i = 0; i < home->num_neighbors; i++) {
    if(UNVISITED == home->neighbors[i]->status) {
      struct vis_node *temp = closest_node_helper(home->neighbors[i], target_x, target_y, closest_distsq_so_far);
      xdist = temp->x - target_x;
      ydist = temp->y - target_y;
      float cur_distsq = xdist*xdist + ydist*ydist;
      if(cur_distsq < closest_distsq_so_far) {
	closest = temp;
	closest_distsq_so_far = cur_distsq;
      }
    }
  }
  home->status = VISITED;
  if(NULL == closest) closest = home;
  return closest;
}

struct vis_node *closest_node(struct vis_node *home, float target_x, float target_y) {
  reset_visit_statuses(home);
  if(home->x == target_x && home->y == target_y) {
    return home;
  }
  struct vis_node *closest = closest_node_helper(home, target_x, target_y, MAX_DIST_FROM_HOME*MAX_DIST_FROM_HOME*4);
  reset_visit_statuses(home);
  return closest;
}

void prune(struct path_node *start) {
  for(struct path_node *cur = start; NULL != cur && NULL != cur->next && NULL != cur->next->next;) {
    bool can_skip = false;
    for(int i = 0; i < cur->wp->num_neighbors; i++) {
      if(cur->next->next->wp == cur->wp->neighbors[i]) {
	can_skip = true;
	break;
      }
    }
    if(can_skip) {
      struct path_node *temp = cur->next;
      cur->next = cur->next->next;
      free(temp);
    }
    else {
      cur = cur->next;
    }
  }
}

struct path_node *greedy_path(struct vis_node * const start, struct vis_node * const target) {
  reset_visit_statuses(start);
  struct path_node *first_wp = init_path_node(start, NULL);
  struct path_node *cur = first_wp;
  while(target != cur->wp) {
    cur->wp->status = VISITED;
    struct vis_node *next = best_neighbor(cur->wp, target);
    cur = init_path_node(next, cur);
  }
  prune(first_wp);
  reset_visit_statuses(start);
  return first_wp;
}

struct path_node *extend_greedy_path(struct path_node *path, struct vis_node *const target) {
  struct path_node *last_dest = path;
  while(NULL != last_dest && NULL != last_dest->next) {
    last_dest = last_dest->next;
  }
  struct path_node *cur = greedy_path(last_dest->wp, target);
  last_dest->next = cur->next;
  free(cur);
  reset_visit_statuses(path->wp);
  return path;
}

struct path_node *greedy_path_xy(struct vis_node *const home, float start_x, float start_y, float end_x, float end_y) {
  struct vis_node *start_node = closest_node(home, start_x, start_y),
    *end_node = closest_node(home, end_x, end_y);
  return greedy_path(start_node, end_node);
}



struct path_node *astar_path(struct vis_node *const start, struct vis_node *const target) {
  //printf("Using A* to find the shortest path from (%.1f, %.1f) to (%.1f, %.1f)\n", start->x, start->y, target->x, target->y);
  struct astar_node *head = init_first_astar_node(start, target);
  struct astar_node *goal = NULL;
  struct astar_node *considered = NULL;
  while(NULL != head) {
    struct astar_node *node = astar_pop(&head);
    if(node->wp == target) {
      //printf("Reached the goal: (%.1f, %.1f)\n", node->wp->x, node->wp->y);
      goal = node;
      break;
    }
    struct vis_node *wp = node->wp;
    //printf("Now considering (%.1f, %.1f)\n", wp->x, wp->y);
    for(int i = 0; i < wp->num_neighbors; i++) {
      struct vis_node *neighbor = wp->neighbors[i];
      float dist_so_far = node->dist_so_far + sqrt(DistanceSquare(wp->x, wp->y, neighbor->x, neighbor->y));
      struct astar_node *prospective = init_astar_node(node, neighbor, target, dist_so_far);
      head = astar_insert(head, prospective);
    }
    considered = astar_insert(considered, node);
  }
  if(NULL == goal) {
    printf("Null goal\n");
    return NULL;
  }
  struct path_node *first = NULL;//init_path_node(goal->wp, NULL);
  for(struct astar_node *pt = goal; NULL != pt; pt = pt->parent) {
    //printf("prepending (%.1f, %.1f)\n", pt->wp->x, pt->wp->y);
    struct path_node *tmp = init_path_node(pt->wp, NULL);
    tmp->next = first;
    first = tmp;
  }
  //printf("Finished calculating the path\n");
  //print_path(first);
  //astar_free(&head);
  astar_free(considered);
  astar_free(head);
  free(goal);
  return first;
}

struct path_node *extend_astar_path(struct path_node *path, struct vis_node *const target) {
  //printf("Extending the path to (%.1f, %.1f)\n", target->x, target->y);
  struct path_node *last_dest = path;
  while(NULL != last_dest && NULL != last_dest->next) {
    last_dest = last_dest->next;
  }
  struct path_node *cur = astar_path(last_dest->wp, target);
  last_dest->next = cur->next;
  free(cur);
  return path;
}

struct path_node *astar_path_xy(struct vis_node *const home, float start_x, float start_y, float end_x, float end_y) {
  struct vis_node *start_node = closest_node(home, start_x, start_y),
    *end_node = closest_node(home, end_x, end_y);
  return astar_path(start_node, end_node);
}

void print_path(struct path_node *start) {
  printf("Calculated path:");
  for(struct path_node *cur = start; NULL != cur; cur = cur->next) {
    printf(" %d(%.1f, %.1f)", cur->wp->node_id, cur->wp->x, cur->wp->y);
  }
  printf("\n");
}

bool nav_path(struct path_node *start_node) {
  if(NULL == start_node) {
    printf("Start node is null, somehow\n");
    return false;
  }
  struct EnuCoor_i start_coords, end_coords;
  if(start_node) {
    start_coords.x = (int)(start_node->wp->x);
    start_coords.y = (int)(start_node->wp->y);
    start_coords.z = (int)(NAV_DEFAULT_ALT);
    if(start_node->next) {
      end_coords.x = (int)(start_node->next->wp->x);
      end_coords.y = (int)(start_node->next->wp->y);
      end_coords.z = (int)(NAV_DEFAULT_ALT);
      VECT3_COPY(navigation_target, end_coords);
      dist2_to_wp = get_dist2_to_point(&end_coords);
      nav_route(&start_coords, &end_coords);
    }
    else {
      return true;
    }
  }
  if(nav_approaching_from(&end_coords, NULL, 0)) {
    return true;
  }
  return false;
}

void free_path(struct path_node *start_node) {
  if(start_node) {
    free_path(start_node->next);
    start_node->next = NULL;
    free(start_node);
  }
}



/**
 * END FOR NO FLY ZONES
 **/
//#endif
