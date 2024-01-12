// Author(s):         Chansol Hong, Taeyoung Kim

#include <ode/ode.h>
#include <plugins/physics.h>
#include <stdbool.h>

#define MAX_CONTACTS 10
#define ROBOT_COUNT 1

static pthread_mutex_t mutex;

const char robot_name[2][1][12] = {
  {"DEF_ROBOTR0"},
  {"DEF_ROBOTB0"}
};
const char head_name[2][1][22] = {
  {"DEF_ROBOTR0.HEADSHAPE"},
  {"DEF_ROBOTB0.HEADSHAPE"}
};

// plugin variables
static dGeomID robot_geom[2][1] = {{NULL}, {NULL}};
static dGeomID ball_geom = NULL;
static dGeomID robot_ceiling_geom = NULL;

bool robot_collision[2][1] = {{false}, {false}};

int step = 0;
void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL);

  // get ODE handles to .wbt objects
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < ROBOT_COUNT; j++) {
      robot_geom[i][j] = dWebotsGetGeomFromDEF(robot_name[i][j]);
      if (robot_geom[i][j] == NULL)
        dWebotsConsolePrintf("%s is missing.", robot_name[i][j]);
    }
  }
  ball_geom = dWebotsGetGeomFromDEF("DEF_BALL");
  robot_ceiling_geom = dWebotsGetGeomFromDEF("ROBOTCEILING");
  if (robot_ceiling_geom == NULL)
    dWebotsConsolePrintf("Robot ceiling is missing");
}

void webots_physics_step() {
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < ROBOT_COUNT; j++)
      robot_collision[i][j] = false;
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  pthread_mutex_lock(&mutex);
  if (dAreGeomsSame(g1, ball_geom)) {
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < ROBOT_COUNT; j++) {
        if ((dAreGeomsSame(g2, robot_geom[i][j]))) {
          robot_collision[i][j] = true;
        }
      }
    }

    if (dAreGeomsSame(g2, robot_ceiling_geom)) {
      pthread_mutex_unlock(&mutex);
      return 1;
    }
  }
  else if (dAreGeomsSame(g2, ball_geom)) {
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < ROBOT_COUNT; j++) {
        if ((dAreGeomsSame(g1, robot_geom[i][j]))) {
          robot_collision[i][j] = true;
        }
      }
    }

    if (dAreGeomsSame(g1, robot_ceiling_geom)) {
      pthread_mutex_unlock(&mutex);
      return 1;
    }
  }
  pthread_mutex_unlock(&mutex);

  return 0;
}

void webots_physics_step_end() {
  char collision_packet[2*ROBOT_COUNT];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < ROBOT_COUNT; j++) {
      collision_packet[i+2*j] = (char)robot_collision[i][j];
    }
  }

  dWebotsSend(0, collision_packet, sizeof(char)*2*ROBOT_COUNT);
}

void webots_physics_cleanup() {
   pthread_mutex_destroy(&mutex);
}
