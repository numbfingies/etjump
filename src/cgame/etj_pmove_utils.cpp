/*
 * MIT License
 *
 * Copyright (c) 2024 ETJump team <zero@etjump.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "etj_pmove_utils.h"
#include "../game/bg_local.h"

// etj_pmove_utils.cpp - helper functions for pmove related calculations

namespace ETJump {
static pmove_t pmove;
static pmoveExt_t pmext;
static playerState_t temp_ps;

usercmd_t PmoveUtils::getUserCmd(const playerState_t &ps, int8_t uCmdScale) {
  usercmd_t cmd{};

  if (!cg.demoPlayback && !(cg.snap->ps.pm_flags & PMF_FOLLOW)) {
    int32_t const cmdNum = trap_GetCurrentCmdNumber();
    trap_GetUserCmd(cmdNum, &cmd);
  }
  // generate fake userCmd for following spectators and demos
  else {
    cmd.forwardmove = static_cast<signed char>(
        uCmdScale * (!!(ps.stats[STAT_USERCMD_MOVE] & UMOVE_FORWARD) -
                     !!(ps.stats[STAT_USERCMD_MOVE] & UMOVE_BACKWARD)));
    cmd.rightmove = static_cast<signed char>(
        uCmdScale * (!!(ps.stats[STAT_USERCMD_MOVE] & UMOVE_RIGHT) -
                     !!(ps.stats[STAT_USERCMD_MOVE] & UMOVE_LEFT)));
    cmd.upmove = static_cast<signed char>(
        uCmdScale * (!!(ps.stats[STAT_USERCMD_MOVE] & UMOVE_UP) -
                     !!(ps.stats[STAT_USERCMD_MOVE] & UMOVE_DOWN)));

    // store buttons too, so we get correct scale when sprint is held
    cmd.buttons = ps.stats[STAT_USERCMD_BUTTONS] >> 8;
    cmd.wbuttons = ps.stats[STAT_USERCMD_BUTTONS] & 0xff;

    // generate correct angles
    for (auto i = 0; i < 3; i++) {
      cmd.angles[i] = ANGLE2SHORT(ps.viewangles[i]) - ps.delta_angles[i];
    }

    cmd.serverTime = cg.snap->serverTime;
  }
  return cmd;
}

pmove_t *PmoveUtils::getPmove(usercmd_t cmd) {
  if (cg.snap->ps.clientNum == cg.clientNum && !cg.demoPlayback) {
    cg_pmove.pmext = &cg.pmext;
    return &cg_pmove;
  }

  temp_ps = cg.predictedPlayerState;
  pmove.ps = &temp_ps;
  pmove.pmext = &pmext;
  pmove.character =
      CG_CharacterForClientinfo(&cgs.clientinfo[cg.snap->ps.clientNum],
                                &cg_entities[cg.snap->ps.clientNum]);
  pmove.trace = CG_TraceCapsule;
  pmove.tracemask = cg.snap->ps.pm_type == PM_DEAD
                        ? MASK_PLAYERSOLID & ~CONTENTS_BODY
                        : MASK_PLAYERSOLID;
  pmove.pointcontents = CG_PointContents;
  pmove.skill = cgs.clientinfo[cg.snap->ps.clientNum].skill;
  pmove.cmd = cmd;
  pmove.pmove_msec = cgs.pmove_msec;
  PmoveSingle(&pmove);
  return &pmove;
}

float PmoveUtils::PM_SprintScale(const playerState_t *ps) {
  // based on PM_CmdScale from bg_pmove.c
  float scale = ps->stats[STAT_USERCMD_BUTTONS] & (BUTTON_SPRINT << 8) &&
                        cg.pmext.sprintTime > 50
                    ? ps->sprintSpeedScale
                    : ps->runSpeedScale;
  return scale;
}

static void PM_Accelerate(vec3_t vel, vec3_t wishdir, float wishspeed,
                          float accel) {
  // q2 style
  int i;
  float addspeed, accelspeed, currentspeed;

  currentspeed = DotProduct(vel, wishdir);
  addspeed = wishspeed - currentspeed;
  if (addspeed <= 0) {
    return;
  }
  accelspeed = accel * pml.frametime * wishspeed;
  if (accelspeed > addspeed) {
    accelspeed = addspeed;
  }

  // Ridah, variable friction for AI's
  if (pm->ps->groundEntityNum != ENTITYNUM_NONE) {
    accelspeed *= (1.0 / pm->ps->friction);
  }
  if (accelspeed > addspeed) {
    accelspeed = addspeed;
  }

  for (i = 0; i < 3; i++) {
    vel[i] += accelspeed * wishdir[i];
  }
}

float PmoveUtils::PM_GetWishspeed(vec3_t wishvel, float scale, usercmd_t cmd,
                                  vec3_t forward, vec3_t right, vec3_t up,
                                  const playerState_t &ps, pmove_t *pm) {
  PM_UpdateWishvel(wishvel, cmd, forward, right, up, ps);

  //float wishspeed = scale * VectorLength2(wishvel);

  vec3_t wishdir{};
  VectorCopy(wishvel, wishdir);

  float wishspeed = VectorNormalize(wishdir);
  wishspeed *= scale;

  // if walking, account for prone, crouch and water
  if (pm->pmext->walking) {
    // clamp the speed lower if prone
    if (pm->ps->eFlags & EF_PRONE) {
      if (wishspeed > pm->ps->speed * pm_proneSpeedScale) {
        wishspeed = pm->ps->speed * pm_proneSpeedScale;
      }
    }
    // clamp the speed lower if ducking
    else if (pm->ps->pm_flags & PMF_DUCKED) {
      if (wishspeed > pm->ps->speed * pm->ps->crouchSpeedScale) {
        wishspeed = pm->ps->speed * pm->ps->crouchSpeedScale;
      }
    }

    // clamp the speed lower if wading or walking on the bottom
    if (pm->pmext->waterlevel) {
      float waterScale = pm->pmext->waterlevel / 3.0f;
      if (pm->watertype == CONTENTS_SLIME) {
        waterScale = 1.0 - (1.0 - pm_slagSwimScale) * waterScale;
      } else {
        waterScale = 1.0 - (1.0 - pm_waterSwimScale) * waterScale;
      }

      if (wishspeed > pm->ps->speed * waterScale) {
        wishspeed = pm->ps->speed * waterScale;
      }
    }
  }

  return wishspeed;
}

void PmoveUtils::PM_UpdateWishvel(vec3_t wishvel, usercmd_t cmd, vec3_t forward,
                                  vec3_t right, vec3_t up,
                                  const playerState_t &ps) {
  AngleVectors(ps.viewangles, forward, right, up);

  // project moves down to flat plane
  forward[2] = 0;
  right[2] = 0;
  VectorNormalize(forward);
  VectorNormalize(right);

  for (uint8_t i = 0; i < 2; ++i) {
    wishvel[i] = cmd.forwardmove * forward[i] + cmd.rightmove * right[i];
  }
  wishvel[2] = 0;
}

float PmoveUtils::PM_GetGroundWalkWishspeed(vec3_t wishvel, float scale,
                                            usercmd_t cmd, vec3_t forward,
                                            vec3_t right, vec3_t up,
                                            float yaw,
                                            pmove_t *pm, bool trueness) {
  PM_UpdateGroundWalkWishvel(wishvel, cmd, forward, right, up, yaw);

  vec3_t wishdir;
  VectorCopy(wishvel, wishdir);

  float wishspeed = VectorNormalize(wishdir);
  wishspeed *= scale;

  // clamp the speed lower if prone
  if (pm->ps->eFlags & EF_PRONE) {
    if (wishspeed > pm->ps->speed * pm_proneSpeedScale) {
      wishspeed = pm->ps->speed * pm_proneSpeedScale;
    }
  } else if (pm->ps->pm_flags &
             PMF_DUCKED) { // clamp the speed lower if ducking
    /*if ( wishspeed > pm->ps->speed * pm_duckScale ) {
            wishspeed = pm->ps->speed * pm_duckScale;
    }*/
    if (wishspeed > pm->ps->speed * pm->ps->crouchSpeedScale) {
      wishspeed = pm->ps->speed * pm->ps->crouchSpeedScale;
    }
  }

  // clamp the speed lower if wading or walking on the bottom
  if (pm->waterlevel) {
    float waterScale;

    waterScale = pm->waterlevel / 3.0;
    if (pm->watertype == CONTENTS_SLIME) { //----(SA)	slag
      waterScale = 1.0 - (1.0 - pm_slagSwimScale) * waterScale;
    } else {
      waterScale = 1.0 - (1.0 - pm_waterSwimScale) * waterScale;
    }

    if (wishspeed > pm->ps->speed * waterScale) {
      wishspeed = pm->ps->speed * waterScale;
    }
  }

  return wishspeed;
}
//
//float PmoveUtils::PM_GetPredictedVelocityxy(const float yaw, pmove_t *pm,
//                                          bool trueness) {
//  vec3_t wishvel;
//  vec3_t wishdir;
//  vec3_t forward, right, up;
//  float wishspeed;
//  float scale;
//  usercmd_t cmd;
//
//  cmd = pm->cmd;
//
//  if (trueness) {
//    scale = pm->pmext->scale;
//
//    if (pm->cmd.buttons & BUTTON_SPRINT && pm->pmext->sprintTime > 50) {
//      scale *= pm->ps->sprintSpeedScale;
//    } else {
//      scale *= pm->ps->runSpeedScale;
//    }
//  } else {
//    scale =pm->pmext->scaleAlt;
//    scale *= pm->ps->sprintSpeedScale;
//  }
//
//  PM_UpdateGroundWalkWishvel(wishvel, cmd, forward, right, up, yaw);
//  VectorCopy(wishvel, wishdir);
//
//  wishspeed = VectorNormalize(wishdir);
//  wishspeed *= scale;
//
//  if (trueness) {
//    // clamp the speed lower if prone
//    if (pm->ps->eFlags & EF_PRONE) {
//      if (wishspeed > pm->ps->speed * pm_proneSpeedScale) {
//        wishspeed = pm->ps->speed * pm_proneSpeedScale;
//      }
//    } else if (pm->ps->pm_flags &
//               PMF_DUCKED) { // clamp the speed lower if ducking
//      /*if ( wishspeed > pm->ps->speed * pm_duckScale ) {
//              wishspeed = pm->ps->speed * pm_duckScale;
//      }*/
//      if (wishspeed > pm->ps->speed * pm->ps->crouchSpeedScale) {
//        wishspeed = pm->ps->speed * pm->ps->crouchSpeedScale;
//      }
//    }
//
//    // clamp the speed lower if wading or walking on the bottom
//    if (pm->waterlevel) {
//      float waterScale;
//
//      waterScale = pm->waterlevel / 3.0;
//      if (pm->watertype == CONTENTS_SLIME) { //----(SA)	slag
//        waterScale = 1.0 - (1.0 - pm_slagSwimScale) * waterScale;
//      } else {
//        waterScale = 1.0 - (1.0 - pm_waterSwimScale) * waterScale;
//      }
//
//      if (wishspeed > pm->ps->speed * waterScale) {
//        wishspeed = pm->ps->speed * waterScale;
//      }
//    }
//  }
//
//  vec3_t predictedVelocity;
//  VectorCopy(pm->ps->velocity, predictedVelocity);
//
//  PM_Accelerate(predictedVelocity, wishdir, wishspeed, pm_airaccelerate);
//
//  // gravity influence
//  pm->ps->velocity[2] -= pm->ps->gravity * pml.frametime;
//
//  float vel = VectorLength(predictedVelocity);
//
//  // slide along the ground plane
//  PM_ClipVelocity(predictedVelocity, pm->pmext->groundTrace.plane.normal,
//                  predictedVelocity, OVERCLIP);
//
//  // don't decrease velocity when going up or down a slope
//  VectorNormalize(predictedVelocity);
//  VectorScale(predictedVelocity, vel, predictedVelocity);
//
//  return VectorLength2(predictedVelocity);
//}

void PmoveUtils::PM_UpdateGroundWalkWishvel(vec3_t wishvel, usercmd_t cmd,
                                            vec3_t forward,
                                  vec3_t right, vec3_t up,
                                  const float yaw) {
  const vec3_t angles{0.f, yaw, 0.f};
  AngleVectors(angles, forward, right, up);

  // project the forward and right directions onto the ground plane
  PM_ClipVelocity(forward, pm->pmext->groundTrace.plane.normal, forward,
                  OVERCLIP);
  PM_ClipVelocity(right, pm->pmext->groundTrace.plane.normal, right, OVERCLIP);
  

  VectorNormalize(forward);
  VectorNormalize(right);

  for (uint8_t i = 0; i < 3; i++) {
    wishvel[i] = forward[i] * cmd.forwardmove + right[i] * cmd.rightmove;
  }
}

float PmoveUtils::getFrameAccel(const playerState_t &ps, pmove_t *pm) {
  const auto ucmdScale =
      static_cast<int8_t>(ps.stats[STAT_USERCMD_BUTTONS] & (BUTTON_WALKING << 8)
                              ? CMDSCALE_WALK
                              : CMDSCALE_DEFAULT);
  const usercmd_t cmd = PmoveUtils::getUserCmd(ps, ucmdScale);

  // no meaningful value if no user input
  if (cmd.forwardmove == 0 && cmd.rightmove == 0) {
    return 0;
  }

  vec3_t wishvel;
  const float wishspeed = PmoveUtils::PM_GetWishspeed(
      wishvel, pm->pmext->scale, cmd, pm->pmext->forward, pm->pmext->right,
      pm->pmext->up, ps, pm);

  float wishAccel = pm->pmext->accel * wishspeed * pm->pmext->frametime;

  return wishAccel;
}

} // namespace ETJump
