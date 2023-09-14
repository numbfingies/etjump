/*
 * MIT License
 *
 * Copyright (c) 2023 ETJump team <zero@etjump.com>
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

#include <string>

#include "cg_local.h"
#include "etj_timerun_view.h"
#include "etj_utilities.h"
#include "etj_cvar_update_handler.h"
#include "etj_player_events_handler.h"
#include "../game/etj_string_utilities.h"
#include "../game/etj_container_utilities.h"
#include "../game/etj_numeric_utilities.h"

ETJump::TimerunView::TimerunView(std::shared_ptr<Timerun> timerun)
    : Drawable(), _timerun(timerun) {
  parseColorString(etj_runTimerInactiveColor.string, inactiveTimerColor);
  cvarUpdateHandler->subscribe(
      &etj_runTimerInactiveColor, [&](const vmCvar_t *cvar) {
        parseColorString(cvar->string, inactiveTimerColor);
      });
}

ETJump::TimerunView::~TimerunView() {}

const ETJump::Timerun::PlayerTimerunInformation *
ETJump::TimerunView::currentRun() const {
  return _timerun->getTimerunInformationFor(cg.snap->ps.clientNum);
}

std::string ETJump::TimerunView::getTimerString(int msec) {
  if (msec < 0) {
    msec *= -1;
  }

  auto millis = msec;
  auto minutes = millis / 60000;
  millis -= minutes * 60000;
  auto seconds = millis / 1000;
  millis -= seconds * 1000;

  return stringFormat("%02d:%02d.%03d", minutes, seconds, millis);
}

void ETJump::TimerunView::draw() {
  if (canSkipDraw()) {
    return;
  }

  auto run = currentRun();
  auto hasTimerun = ((cg.demoPlayback && (run->lastRunTimer || run->running)) ||
                     cg.hasTimerun);

  if (etj_drawRunTimer.integer == 0 || !hasTimerun) {
    return;
  }

  auto startTime = run->startTime;
  auto millis = 0;
  auto color = &colorWhite;
  const auto font = &cgs.media.limboFont1;

  // ensure correct 8ms interval timer when playing
  // specs/demo playback get approximation from cg.time, so timer stays smooth
  // one day this can maybe be real commandTime for all scenarios
  // if we get to sv_fps 125 servers...
  const int timeVar = (isPlaying(cg.clientNum) && !cg.demoPlayback)
                          ? cg.predictedPlayerState.commandTime
                          : cg.time;

  if (run->running) {
    millis = timeVar - startTime;
  } else {
    millis = run->completionTime;
    if (millis == -1) {
      millis = 0;
    }
    color = &inactiveTimerColor;
  }

  vec4_t colorTemp;
  vec4_t colorWhite = {1.0f, 1.0f, 1.0f, 1.0f};
  vec4_t colorSuccess = {0.627f, 0.941f, 0.349f, 1.0f};
  vec4_t colorFail = {0.976f, 0.262f, 0.262f, 1.0f};

  auto range = getTransitionRange(run->previousRecord);
  auto style = ITEM_TEXTSTYLE_NORMAL;
  auto fadeOut = 2000;   // 2s fade out
  auto fadeStart = 5000; // 5s pause

  if (etj_runTimerShadow.integer > 0) {
    style = ITEM_TEXTSTYLE_SHADOWED;
  }

  if (run->previousRecord > 0) {
    if (millis > run->previousRecord) {
      color = &colorFail;
    }
    // add timer color transition when player gets closer to his
    // pb
    else if (millis + range >= run->previousRecord) {
      auto start = run->previousRecord - range;
      auto step = (millis - start) / (float)(run->previousRecord - start);

      ETJump_LerpColors(&colorWhite, &colorFail, &colorTemp, step / 2);
      color = &colorTemp;
    }
  }

  // set green color for pb time
  if (!run->running && millis &&
      (run->previousRecord > millis || run->previousRecord == -1)) {
    color = &colorSuccess;
  }

  auto ms = millis;
  auto minutes = millis / 60000;
  millis -= minutes * 60000;
  auto seconds = millis / 1000;
  millis -= seconds * 1000;

  auto text = ETJump::stringFormat("%02d:%02d.%03d", minutes, seconds, millis);

  float textWidth = CG_Text_Width_Ext(text.c_str(), 0.3, 0, font) / 2;
  auto x = etj_runTimerX.value;
  auto y = etj_runTimerY.value;

  // timer fading/hiding routine
  ETJump_AdjustPosition(&x);

  bool hideCheckpoints = false;

  if (!run->running && etj_runTimerAutoHide.integer) {
    auto fstart = run->lastRunTimer + fadeStart;
    auto fend = fstart + fadeOut;

    if (fstart < timeVar && fend > timeVar) {
      vec4_t toColor;
      memcpy(&toColor, color, sizeof(toColor));
      toColor[3] = 0;

      auto step = (timeVar - fstart) / (float)(fend - fstart);

      ETJump_LerpColors(color, &toColor, &colorTemp, step);
      color = &colorTemp;

      hideCheckpoints = true;
    } else if (timeVar > fend) {
      // dont draw timer once fading is done
      return;
    }
  }

  CG_Text_Paint_Ext(x - textWidth, y, 0.3, 0.3, *color, text.c_str(), 0, 0,
                    style, font);

  // FIXME: hack, cba dealing with handcrafted fading
  // for checkpoints.
  if (hideCheckpoints) {
    return;
  }

  if (etj_drawCheckpoints.integer && run->runHasCheckpoints) {
    // only adjust x/y if we're drawing checkpoints detached from runtimer
    if (etj_drawCheckpoints.integer == 2) {
      x = etj_checkpointsX.value;
      y = etj_checkpointsY.value;
      ETJump_AdjustPosition(&x);
    } else {
      // position the times below runtimer
      y += 20;
    }
    const int currentTime =
        run->running ? timeVar - run->startTime : run->completionTime;
    const float textSize = 0.1f * etj_checkpointsSize.value;
    const auto textStyle = etj_checkpointsShadow.integer
                               ? ITEM_TEXTSTYLE_SHADOWED
                               : ITEM_TEXTSTYLE_NORMAL;

    const int count = Numeric::clamp(etj_checkpointsCount.integer, 1, 5);
    const int startIndex = run->numCheckpointsHit;
    const int endIndex = run->numCheckpointsHit - count;

    // do not render checkpoints if we're not running and
    // did not just complete a run
    if (!run->running && run->completionTime <= 0) {
      return;
    }

    for (int i = startIndex; i >= 0 && i > endIndex; i--) {
      vec4_t *checkpointColor = &colorWhite;
      const int checkpointTime = run->checkpoints[i];
      // Use the previous record as checkpoint time if no new ones
      // are available
      const int recordCheckpointTime =
          run->previousRecordCheckpoints[i] == TIMERUN_CHECKPOINT_NOT_SET
              ? run->previousRecord
              : run->previousRecordCheckpoints[i];
      bool noRecordCheckpoint =
          recordCheckpointTime == TIMERUN_CHECKPOINT_NOT_SET;

      bool checkpointTimeNotSet = checkpointTime == TIMERUN_CHECKPOINT_NOT_SET;

      // make sure we don't subtract -1 from timestamp if we don't
      // have a checkpoint time set at all
      int relativeTime;
      if (checkpointTimeNotSet) {
        relativeTime =
            currentTime - (noRecordCheckpoint ? 0 : recordCheckpointTime);
      } else {
        relativeTime =
            checkpointTime - (noRecordCheckpoint ? 0 : recordCheckpointTime);
      }

      // if we don't have current or next checkpoint set, just display the
      // runtimer
      if (checkpointTimeNotSet &&
          recordCheckpointTime == TIMERUN_CHECKPOINT_NOT_SET) {
        relativeTime = currentTime;
      }

      std::string dir = "";
      // if we don't have a next checkpoint set, we can't compare to anything
      // so render checkpoint as white
      if (recordCheckpointTime == TIMERUN_CHECKPOINT_NOT_SET ||
          relativeTime == 0) {
        checkpointColor = &colorWhite;
      } else if ((checkpointTimeNotSet && currentTime < recordCheckpointTime) ||
                 (!checkpointTimeNotSet &&
                  checkpointTime < recordCheckpointTime)) {
        checkpointColor = &colorSuccess;
        dir = "-";
      } else {
        checkpointColor = &colorFail;
        dir = "+";
      }

      auto absoluteTime = checkpointTimeNotSet ? currentTime : checkpointTime;

      std::string timerStr = getTimerString(
          etj_checkpointsStyle.integer == 1 ? absoluteTime : relativeTime);

      timerStr = dir + timerStr;

      textWidth =
          static_cast<float>(CG_Text_Width_Ext(timerStr, textSize, 0, font)) *
          0.5f;

      CG_Text_Paint_Ext(x - textWidth, y, textSize, textSize, *checkpointColor,
                        timerStr.c_str(), 0, 0, textStyle, font);
      y += 15;
    }
  }

  if (run->running) {

    if (run->previousRecord != -1 && ms > run->previousRecord) {
      pastRecordAnimation(color, text.c_str(), ms, run->previousRecord);
    }
  }
}

int ETJump::TimerunView::getTransitionRange(int previousRunTime) {
  auto range = 10000;

  if (3 * 1000 > previousRunTime) {
    range = 0;
  } else if (10 * 1000 > previousRunTime) {
    range = 500; // just for a nice short transition effect,
    // could be 0
  } else if (30 * 1000 > previousRunTime) {
    range = 2000;
  } else if (60 * 1000 > previousRunTime) {
    range = 3500;
  } else if (120 * 1000 > previousRunTime) {
    range = 5000;
  }

  return range;
}

void ETJump::TimerunView::pastRecordAnimation(vec4_t *color, const char *text,
                                              int timerTime, int record) {
  auto animationTime = 300;

  if (timerTime - record > animationTime) {
    return;
  }

  vec4_t toColor;
  vec4_t incolor;

  auto x = etj_runTimerX.value;
  auto y = etj_runTimerY.value;

  ETJump_AdjustPosition(&x);

  auto step = ((float)(timerTime - record) / animationTime);
  auto scale = 0.3 + 0.25 * step;

  auto originalTextHeight =
      CG_Text_Height_Ext(text, 0.3, 0, &cgs.media.limboFont1);
  auto textWidth = CG_Text_Width_Ext(text, scale, 0, &cgs.media.limboFont1) / 2;
  auto textHeight = (CG_Text_Height_Ext(text, scale, 0, &cgs.media.limboFont1) -
                     originalTextHeight) /
                    2;

  memcpy(&toColor, color, sizeof(toColor));
  toColor[3] = 0;

  ETJump_LerpColors(color, &toColor, &incolor, step);

  CG_Text_Paint_Ext(x - textWidth, y + textHeight, scale, scale, incolor, text,
                    0, 0, 0, &cgs.media.limboFont1);
}

bool ETJump::TimerunView::canSkipDraw() const { return showingScores(); }
