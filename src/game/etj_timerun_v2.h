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

#pragma once

#include <chrono>

#include "etj_database_v2.h"
#include "etj_log.h"
#include "etj_synchronization_context.h"
#include "etj_time_utilities.h"
#include "etj_utilities.h"

namespace ETJump {
class TimerunV2 {
public:
  TimerunV2(std::unique_ptr<Log> logger,
            std::unique_ptr<SynchronizationContext>
            synchronizationContext);

  struct Options {
    std::string path;
  };

  void initialize(const Options &options);
  void shutdown();
  void runFrame();

  struct AddSeasonParams {
    int clientNum;
    std::string name;
    Time startTime;
    opt<Time> endTime;
  };

  void addSeason(AddSeasonParams season);

private:
  std::unique_ptr<DatabaseV2> _database;
  std::unique_ptr<Log> _logger;
  std::unique_ptr<SynchronizationContext> _sc;
};
}
