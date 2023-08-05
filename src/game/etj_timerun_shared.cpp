#include "etj_timerun_shared.h"

#include "etj_container_utilities.h"

namespace ETJump {
int TimerunCommands::parseClientNum(const std::string &arg) {
  try {
    const auto clientNum = std::stoi(arg);

    if (clientNum < 0 || clientNum >= MAX_CLIENTS) {
      return INVALID_CLIENT_NUM;
    }

    return clientNum;
  } catch (const std::runtime_error &) {
    return INVALID_CLIENT_NUM;
  }
}

opt<int> TimerunCommands::parseTime(const std::string &arg) {
  try {
    const auto time = std::stoi(arg);

    if (time < 0) {
      return opt<int>();
    }

    return time;
  } catch (const std::runtime_error &) {
    return opt<int>();
  }
}

opt<int> TimerunCommands::parseInteger(const std::string &arg) {
  try {
    return std::stoi(arg);
  } catch (const std::runtime_error &) {
    return opt<int>();
  }
}

TimerunCommands::Start::Start() = default;

TimerunCommands::Start::Start(int clientNum, int startTime,
                              const std::string &runName,
                              const opt<int> &previousRecord,
                              std::array<int, MAX_TIMERUN_CHECKPOINTS>
                              checkpoints)
  : clientNum(clientNum),
    startTime(startTime),
    runName(runName),
    previousRecord(previousRecord), checkpoints(checkpoints) {
}

std::string TimerunCommands::Start::serialize() {
  return stringFormat(
      "timerun start %d %d \"%s\" %d \"%s\"", clientNum, startTime, runName,
      previousRecord.hasValue() ? previousRecord.value() : -1,
      StringUtil::join(checkpoints, ","));
}

opt<TimerunCommands::Start>
TimerunCommands::Start::deserialize(
    const std::vector<std::string> &args) {
  const int numExpectedFields = 7;

  if (args.size() < numExpectedFields) {
    return opt<Start>();
  }

  if (args[0] != "timerun") {
    return opt<Start>();
  }

  if (args[1] != "start") {
    return opt<Start>();
  }

  Start start;

  start.clientNum = parseClientNum(args[2]);
  if (start.clientNum == INVALID_CLIENT_NUM) {
    return opt<Start>();
  }

  auto startTime = parseTime(args[3]);
  if (!startTime.hasValue()) {
    return opt<Start>();
  }

  start.startTime = startTime.value();

  start.runName = args[4];

  start.previousRecord = parseTime(args[5]);

  unsigned idx = 0;
  for (const auto &v : Container::map(StringUtil::split(args[6], ","),
                                      [](auto c) { return std::stoi(c); })) {
    if (idx >= start.checkpoints.size()) {
      break;
    }

    start.checkpoints[idx] = v;

    idx++;
  }

  return start;
}

std::string TimerunCommands::Checkpoint::serialize() {
  return stringFormat("timerun checkpoint %d %d %d \"%s\"", clientNum,
                      checkpointIndex, checkpointTime, runName);
}

opt<TimerunCommands::Checkpoint> TimerunCommands::Checkpoint::deserialize(
    const std::vector<std::string> &args) {
  auto empty = opt<Checkpoint>();
  const int expectedFields = 6;
  if (args.size() < expectedFields) {
    return empty;
  }

  if (args[0] != "timerun") {
    return empty;
  }

  if (args[1] != "checkpoint") {
    return empty;
  }

  Checkpoint cp;

  cp.clientNum = parseClientNum(args[2]);
  if (cp.clientNum == INVALID_CLIENT_NUM) {
    return empty;
  }

  auto cpi = parseInteger(args[3]);
  if (!cpi.hasValue()) {
    return empty;
  }
  cp.checkpointIndex = cpi.value();

  auto time = parseTime(args[4]);
  if (!time.hasValue()) {
    return empty;
  }
  cp.checkpointTime = time.value();
  cp.runName = args[5];

  return cp;
}

std::string TimerunCommands::Interrupt::serialize() {
  return stringFormat("timerun interrupt %d", clientNum);
}

opt<TimerunCommands::Interrupt> TimerunCommands::Interrupt::deserialize(
    const std::vector<std::string> &args) {
  auto empty = opt<Interrupt>();

  if (args[0] != "timerun" || args[1] != "interrupt") {
    return empty;
  }

  Interrupt interrupt{};
  interrupt.clientNum = parseClientNum(args[2]);
  if (interrupt.clientNum == INVALID_CLIENT_NUM) {
    return empty;
  }

  return interrupt;
}

std::string TimerunCommands::Completion::serialize() {
  return stringFormat("timerun completion %d %d %d \"%s\"", clientNum,
                      completionTime,
                      previousRecordTime.valueOr(NO_PREVIOUS_RECORD),
                      runName);
}

opt<TimerunCommands::Completion> TimerunCommands::Completion::deserialize(
    const std::vector<std::string> &args) {
  auto empty = opt<Completion>();

  if (args[0] != "timerun" || args[1] != "completion") {
    return empty;
  }

  Completion completion{};
  completion.clientNum = parseClientNum(args[2]);
  if (completion.clientNum == INVALID_CLIENT_NUM) {
    return empty;
  }

  auto completionTime = parseTime(args[3]);
  if (!completionTime.hasValue()) {
    return empty;
  }

  completion.completionTime = completionTime.value();

  completion.previousRecordTime = parseTime(args[4]);

  completion.runName = args[5];

  return completion;

}

std::string TimerunCommands::Record::serialize() {
  return stringFormat("timerun record %d %d %d \"%s\"", clientNum,
                      completionTime,
                      previousRecordTime.valueOr(NO_PREVIOUS_RECORD),
                      runName);
}

opt<TimerunCommands::Record>
TimerunCommands::Record::deserialize(
    const std::vector<std::string> &args) {
  auto empty = opt<Record>();

  if (args[0] != "timerun" || args[1] != "record") {
    return empty;
  }

  Record record{};
  record.clientNum = parseClientNum(args[2]);
  if (record.clientNum == INVALID_CLIENT_NUM) {
    return empty;
  }

  auto completionTime = parseTime(args[3]);
  if (!completionTime.hasValue()) {
    return empty;
  }

  record.completionTime = completionTime.value();

  record.previousRecordTime = parseTime(args[4]);

  record.runName = args[5];

  return record;
}

std::string TimerunCommands::Stop::serialize() {
  return stringFormat("timerun stop %d %d \"%s\"", clientNum, completionTime,
                      runName);
}

opt<TimerunCommands::Stop> TimerunCommands::Stop::deserialize(
    const std::vector<std::string> &args) {
  auto empty = opt<Stop>();

  if (args[0] != "timerun" || args[1] != "stop") {
    return empty;
  }

  Stop stop{};
  stop.clientNum = parseClientNum(args[2]);
  if (stop.clientNum == INVALID_CLIENT_NUM) {
    return empty;
  }

  auto completionTime = parseTime(args[3]);
  if (!completionTime.hasValue()) {
    return empty;
  }

  stop.completionTime = completionTime.value();

  stop.runName = args[4];

  return stop;
}

}
