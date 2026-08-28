#ifndef CELLULAR_OPERATOR_SCANNER_H
#define CELLULAR_OPERATOR_SCANNER_H

#include <atomic>
#include <cstdint>
#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "WiFiManager.h"

class CellularModuleA7672XX;

class CellularOperatorScanner {
public:
  bool start();
  void stop();
  bool requestScan();
  CellularOperatorScanStatus getStatus();

private:
  static constexpr uint32_t SCAN_REQUEST_BIT = (1 << 0);
  static constexpr uint32_t STOP_REQUEST_BIT = (1 << 1);
  static constexpr uint32_t TASK_STACK_SIZE = 8192;
  static constexpr UBaseType_t TASK_PRIORITY = 5;

  static void _taskEntry(void *arg);
  void _task();
  CellularOperatorScanStatus _performScan();
  void _scanOperators(CellularModuleA7672XX &cellularModule, CellularOperatorScanStatus &status);

  std::atomic<TaskHandle_t> _taskHandle{nullptr};
  std::mutex _mutex;
  CellularOperatorScanStatus _status;
};

#endif // CELLULAR_OPERATOR_SCANNER_H
