#include "CellularOperatorScanner.h"

#include <vector>

#include "driver/gpio.h"
#include "esp_log.h"

#include "AirgradientUART.h"
#include "MaxConfig.h"
#include "cellularModule.h"
#include "cellularModuleA7672xx.h"

static const char *const TAG = "CellOperatorScanner";

bool CellularOperatorScanner::start() {
  if (_taskHandle.load() != nullptr) {
    return true;
  }

  TaskHandle_t taskHandle = nullptr;
  BaseType_t result = xTaskCreate(_taskEntry, "CellOperatorScan", TASK_STACK_SIZE, this,
                                  TASK_PRIORITY, &taskHandle);
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create cellular operator scan task");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(_mutex);
    _status = CellularOperatorScanStatus();
  }
  _taskHandle.store(taskHandle);
  return true;
}

void CellularOperatorScanner::stop() {
  TaskHandle_t taskHandle = _taskHandle.load();
  if (taskHandle == nullptr) {
    return;
  }

  xTaskNotify(taskHandle, STOP_REQUEST_BIT, eSetBits);
  for (int retry = 0; retry < 100 && _taskHandle.load() != nullptr; retry++) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (_taskHandle.load() != nullptr) {
    ESP_LOGW(TAG, "Cellular operator scan task did not stop");
  }
}

bool CellularOperatorScanner::requestScan() {
  TaskHandle_t taskHandle = _taskHandle.load();
  if (taskHandle == nullptr) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_status.state == CellularOperatorScanState::Scanning) {
      return false;
    }
    _status.state = CellularOperatorScanState::Scanning;
    _status.error.clear();
    _status.operators.clear();
  }

  if (xTaskNotify(taskHandle, SCAN_REQUEST_BIT, eSetBits) != pdPASS) {
    std::lock_guard<std::mutex> lock(_mutex);
    _status.state = CellularOperatorScanState::Failed;
    _status.error = "scan_task_notification_failed";
    return false;
  }

  return true;
}

CellularOperatorScanStatus CellularOperatorScanner::getStatus() {
  std::lock_guard<std::mutex> lock(_mutex);
  return _status;
}

void CellularOperatorScanner::_taskEntry(void *arg) {
  CellularOperatorScanner *scanner = static_cast<CellularOperatorScanner *>(arg);
  scanner->_task();
}

void CellularOperatorScanner::_task() {
  while (1) {
    uint32_t notification = 0;
    xTaskNotifyWait(0, UINT32_MAX, &notification, portMAX_DELAY);
    if ((notification & STOP_REQUEST_BIT) != 0) {
      break;
    }
    if ((notification & SCAN_REQUEST_BIT) == 0) {
      continue;
    }

    CellularOperatorScanStatus status = _performScan();
    {
      std::lock_guard<std::mutex> lock(_mutex);
      _status = status;
    }
  }

  _taskHandle.store(nullptr);
  vTaskDelete(nullptr);
}

CellularOperatorScanStatus CellularOperatorScanner::_performScan() {
  CellularOperatorScanStatus status;
  status.state = CellularOperatorScanState::Failed;

  gpio_set_level(EN_CE_CARD, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  AirgradientUART serial;
  bool serialStarted =
      serial.begin(UART_BAUD_PORT_CE_CARD, UART_BAUD_CE_CARD, UART_RX_CE_CARD, UART_TX_CE_CARD);

  if (!serialStarted) {
    status.error = "uart_initialization_failed";
  } else {
    serial.setDebug(true);
    CellularModuleA7672XX cellularModule(&serial, IO_CE_POWER);

    if (!cellularModule.init()) {
      status.error = "cellular_module_initialization_failed";
    } else if (cellularModule.isSimReady() != CellReturnStatus::Ok) {
      status.error = "sim_not_ready";
    } else if (cellularModule.prepareOperatorScan(CellTechnology::LTE) != CellReturnStatus::Ok) {
      status.error = "cellular_module_not_ready";
    } else {
      _scanOperators(cellularModule, status);
    }
    cellularModule.powerOff(true);
  }
  if (serialStarted) {
    serial.setDebug(false);
    serial.end();
  }
  gpio_set_level(EN_CE_CARD, 0);

  return status;
}

void CellularOperatorScanner::_scanOperators(CellularModuleA7672XX &cellularModule,
                                             CellularOperatorScanStatus &status) {
  auto scanResult = cellularModule.scanAvailableOperators();
  if (scanResult.status == CellReturnStatus::Error) {
    ESP_LOGW(TAG, "Operator scan failed, retrying once");
    vTaskDelay(pdMS_TO_TICKS(3000));
    scanResult = cellularModule.scanAvailableOperators();
  }

  if (scanResult.status == CellReturnStatus::Ok) {
    status.operators.reserve(scanResult.data.size());
    for (const CellularModule::OperatorRecord &operatorRecord : scanResult.data) {
      CellularOperatorRecord record;
      record.operatorId = operatorRecord.operatorId;
      record.accessTech = operatorRecord.accessTech;
      record.operatorName = operatorRecord.operatorName;
      status.operators.push_back(record);
    }
    status.state = CellularOperatorScanState::Succeeded;
  } else if (scanResult.status == CellReturnStatus::Timeout) {
    status.error = "scan_timeout";
  } else if (scanResult.status == CellReturnStatus::Failed) {
    status.error = "no_operators_found";
  } else {
    status.error = "scan_failed";
  }
}
