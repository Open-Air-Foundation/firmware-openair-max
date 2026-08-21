/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef AG_SENSOR_H
#define AG_SENSOR_H

#include "AirgradientIICSerial.h"
#include "AirgradientUART.h"
#include "BQ25672.h"
#include "PMS.h"
#include "SPS30.h"
#include "Sunlight.h"
#include "airgradientClient.h"
#include "sht4x.h"
#include "sgp4x.h"
#include "AlphaSenseSensor.h"
#include "Configuration.h"

struct MaxSensorPayload {
  AirgradientClient::CommonPayload common;
  AirgradientClient::ExtraPayload extra;
};

class Sensor {
public:
  Sensor(i2c_master_bus_handle_t busHandle);
  ~Sensor() {}
  bool init(Configuration::Model model, int co2ABCDays);
  bool startMeasures(int iterations, int intervalMs);
  void printMeasures();
  MaxSensorPayload getLastAverageMeasure();
  bool co2AttemptManualCalibration();
  float batteryVoltage();

private:
  enum class PMSensorType { NONE, PMS5003, SPS30 };

  struct PMData {
    float pm01Ae;
    float pm25Ae;
    float pm10Ae;
    float pm25Sp;
    int particleCount003;
    int particleCount005;
    int particleCount01;
    int particleCount02;
    int particleCount50;
    int particleCount10;
  };

  const char *const TAG = "Sensor";

  void _measure(int iteration, MaxSensorPayload &data);
  void _applyIteration(MaxSensorPayload &data);
  void _calculateMeasuresAverage();
  void _warmUpSensor();
  bool _applySunlightMeasurementSample();
  bool _initPMChannel(int ch, AirgradientSerial *serial, PMS *&pms, SPS30 *&sps,
                      PMSensorType &type);
  bool _readPMData(int ch, PMS *pms, SPS30 *sps, PMSensorType type, PMData &data);
  void _printPMData(int ch, PMSensorType type, const PMData &data);
  static float _averagePMValue(float first, float second);
  static int _averagePMValue(int first, int second);

  int _rco2IterationOkCount = 0;
  int _atmpIterationOkCount = 0;
  int _rhumIterationOkCount = 0;
  int _pm01IterationOkCount = 0;
  int _pm25IterationOkCount[2] = {0, 0};
  int _pm10IterationOkCount = 0;
  int _pm25SpIterationOkCount[2] = {0, 0};
  int _pm003CountIterationOkCount[2] = {0, 0};
  int _pm005CountIterationOkCount = 0;
  int _pm01CountIterationOkCount = 0;
  int _pm02CountIterationOkCount = 0;
  int _pm50CountIterationOkCount = 0;
  int _pm10CountIterationOkCount = 0;
  int _tvocIterationOkCount = 0;
  int _noxIterationOkCount = 0;
  int _vbatIterationOkCount = 0;
  int _vpanelIterationOkCount = 0;
  int _o3WEIterationOkCount = 0;
  int _o3AEIterationOkCount = 0;
  int _no2WEIterationOkCount = 0;
  int _no2AEIterationOkCount = 0;
  int _afeTempIterationOkCount = 0;
  MaxSensorPayload _averageMeasure;
  i2c_master_bus_handle_t _busHandle;

  bool _co2Available = true;
  AirgradientSerial *agsCO2_ = nullptr;
  Sunlight *co2_ = nullptr;
  bool _co2ReadTriggered = false;

  bool _pm1Available = true;
  AirgradientSerial *agsPM1_ = nullptr;
  PMS *pms1_ = nullptr;
  SPS30 *sps1_ = nullptr;
  PMSensorType _pm1Type = PMSensorType::NONE;

  bool _pm2Available = true;
  AirgradientSerial *agsPM2_ = nullptr;
  PMS *pms2_ = nullptr;
  SPS30 *sps2_ = nullptr;
  PMSensorType _pm2Type = PMSensorType::NONE;

  bool _chargerAvailable = true;
  BQ25672 *charger_ = nullptr;

  bool _tempHumAvailable = true;
  sht4x_handle_t sht_dev_hdl = NULL;

  bool _tvocNoxAvailable = true;
  sgp4x_handle_t sgp_dev_hdl = NULL;

  bool _alphaSenseGasAvailable = true;
  bool _alphaSenseTempAvailable = true;
  AlphaSenseSensor *alphaSense_ = nullptr;
};

#endif // !AG_SENSOR_H
