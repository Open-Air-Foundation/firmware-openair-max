#ifndef SPS30_H
#define SPS30_H

#include <stddef.h>
#include <stdint.h>

#include "AirgradientSerial.h"

class SPS30 {
public:
  struct Data {
    // Mass concentration (ug/m3)
    float pm_ae_1_0;
    float pm_ae_2_5;
    float pm_ae_4_0;
    float pm_ae_10_0;

    // Number concentration (#/cm3)
    float pm_raw_0_5;
    float pm_raw_1_0;
    float pm_raw_2_5;
    float pm_raw_4_0;
    float pm_raw_10_0;

    float typical_particle_size;
  };

  explicit SPS30(AirgradientSerial *serial);

  bool begin();
  void end();
  bool read(Data &data);
  bool connected() const;

private:
  static constexpr size_t MAX_PAYLOAD_SIZE = 40;
  static constexpr size_t MAX_RESPONSE_SIZE = MAX_PAYLOAD_SIZE + 5;
  static constexpr size_t MAX_TX_FRAME_SIZE = 14;
  // The WK2132 bridge services the response through repeated I2C FIFO reads.
  static constexpr uint32_t RESPONSE_TIMEOUT_US = 250'000;
  static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 3;

  bool _execute_command(uint8_t command, const uint8_t *command_data, size_t command_data_size,
                        uint8_t *response_data, size_t response_capacity,
                        size_t *response_size = nullptr);
  bool _write_frame(uint8_t command, const uint8_t *data, size_t data_size);
  bool _read_frame(uint8_t command, uint8_t *data, size_t data_capacity, size_t &data_size);
  static bool _append_stuffed(uint8_t value, uint8_t *frame, size_t frame_size,
                              size_t &frame_index);
  static float _read_float(const uint8_t *data);

  AirgradientSerial *_serial;
  bool _is_initialized = false;
  bool _is_connected = false;
  uint8_t _consecutive_errors = 0;
};

#endif // SPS30_H
