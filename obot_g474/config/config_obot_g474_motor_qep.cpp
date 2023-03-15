#include "../../motorlib/gpio.h"
#include "../../motorlib/qep_encoder.h"
#include "../../motorlib/torque_sensor.h"
#include "../param/param_obot_g474.h"
#include "../st_device.h"

using MotorEncoder = QEPEncoder;
using OutputEncoder = EncoderBase;
using TorqueSensor = TorqueSensorBase;

extern "C" void SystemClock_Config();
void pin_config_obot_g474_motor_r0();

struct InitCode {
  InitCode() {
    SystemClock_Config();
    pin_config_obot_g474_motor_r0();
  }
};

namespace config {
const uint32_t main_loop_frequency = 10000;
const uint32_t pwm_frequency = 50000;
InitCode init_code;

MotorEncoder motor_encoder(*TIM2);
OutputEncoder output_encoder;
TorqueSensor torque_sensor;
};  // namespace config

#include "../../motorlib/boards/config_obot_g474_motor.cpp"

void config_init() {
  System::api.add_api_variable(
      "index_count", new APIUint32(&config::motor_encoder.index_count_));
}

void config_maintenance() {}