#include <memory>

#include "../../motorlib/gpio.h"
#include "../../motorlib/hall.h"
#include "../../motorlib/peripheral/stm32g4/pin_config.h"
#include "../../motorlib/torque_sensor.h"
#include "../param/param_obot_g474.h"
#include "../st_device.h"

extern "C" void SystemClock_Config();
void pin_config_obot_g474_motor_r0();

struct InitCode {
  InitCode() {
    SystemClock_Config();
    pin_config_obot_g474_motor_r0();

    // Configure pins through registers to allow reading halls.
    GPIO_SETL(A, 0, GPIO::INPUT, GPIO_SPEED::VERY_HIGH, 0);
    GPIO_SETL(A, 1, GPIO::INPUT, GPIO_SPEED::VERY_HIGH, 0);
    GPIO_SETL(A, 2, GPIO::INPUT, GPIO_SPEED::VERY_HIGH, 0);
  }
};

namespace config {
const uint32_t main_loop_frequency = 10000;
const uint32_t pwm_frequency = 50000;
InitCode init_code;

GPIO gpio_a(*GPIOA, 0);
GPIO gpio_b(*GPIOA, 1);
GPIO gpio_c(*GPIOA, 2);
HallEncoder motor_encoder(gpio_a, gpio_b, gpio_c);
TorqueSensorBase torque_sensor;
EncoderBase output_encoder;
};  // namespace config

// =======================================================
// | BEGIN | #include "../../motorlib/boards/config_obot_g474_motor.cpp"
// =======================================================

#include "../../motorlib/peripheral/stm32g4/drv8323s.h"
#include "../../motorlib/peripheral/stm32g4/hrpwm.h"
#include "../../motorlib/peripheral/stm32g4/pin_config.h"
#include "../../motorlib/peripheral/usb.h"
#include "../../motorlib/usb_communication.h"
#include "../../motorlib/util.h"

volatile uint32_t *const cpu_clock = &DWT->CYCCNT;
uint16_t drv_regs_error = 0;

#ifndef GPIO_OUT
#define GPIO_OUT (reinterpret_cast<volatile gpio_bits *>(&GPIOA->ODR)->bit1)
#endif

#ifndef GPIO_IN
#define GPIO_IN ((GPIOA->IDR & (1 << 2)) ? 1 : 0)
#endif

#include "../../motorlib/actuator.h"
#include "../../motorlib/boards/pin_config_obot_g474_motor.h"
#include "../../motorlib/controller/impedance_controller.h"
#include "../../motorlib/controller/joint_position_controller.h"
#include "../../motorlib/controller/position_controller.h"
#include "../../motorlib/controller/state_controller.h"
#include "../../motorlib/controller/torque_controller.h"
#include "../../motorlib/controller/velocity_controller.h"
#include "../../motorlib/fast_loop.h"
#include "../../motorlib/led.h"
#include "../../motorlib/main_loop.h"
#include "../../motorlib/peripheral/stm32g4/temp_sensor.h"
#include "../../motorlib/system.h"

#if defined(R3) || defined(R4) || defined(MR0) || defined(MR0P)
#define HAS_MAX31875
#include "../peripheral/stm32g4/max31875.h"
#endif

namespace config {
static_assert(((double)CPU_FREQUENCY_HZ * 8 / 2) / pwm_frequency <
              65535);  // check pwm frequency
auto logger = std::make_unique<Logger>();
auto round_robin_logger = std::make_unique<RoundRobinLogger>();
#ifdef SPI1_REINIT_CALLBACK
DRV8323S drv(*SPI1, *logger, spi1_dma.register_operation_,
             spi1_reinit_callback);
#else
DRV8323S drv(*SPI1, *logger);
#endif
TempSensor temp_sensor;
#ifdef HAS_MAX31875
I2C i2c1(*I2C1, 1000);
MAX31875 board_temperature(i2c1);
#endif
USB1 usb;
std::unique_ptr<CommunicationBase> communication =
    std::make_unique<USBCommunication>(usb);
HRPWM motor_pwm = {pwm_frequency, *HRTIM1, 3, 5, 4, false, 200, 1000, 0};
FastLoop fast_loop = {(int32_t)pwm_frequency,
                      motor_pwm,
                      motor_encoder,
                      param->fast_loop_param,
                      &I_A_DR,
                      &I_B_DR,
                      &I_C_DR,
                      &V_BUS_DR};
LED led = {
    const_cast<uint16_t *>(reinterpret_cast<volatile uint16_t *>(&TIM_R)),
    const_cast<uint16_t *>(reinterpret_cast<volatile uint16_t *>(&TIM_G)),
    const_cast<uint16_t *>(reinterpret_cast<volatile uint16_t *>(&TIM_B))};
PositionController position_controller = {(float)(1.0 / main_loop_frequency)};
TorqueController torque_controller = {(float)(1.0 / main_loop_frequency)};
ImpedanceController impedance_controller = {(float)(1.0 / main_loop_frequency)};
VelocityController velocity_controller = {(float)(1.0 / main_loop_frequency)};
StateController state_controller = {(float)(1.0 / main_loop_frequency)};
JointPositionController joint_position_controller(1.0 / main_loop_frequency);
MainLoop main_loop = {fast_loop,
                      position_controller,
                      torque_controller,
                      impedance_controller,
                      velocity_controller,
                      state_controller,
                      joint_position_controller,
                      *communication,
                      led,
                      output_encoder,
                      torque_sensor,
                      drv,
                      *logger,
                      *round_robin_logger,
                      param->main_loop_param};
auto actuator =
    std::make_unique<Actuator>(fast_loop, main_loop, param->startup_param);

};  // namespace config

System &sys = System::init(config::actuator, config::communication,
                           config::logger, config::round_robin_logger);

void usb_interrupt() { config::usb.interrupt(); }

float v3v3 = 3.3;
int32_t index_mod = 0;

void config_init();

void system_init() {
  if (config::motor_encoder.init()) {
    sys.log("Motor encoder init success");
  } else {
    sys.log("Motor encoder init failure");
  }
  if (config::output_encoder.init()) {
    sys.log("Output encoder init success");
  } else {
    sys.log("Output encoder init failure");
  }
  if (drv_regs_error) {
    sys.log("drv configure failure");
  } else {
    sys.log("drv configure success");
  }
  if (config::torque_sensor.init()) {
    sys.log("torque sensor init success");
  } else {
    sys.log("torque sensor init failure");
  }

  sys.api_.add_api_variable("3v3", new APIFloat(&v3v3));
  std::function<float()> get_t =
      std::bind(&TempSensor::get_value, &config::temp_sensor);
  std::function<void(float)> set_t = std::bind(
      &TempSensor::set_value, &config::temp_sensor, std::placeholders::_1);
  sys.api_.add_api_variable("T", new APICallbackFloat(get_t, set_t));
#ifdef HAS_MAX31875
  sys.api_.add_api_variable(
      "Tboard", new const APICallbackFloat([]() {
        return config::board_temperature.get_temperature();
      }));
#endif
  sys.api_.add_api_variable("index_mod", new APIInt32(&index_mod));
  sys.api_.add_api_variable("drv_err", new const APICallbackUint32([]() {
                              return config::drv.get_drv_status();
                            }));
  sys.api_.add_api_variable("drv_reset", new const APICallback([]() {
                              return config::drv.drv_reset();
                            }));
  sys.api_.add_api_variable(
      "usb_err",
      new APICallbackUint32(
          []() {
            return static_cast<USBCommunication &>(*sys.communication_)
                .get_error_count();
          },
          [](uint32_t u) {}));
  sys.api_.add_api_variable("A1",
                            new const APICallbackFloat([]() { return A1_DR; }));
  sys.api_.add_api_variable("A2",
                            new const APICallbackFloat([]() { return A2_DR; }));
  sys.api_.add_api_variable("A3",
                            new const APICallbackFloat([]() { return A3_DR; }));
  sys.api_.add_api_variable("shutdown", new const APICallback([]() {
                              // requires power cycle to return
                              setup_sleep();
                              SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
                              PWR->CR1 |= 0b100 << PWR_CR1_LPMS_Pos;
                              __WFI();
                              return "";
                            }));
  sys.api_.add_api_variable(
      "deadtime", new APICallbackUint16(
                      []() { return config::motor_pwm.deadtime_ns_; },
                      [](uint16_t u) { config::motor_pwm.set_deadtime(u); }));

  for (auto regs : std::vector<ADC_TypeDef *>{ADC1, ADC2, ADC3, ADC4, ADC5}) {
    regs->CR = ADC_CR_ADVREGEN;
    ns_delay(20000);
    regs->CR |= ADC_CR_ADCAL;
    while (regs->CR & ADC_CR_ADCAL)
      ;
    ns_delay(100);
    regs->CR |= ADC_CR_ADCALDIF;
    regs->CR |= ADC_CR_ADCAL;
    while (regs->CR & ADC_CR_ADCAL)
      ;
    ns_delay(100);

    regs->ISR = ADC_ISR_ADRDY;
    regs->CR |= ADC_CR_ADEN;
    while (!(regs->ISR & ADC_ISR_ADRDY))
      ;
  }

  ADC1->CR |= ADC_CR_JADSTART;
  while (ADC1->CR & ADC_CR_JADSTART)
    ;

  v3v3 = *((uint16_t *)(0x1FFF75AA)) * 3.0 / V_REF_DR;
  sys.log("3v3: " + std::to_string(v3v3));

  ADC1->GCOMP = v3v3 * 4096;
  ADC1->CFGR2 |= ADC_CFGR2_GCOMP;
  ADC1->CR |= ADC_CR_ADSTART;
  ADC2->CR |= ADC_CR_JADSTART;
  ADC5->CR |= ADC_CR_JADSTART;
  ADC5->IER |= ADC_IER_JEOCIE;
  ADC4->CR |= ADC_CR_JADSTART;
  ADC3->CR |= ADC_CR_JADSTART;

  config_init();

  TIM1->CR1 = TIM_CR1_CEN;  // start main loop interrupt
  config::usb.connect();
  HRTIM1->sMasterRegs.MCR = HRTIM_MCR_TDCEN + HRTIM_MCR_TECEN +
                            HRTIM_MCR_TFCEN;  // start high res timer
}

FrequencyLimiter temp_rate = {10};
float T = 0;

void config_maintenance();
void system_maintenance() {
  static bool driver_fault = false;
  if (temp_rate.run()) {
    ADC1->CR |= ADC_CR_JADSTART;
    while (ADC1->CR & ADC_CR_JADSTART)
      ;
    T = config::temp_sensor.read();
    v3v3 =
        *((uint16_t *)(0x1FFF75AA)) * 3.0 * ADC1->GCOMP / 4096.0 / ADC1->JDR2;
    if (T > 100) {
      config::main_loop.status_.error.microcontroller_temperature = 1;
    }
#ifdef HAS_MAX31875
    config::board_temperature.read();
    if (config::board_temperature.get_temperature() > 100) {
      config::main_loop.status_.error.board_temperature = 1;
    }
#endif
  }
  if (!(GPIOC->IDR & 1 << 14)) {
    driver_fault = true;
  } else if (param->main_loop_param.no_latch_driver_fault) {
    driver_fault = false;
  }
  config::main_loop.status_.error.driver_fault |=
      driver_fault;  // maybe latch driver fault until reset
  index_mod = config::motor_encoder.index_error(
      param->fast_loop_param.motor_encoder.cpr);
  config_maintenance();
}

void setup_sleep() {
  NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  NVIC_DisableIRQ(ADC5_IRQn);
  config::drv.disable();
  NVIC_SetPriority(USB_LP_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
  NVIC_EnableIRQ(RTC_WKUP_IRQn);
  MASK_SET(RCC->CFGR, RCC_CFGR_SW, 2);  // HSE is system clock source
  RTC->SCR = RTC_SCR_CWUTF;
}

void finish_sleep() {
  MASK_SET(RCC->CFGR, RCC_CFGR_SW, 3);  // PLL is system clock source
  config::drv.enable();
  NVIC_DisableIRQ(RTC_WKUP_IRQn);
  NVIC_SetPriority(USB_LP_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  NVIC_EnableIRQ(ADC5_IRQn);
}
// =======================================================
// | END | #include "../../motorlib/boards/config_obot_g474_motor.cpp"
// =======================================================

void config_init() {}

void config_maintenance() {}