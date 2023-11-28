/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"
#include "app_config.h"
#include "led_util.h"

#include <platform/CHIPDeviceLayer.h>

#include "board_util.h"
#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/cluster-objects.h>
#include <app/server/OnboardingCodesUtil.h>
#include <app/server/Server.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <lib/support/CHIPMem.h>
#include <lib/support/CodeUtils.h>
#include <system/SystemError.h>

#ifdef CONFIG_CHIP_WIFI
#include <app/clusters/network-commissioning/network-commissioning.h>
#include <platform/nrfconnect/wifi/NrfWiFiDriver.h>
#endif

#ifdef CONFIG_CHIP_OTA_REQUESTOR
#include "ota_util.h"
#endif

#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app, CONFIG_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::Credentials;
using namespace ::chip::DeviceLayer;

namespace
{
constexpr size_t kAppEventQueueSize = 10;
constexpr uint32_t kFactoryResetTriggerTimeout = 6000;

K_MSGQ_DEFINE(sAppEventQueue, sizeof(AppEvent), kAppEventQueueSize, alignof(AppEvent));
k_timer sFunctionTimer;
k_timer sSensorTimer;

LEDWidget sStatusLED;
FactoryResetLEDsWrapper<1> sFactoryResetLEDs{ { FACTORY_RESET_SIGNAL_LED } };

bool sIsNetworkProvisioned = false;
bool sIsNetworkEnabled = false;
bool sHaveBLEConnections = false;

const struct device *sBme680SensorDev = DEVICE_DT_GET_ONE(bosch_bme680);

static const struct gpio_dt_spec load_switch = GPIO_DT_SPEC_GET(DT_NODELABEL(load_switch), gpios);

static const struct pwm_dt_spec pwm_fan = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_fan));
static const uint32_t off_pulse = DT_PROP(DT_NODELABEL(pwm_fan), off_pulse);
static const uint32_t low_pulse = DT_PROP(DT_NODELABEL(pwm_fan), low_pulse);
static const uint32_t mid_pulse = DT_PROP(DT_NODELABEL(pwm_fan), mid_pulse);
static const uint32_t high_pulse = DT_PROP(DT_NODELABEL(pwm_fan), high_pulse);
} /* namespace */

namespace LedConsts
{
namespace StatusLed
{
	namespace Unprovisioned
	{
		constexpr uint32_t kOn_ms{ 100 };
		constexpr uint32_t kOff_ms{ kOn_ms };
	} /* namespace Unprovisioned */
	namespace Provisioned
	{
		constexpr uint32_t kOn_ms{ 50 };
		constexpr uint32_t kOff_ms{ 950 };
	} /* namespace Provisioned */

} /* namespace StatusLed */
} /* namespace LedConsts */

#ifdef CONFIG_CHIP_WIFI
app::Clusters::NetworkCommissioning::Instance
	sWiFiCommissioningInstance(0, &(NetworkCommissioning::NrfWiFiDriver::Instance()));
#endif

CHIP_ERROR AppTask::Init()
{
	/* Initialize CHIP stack */
	LOG_INF("Init CHIP stack");

	CHIP_ERROR err = chip::Platform::MemoryInit();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("Platform::MemoryInit() failed");
		return err;
	}

	err = PlatformMgr().InitChipStack();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("PlatformMgr().InitChipStack() failed");
		return err;
	}

#if defined(CONFIG_NET_L2_OPENTHREAD)
	err = ThreadStackMgr().InitThreadStack();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("ThreadStackMgr().InitThreadStack() failed: %s", ErrorStr(err));
		return err;
	}

#ifdef CONFIG_OPENTHREAD_MTD_SED
	err = ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_SleepyEndDevice);
#elif CONFIG_OPENTHREAD_MTD
	err = ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_MinimalEndDevice);
#else
	err = ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_Router);
#endif /* CONFIG_OPENTHREAD_MTD_SED */
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("ConnectivityMgr().SetThreadDeviceType() failed: %s", ErrorStr(err));
		return err;
	}

#elif defined(CONFIG_CHIP_WIFI)
	sWiFiCommissioningInstance.Init();
#else
	return CHIP_ERROR_INTERNAL;
#endif /* CONFIG_NET_L2_OPENTHREAD */

	/* Initialize LEDs */
	LEDWidget::InitGpio();
	LEDWidget::SetStateUpdateCallback(LEDStateUpdateHandler);

	sStatusLED.Init(SYSTEM_STATE_LED);

	UpdateStatusLED();

	/* Initialize buttons */
	int ret = dk_buttons_init(ButtonEventHandler);
	if (ret) {
		LOG_ERR("dk_buttons_init() failed");
		return chip::System::MapErrorZephyr(ret);
	}

	/* Initialize  BME680 sensor*/
	if (!device_is_ready(sBme680SensorDev)) {
		LOG_ERR("BME680 sensor device not ready");
		return chip::System::MapErrorZephyr(-ENODEV);
	}

	/* Initialiaze LED relay */
	if (!gpio_is_ready_dt(&load_switch)) {
		LOG_ERR("The load switch pin GPIO port is not ready");
		return chip::System::MapErrorZephyr(-ENODEV);
	}

	ret = gpio_pin_configure_dt(&load_switch, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("Configuring GPIO pin failed: %d", ret);
		return chip::System::MapErrorZephyr(-ENODEV);
	}

	/* Initialize PWM fan */
	if (!device_is_ready(pwm_fan.dev)) {
		LOG_ERR("PWM fan device not ready");
		return chip::System::MapErrorZephyr(-ENODEV);
	}

	ret = pwm_set_pulse_dt(&pwm_fan, 0u);

	/* Initialize function timer */
	k_timer_init(&sFunctionTimer, &AppTask::FunctionTimerTimeoutCallback, nullptr);
	k_timer_user_data_set(&sFunctionTimer, this);

	/* Initialize sensor timer */
	k_timer_init(&sSensorTimer, &AppTask::SensorTimerTimeoutCallback, nullptr);
	k_timer_user_data_set(&sSensorTimer, this);
	/* Start sensor timer */
	k_timer_start(&sSensorTimer, K_SECONDS(2), K_SECONDS(5));

	/* Initialize CHIP server */
#if CONFIG_CHIP_FACTORY_DATA
	ReturnErrorOnFailure(mFactoryDataProvider.Init());
	SetDeviceInstanceInfoProvider(&mFactoryDataProvider);
	SetDeviceAttestationCredentialsProvider(&mFactoryDataProvider);
	SetCommissionableDataProvider(&mFactoryDataProvider);
#else
	SetDeviceInstanceInfoProvider(&DeviceInstanceInfoProviderMgrImpl());
	SetDeviceAttestationCredentialsProvider(Examples::GetExampleDACProvider());
#endif

	static chip::CommonCaseDeviceServerInitParams initParams;
	(void)initParams.InitializeStaticResourcesBeforeServerInit();

	ReturnErrorOnFailure(chip::Server::GetInstance().Init(initParams));
	ConfigurationMgr().LogDeviceConfig();
	PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

	/*
	 * Add CHIP event handler and start CHIP thread.
	 * Note that all the initialization code should happen prior to this point to avoid data races
	 * between the main and the CHIP threads.
	 */
	PlatformMgr().AddEventHandler(ChipEventHandler, 0);

	err = PlatformMgr().StartEventLoopTask();
	if (err != CHIP_NO_ERROR) {
		LOG_ERR("PlatformMgr().StartEventLoopTask() failed");
		return err;
	}

	return CHIP_NO_ERROR;
}

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(Init());

	AppEvent event = {};

	while (true) {
		k_msgq_get(&sAppEventQueue, &event, K_FOREVER);
		DispatchEvent(event);
	}

	return CHIP_NO_ERROR;
}

void AppTask::ButtonEventHandler(uint32_t buttonState, uint32_t hasChanged)
{
	AppEvent button_event;
	button_event.Type = AppEventType::Button;

	if (FUNCTION_BUTTON_MASK & hasChanged) {
		button_event.ButtonEvent.PinNo = FUNCTION_BUTTON;
		button_event.ButtonEvent.Action =
			static_cast<uint8_t>((FUNCTION_BUTTON_MASK & buttonState) ? AppEventType::ButtonPushed :
										    AppEventType::ButtonReleased);
		button_event.Handler = FunctionHandler;
		PostEvent(button_event);
	}
}

void AppTask::FunctionTimerTimeoutCallback(k_timer *timer)
{
	if (!timer) {
		return;
	}

	AppEvent event;
	event.Type = AppEventType::Timer;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = FunctionTimerEventHandler;
	PostEvent(event);
}

void AppTask::FunctionTimerEventHandler(const AppEvent &)
{
	if (Instance().mFunction == FunctionEvent::FactoryReset) {
		Instance().mFunction = FunctionEvent::NoneSelected;
		LOG_INF("Factory Reset triggered");

		sStatusLED.Set(true);
		sFactoryResetLEDs.Set(true);

		chip::Server::GetInstance().ScheduleFactoryReset();
	}
}

void AppTask::SensorTimerTimeoutCallback(k_timer *timer)
{
	if (!timer) {
		return;
	}

	AppEvent event;
	event.Type = AppEventType::Timer;
	event.TimerEvent.Context = k_timer_user_data_get(timer);
	event.Handler = SensorMeasureHandler;
	PostEvent(event);
}

void AppTask::SensorMeasureHandler(const AppEvent &)
{
	const int result = sensor_sample_fetch(sBme680SensorDev);
	if (result != 0) {
		LOG_ERR("Fetching data from BME688 sensor failed with: %d", result);
		return;
	}

	UpdateTemperatureClusterState();
	UpdatePressureClusterState();
	UpdateHumidityClusterState();
}

void AppTask::UpdateTemperatureClusterState()
{
	struct sensor_value sTemperature;

	int result = sensor_channel_get(sBme680SensorDev, SENSOR_CHAN_AMBIENT_TEMP, &sTemperature);
	if (result != 0) {
		LOG_ERR("Getting temperature measurement data from BME688 failed with: %d", result);
		return;
	}

	/* Defined by cluster temperature measured value = 100 x temperature in degC with resolution of
	 * 0.01 degC. val1 is an integer part of the value and val2 is fractional part in one-millionth
	 * parts. To achieve resolution of 0.01 degC val2 needs to be divided by 10000. */
	int16_t temperature = static_cast<int16_t>(sTemperature.val1 * 100 + sTemperature.val2 / 10000);
	LOG_DBG("Temperature: %d,%d °C", sTemperature.val1, sTemperature.val2);

	EmberAfStatus status = Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(1, temperature);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating temperature measurement %x", status);
	}
}

void AppTask::UpdatePressureClusterState()
{
	struct sensor_value sPressure;

	int result = sensor_channel_get(sBme680SensorDev, SENSOR_CHAN_PRESS, &sPressure);
	if (result != 0) {
		LOG_ERR("Getting pressure measurement data from BME688 failed with: %d", result);
		return;
	}

	/* Defined by cluster pressure measured value = 10 x pressure in kPa with resolution of 0.1 kPa.
	 * val1 is an integer part of the value and val2 is fractional part in one-millionth parts.
	 * To achieve resolution of 0.1 kPa val2 needs to be divided by 100000. */
	int16_t pressure = static_cast<int16_t>(sPressure.val1 * 10 + sPressure.val2 / 100000);
	LOG_DBG("Pressure: %d,%d kPa", sPressure.val1, sPressure.val2);

	EmberAfStatus status = Clusters::PressureMeasurement::Attributes::MeasuredValue::Set(2, pressure);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating pressure measurement %x", status);
	}
}

void AppTask::UpdateHumidityClusterState()
{
	struct sensor_value sHumidity;

	int result = sensor_channel_get(sBme680SensorDev, SENSOR_CHAN_HUMIDITY, &sHumidity);
	if (result != 0) {
		LOG_ERR("Getting humidity measurement data from BME688 failed with: %d", result);
		return;
	}

	/* Defined by cluster humidity measured value = 100 x humidity in %RH with resolution of 0.01 %.
	 * val1 is an integer part of the value and val2 is fractional part in one-millionth parts.
	 * To achieve resolution of 0.01 % val2 needs to be divided by 10000. */
	uint16_t humidity = static_cast<int16_t>(sHumidity.val1 * 100 + sHumidity.val2 / 10000);
	LOG_DBG("Humidity: %d,%d %% rel", sHumidity.val1, sHumidity.val2);

	EmberAfStatus status = Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(3, humidity);
	if (status != EMBER_ZCL_STATUS_SUCCESS) {
		LOG_ERR("Updating humidity measurement %x", status);
	}
}

void AppTask::RelayOnHandler(const AppEvent &)
{
	LOG_DBG("Turning relay on");

	int err = gpio_pin_set_dt(&load_switch, 1);
	if (err != 0) {
		LOG_ERR("Setting GPIO pin on failed: %d", err);
		return;
	}
}

void AppTask::RelayOffHandler(const AppEvent &)
{
	LOG_DBG("Turning relay off");

	int err = gpio_pin_set_dt(&load_switch, 0);
	if (err != 0) {
		LOG_ERR("Setting GPIO pin off failed: %d", err);
		return;
	}
}

void AppTask::FanModeHandler(const AppEvent &event)
{
	// LOG_DBG("Setting fan pwm to state: %d", event.FanModeEvent.FanMode);
	int err;

	switch (event.ControlFanModeEvent.ControlFanMode) {
	case ControlFanMode::Off:
		err = pwm_set_pulse_dt(&pwm_fan, off_pulse);
		break;
	case ControlFanMode::Low:
		err = pwm_set_pulse_dt(&pwm_fan, low_pulse);
		break;
	case ControlFanMode::Mid:
		err = pwm_set_pulse_dt(&pwm_fan, mid_pulse);
		break;
	case ControlFanMode::High:
		err = pwm_set_pulse_dt(&pwm_fan, high_pulse);
		break;
	}

	if (err != 0) {
		LOG_ERR("Setting fan PWM state failed: %d", err);
		return;
	}
}

void AppTask::FunctionHandler(const AppEvent &event)
{
	if (event.ButtonEvent.PinNo != FUNCTION_BUTTON)
		return;

	if (event.ButtonEvent.Action == static_cast<uint8_t>(AppEventType::ButtonPushed)) {
		Instance().StartTimer(kFactoryResetTriggerTimeout);
		Instance().mFunction = FunctionEvent::FactoryReset;
	} else if (event.ButtonEvent.Action == static_cast<uint8_t>(AppEventType::ButtonReleased)) {
		if (Instance().mFunction == FunctionEvent::FactoryReset) {
			sFactoryResetLEDs.Set(false);
			UpdateStatusLED();
			Instance().CancelTimer();
			Instance().mFunction = FunctionEvent::NoneSelected;
			LOG_INF("Factory Reset has been Canceled");
		}
	}
}

void AppTask::LEDStateUpdateHandler(LEDWidget &ledWidget)
{
	AppEvent event;
	event.Type = AppEventType::UpdateLedState;
	event.Handler = UpdateLedStateEventHandler;
	event.UpdateLedStateEvent.LedWidget = &ledWidget;
	PostEvent(event);
}

void AppTask::UpdateLedStateEventHandler(const AppEvent &event)
{
	if (event.Type == AppEventType::UpdateLedState) {
		event.UpdateLedStateEvent.LedWidget->UpdateState();
	}
}

void AppTask::UpdateStatusLED()
{
	/* Update the status LED.
	 *
	 * If IPv6 networking and service provisioned, keep the LED On constantly.
	 *
	 * If the system has BLE connection(s) uptill the stage above, THEN blink the LED at an even
	 * rate of 100ms.
	 *
	 * Otherwise, blink the LED for a very short time. */
	if (sIsNetworkProvisioned && sIsNetworkEnabled) {
		sStatusLED.Set(true);
	} else if (sHaveBLEConnections) {
		sStatusLED.Blink(LedConsts::StatusLed::Unprovisioned::kOn_ms,
				 LedConsts::StatusLed::Unprovisioned::kOff_ms);
	} else {
		sStatusLED.Blink(LedConsts::StatusLed::Provisioned::kOn_ms, LedConsts::StatusLed::Provisioned::kOff_ms);
	}
}

void AppTask::ChipEventHandler(const ChipDeviceEvent *event, intptr_t /* arg */)
{
	switch (event->Type) {
	case DeviceEventType::kCHIPoBLEAdvertisingChange:
		sHaveBLEConnections = ConnectivityMgr().NumBLEConnections() != 0;
		UpdateStatusLED();
		break;
#if defined(CONFIG_NET_L2_OPENTHREAD)
	case DeviceEventType::kDnssdInitialized:
#if CONFIG_CHIP_OTA_REQUESTOR
		InitBasicOTARequestor();
#endif /* CONFIG_CHIP_OTA_REQUESTOR */
		break;
	case DeviceEventType::kThreadStateChange:
		sIsNetworkProvisioned = ConnectivityMgr().IsThreadProvisioned();
		sIsNetworkEnabled = ConnectivityMgr().IsThreadEnabled();
#elif defined(CONFIG_CHIP_WIFI)
	case DeviceEventType::kWiFiConnectivityChange:
		sIsNetworkProvisioned = ConnectivityMgr().IsWiFiStationProvisioned();
		sIsNetworkEnabled = ConnectivityMgr().IsWiFiStationEnabled();
#if CONFIG_CHIP_OTA_REQUESTOR
		if (event->WiFiConnectivityChange.Result == kConnectivity_Established) {
			InitBasicOTARequestor();
		}
#endif /* CONFIG_CHIP_OTA_REQUESTOR */
#endif
		UpdateStatusLED();
		break;
	default:
		break;
	}
}

void AppTask::CancelTimer()
{
	k_timer_stop(&sFunctionTimer);
}

void AppTask::StartTimer(uint32_t timeoutInMs)
{
	k_timer_start(&sFunctionTimer, K_MSEC(timeoutInMs), K_NO_WAIT);
}

void AppTask::PostEvent(const AppEvent &event)
{
	if (k_msgq_put(&sAppEventQueue, &event, K_NO_WAIT) != 0) {
		LOG_INF("Failed to post event to app task event queue");
	}
}

void AppTask::DispatchEvent(const AppEvent &event)
{
	if (event.Handler) {
		event.Handler(event);
	} else {
		LOG_INF("Event received with no handler. Dropping event.");
	}
}