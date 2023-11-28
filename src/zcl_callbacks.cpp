#include "app_task.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app/ConcreteAttributePath.h>

using namespace ::chip;
using namespace ::chip::app::Clusters;

void MatterPostAttributeChangeCallback(const chip::app::ConcreteAttributePath &attributePath, uint8_t type,
				       uint16_t size, uint8_t *value)
{
	if (attributePath.mClusterId == OnOff::Id && attributePath.mAttributeId == OnOff::Attributes::OnOff::Id &&
	    attributePath.mEndpointId == 4) {
		AppEvent event;
		if (*value) {
			event.Type = AppEventType::RelayOn;
			event.Handler = AppTask::RelayOnHandler;
		} else {
			event.Type = AppEventType::RelayOff;
			event.Handler = AppTask::RelayOffHandler;
		}
		AppTask::Instance().PostEvent(event);
		return;
	}

	if (attributePath.mClusterId == FanControl::Id &&
	    attributePath.mAttributeId == FanControl::Attributes::FanMode::Id && attributePath.mEndpointId == 5) {
		if (*value < 0 || *value > 4) {
			return;
		}

		AppEvent event;
		event.Type = AppEventType::ControlFan;
		event.Handler = AppTask::FanModeHandler;
		event.ControlFanModeEvent.ControlFanMode = static_cast<::ControlFanMode>(*value);
		AppTask::Instance().PostEvent(event);
		return;
	}
}

/** @brief OnOff Cluster Init
 *
 * This function is called when a specific cluster is initialized. It gives the
 * application an opportunity to take care of cluster initialization procedures.
 * It is called exactly once for each endpoint where cluster is present.
 */
void emberAfOnOffClusterInitCallback(EndpointId endpoint)
{
	// EmberAfStatus status;
	// bool storedValue;

	// /* Read storedValue on/off value */
	// status = ::OnOff::Attributes::OnOff::Get(endpoint, &storedValue);
	// if (status == EMBER_ZCL_STATUS_SUCCESS) {
	// 	/* Set actual state to the cluster state that was last persisted */
	// 	AppEvent event;
	// 	if (status) {
	// 		event.Type = AppEventType::RelayOn;
	// 		event.Handler = AppTask::RelayOnHandler;
	// 	} else {
	// 		event.Type = AppEventType::RelayOff;
	// 		event.Handler = AppTask::RelayOffHandler;
	// 	}
	// 	AppTask::Instance().PostEvent(event);
	// }
}