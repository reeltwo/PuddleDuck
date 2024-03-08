#pragma once

#include "PDConfig.h"
#include "PDGoMotorBus.h"
#include "PDGoActuator.h"
#include "PDLeg.h"

class PDRobot {
public:
	PDGoMotorBus* buses[MAX_NUM_BUS] = {};
	PDGoActuator neck;
	PDLeg left;
	PDLeg right;

	PDRobot(const PDConfig::Robot& config) :
		neck(config.neck.id, "neck"),
		left("left", config.leg.left),
		right("right", config.leg.right)
	{
		int busCount = 0;
		for (int i = 0; i < MAX_NUM_BUS; i++) {
			buses[i] = nullptr;
			if (config.bus[i].isUsed()) {
				buses[busCount++] = new PDGoMotorBus(
										config.bus[i].name,
										config.bus[i].adapter,
										config.bus[i].version);
			}
		}
		neck.setBus(getBus("neck", config.neck.bus));
		left.setBus(getBus("left", config.leg.left.bus));
		right.setBus(getBus("right", config.leg.right.bus));
	}

	PDGoMotorBus* getBus(PDString group, PDString name) {
		for (int i = 0; i < MAX_NUM_BUS; i++) {
			auto bus = buses[i];
			if (bus == nullptr)
				break;
			if (name == bus->getName()) {
				return bus;
			}
		}
		fprintf(stderr, "Unknown bus %s for %s\n", name.c_str(), group.c_str());
		return nullptr;
	}

	bool checkRanges() {
		bool success = true;
        if (!neck.checkRange()) {
            printf("[%d] %s: RANGE UNINITIALIZED\n",
                neck.getID(), neck.getName());
            success = false;
        }
        for (int i = 0; i < left.numberOfActuators(); i++) {
            if (!left.fActuator[i].checkRange()) {
                printf("[%d] left.%s: RANGE UNINITIALIZED\n",
                    left.fActuator[i].getID(), left.fActuator[i].getName());
	            success = false;
            }
        }
        for (int i = 0; i < right.numberOfActuators(); i++) {
            if (!right.fActuator[i].checkRange()) {
                printf("[%d] right.%s: RANGE UNINITIALIZED\n",
                    right.fActuator[i].getID(), right.fActuator[i].getName());
	            success = false;
            }
        }
        return success;
	}

	void relax() {
		neck.relax();
		left.relax();
		right.relax();
	}

	void stand() {
		left.stand();
		right.stand();
	}

	void updateJointRange(PDConfig::Leg::Joint& joint, PDGoActuator &actuator) {
	    joint.range.value[0] = actuator.getMinDegrees();
	    joint.range.value[1] = actuator.getMaxDegrees();
	    actuator.setRange(joint.range.value[0], joint.range.value[1]);
	}

	void updateJointRange(PDConfig::Robot& config) {
        if (!neck.hasError()) {
            printf("[%d] %s: [%f,%f]\n",
                neck.getID(), neck.getName(),
                neck.getMinDegrees(), neck.getMaxDegrees());
        }
        if (!neck.hasError()) {
		    config.neck.range.value[0] = neck.getMinDegrees();
		    config.neck.range.value[1] = neck.getMaxDegrees();
		    neck.setRange(config.neck.range.value[0], config.neck.range.value[1]);
        }

        for (int i = 0; i < left.numberOfActuators(); i++) {
            if (!left.fActuator[i].hasError()) {
                printf("[%d] left.%s: [%f,%f]\n",
                    left.fActuator[i].getID(), left.fActuator[i].getName(),
                    left.fActuator[i].getMinDegrees(), left.fActuator[i].getMaxDegrees());
            }
        }
        if (!left.fAnklePitch.hasError()) {
            updateJointRange(config.leg.left.ankle.pitch, left.fAnklePitch);
        }
        if (!left.fKneePitch.hasError()) {
            updateJointRange(config.leg.left.knee.pitch, left.fKneePitch);
        }
        if (!left.fHipPitch.hasError()) {
            updateJointRange(config.leg.left.hip.pitch, left.fHipPitch);
        }
        if (!left.fHipRoll.hasError()) {
            updateJointRange(config.leg.left.hip.roll, left.fHipRoll);
        }
        if (!left.fHipYaw.hasError()) {
            updateJointRange(config.leg.left.hip.yaw, left.fHipYaw);
        }

        for (int i = 0; i < right.numberOfActuators(); i++) {
            if (!right.fActuator[i].hasError()) {
                printf("[%d] right.%s: [%f,%f]\n",
                    right.fActuator[i].getID(), right.fActuator[i].getName(),
                    right.fActuator[i].getMinDegrees(), right.fActuator[i].getMaxDegrees());
            }
        }
        if (!right.fAnklePitch.hasError()) {
            updateJointRange(config.leg.right.ankle.pitch, right.fAnklePitch);
        }
        if (!right.fKneePitch.hasError()) {
            updateJointRange(config.leg.right.knee.pitch, right.fKneePitch);
        }
        if (!right.fHipPitch.hasError()) {
            updateJointRange(config.leg.right.hip.pitch, right.fHipPitch);
        }
        if (!right.fHipRoll.hasError()) {
            updateJointRange(config.leg.right.hip.roll, right.fHipRoll);
        }
        if (!right.fHipYaw.hasError()) {
            updateJointRange(config.leg.right.hip.yaw, right.fHipYaw);
        }
	}

	bool init(bool forceContinue) {
	    if (!left.update()) {
    	    fprintf(stderr, "Missing left motors\n");
        	for (int i = 0; i < left.numberOfActuators(); i++) {
	            if (!left.fActuator[i].isResponding()) {
    	            fprintf(stderr, "  [%d] %s\n", left.fActuator[i].getID(), left.fActuator[i].getName());
        	        if (forceContinue) {
            	        left.fActuator[i].setIgnore();
                	}
            	}
        	}
        	if (!forceContinue) {
	        	return false;
        	}
        }
	    if (!right.update()) {
    	    fprintf(stderr, "Missing right motors\n");
        	for (int i = 0; i < right.numberOfActuators(); i++) {
	            if (!right.fActuator[i].isResponding()) {
    	            fprintf(stderr, "  [%d] %s\n", right.fActuator[i].getID(), right.fActuator[i].getName());
        	        if (forceContinue) {
            	        right.fActuator[i].setIgnore();
                	}
            	}
        	}
        	if (!forceContinue) {
	        	return false;
        	}
        }
        return true;
    }

	bool update() {
		bool success = true;
	    if (!left.update()) {
	    	success = false;
	    }
	    if (!right.update()) {
	    	success = false;
	    }
	    return success;
	}
};
