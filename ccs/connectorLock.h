/* ConnectorLock class - controls the connector locking motor */

#pragma once
#include "hardwareInterface.h"
#include <stdint.h>

class ConnectorLock {
public:
    void handleLockRequests();
    void actuatorTest(uint8_t kindOfControl);
    void triggerLocking();
    void triggerUnlocking();
    bool isLocked() const;

private:
    LockStt getLockState();
    bool hasFeedback() const;

    LockStt lockRequest = LOCK_OPEN;
    LockStt lockRequestOld = LOCK_UNKNOWN;
    LockStt lockState = LOCK_OPEN;
    LockStt lastApplicationLockRequest = LOCK_OPEN;
    uint16_t lockTimer = 0;
    uint8_t oldkindOfIoControl = IOCONTROL_LOCK_RETURN_CONTROL_TO_ECU;
    int lastFeedbackValue = 0;
};
