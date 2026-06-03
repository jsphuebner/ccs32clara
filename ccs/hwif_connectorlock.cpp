/* Hardware Interface for Connector Locking


 This module handles the actuation of the connector lock motor and
 the evaluation of the connector lock feedback.

 Behaviour:
 - When LockClosedThresh == LockOpenThresh there is no feedback.
   The motor runs for LockRunTime and then the new position is assumed.
 - When LockClosedThresh != LockOpenThresh the analog feedback is evaluated.
   The motor stops as soon as the feedback confirms the target position, but
   in any case not longer than LockRunTime.

*/

#include "ccs32_globals.h"
#include "connectorLock.h"

static ConnectorLock connectorLock;


bool ConnectorLock::hasFeedback() const
{
   return Param::GetInt(Param::LockOpenThresh) != Param::GetInt(Param::LockClosedThresh);
}

LockStt ConnectorLock::getLockState()
{
   int lockOpenThresh = Param::GetInt(Param::LockOpenThresh);
   int lockClosedThresh = Param::GetInt(Param::LockClosedThresh);
   int feedbackValue = AnaIn::lockfb.Get();
   LockStt state = LOCK_UNKNOWN;

   if (lockClosedThresh >= lockOpenThresh) //Feedback value when closed is greater than when open
   {
      if (feedbackValue > lockClosedThresh)
         state = LOCK_CLOSED;
      else if (feedbackValue < lockOpenThresh)
         state = LOCK_OPEN;
      else if ((feedbackValue - lastFeedbackValue) < 10)
         state = LOCK_OPENING;
      else if ((feedbackValue - lastFeedbackValue) > 10)
         state = LOCK_CLOSING;
   }
   else /* (lockClosedThresh < lockOpenThresh) */
   {
      if (feedbackValue < lockClosedThresh)
         state = LOCK_CLOSED;
      else if (feedbackValue > lockOpenThresh)
         state = LOCK_OPEN;
      else if ((feedbackValue - lastFeedbackValue) > 10)
         state = LOCK_OPENING;
      else if ((feedbackValue - lastFeedbackValue) < 10)
         state = LOCK_CLOSING;
   }

   lastFeedbackValue = feedbackValue;
   return state;
}

/* The lock control via actuator test. */
void ConnectorLock::actuatorTest(uint8_t kindOfControl)
{
   if (kindOfControl != oldkindOfIoControl) {
      switch (kindOfControl) {
         case IOCONTROL_LOCK_CLOSE:
            /* Force motor to run: declare current state as open, request closed. */
            lockState = LOCK_OPEN;
            lockRequest = LOCK_CLOSED;
            break;
         case IOCONTROL_LOCK_OPEN:
            /* Force motor to run: declare current state as closed, request open. */
            lockState = LOCK_CLOSED;
            lockRequest = LOCK_OPEN;
            break;
         case IOCONTROL_LOCK_RETURN_CONTROL_TO_ECU:
            /* End of the actuator test. Restore the application request. */
            lockRequest = lastApplicationLockRequest;
            break;
      }
      oldkindOfIoControl = kindOfControl;
   }
}

/* The locking via application. */
void ConnectorLock::triggerLocking()
{
   if (oldkindOfIoControl == IOCONTROL_LOCK_RETURN_CONTROL_TO_ECU) {
      lockRequest = LOCK_CLOSED;
   }
   lastApplicationLockRequest = LOCK_CLOSED;
}

/* The unlocking via application. */
void ConnectorLock::triggerUnlocking()
{
   if (oldkindOfIoControl == IOCONTROL_LOCK_RETURN_CONTROL_TO_ECU) {
      lockRequest = LOCK_OPEN;
   }
   lastApplicationLockRequest = LOCK_OPEN;
}

bool ConnectorLock::isLocked() const
{
   return lockState == LOCK_CLOSED;
}

/* Cyclic main function for lock handling. Called in 30ms task. */
void ConnectorLock::handleLockRequests()
{
   /* calculation examples:
      1. LockDuty = 0%, means "no lock control". pwmNeg and pwmPos have the identical value. No effective voltage.
      2. LockDuty = 50%, means "half battery voltage". pwmNeg has 0.25*periode, pwmPos has 0.75*periode. Effective 50% voltage.
      3. LockDuty = 100%, means "full voltage". pwmNeg is 0. pwmPos = periode. Effective 100% voltage.
      Discussion reference: https://openinverter.org/forum/viewtopic.php?p=79675#p79675 */
   int pwmNeg = (CONTACT_LOCK_PERIOD / 2) - ((CONTACT_LOCK_PERIOD / 2) * Param::GetInt(Param::LockDuty)) / 100;
   int pwmPos = (CONTACT_LOCK_PERIOD / 2) + ((CONTACT_LOCK_PERIOD / 2) * Param::GetInt(Param::LockDuty)) / 100;

   /* Start motor when the request changes */
   if (lockRequest != lockRequestOld) {
      lockRequestOld = lockRequest;
      if (lockRequest == LOCK_OPEN && lockState != LOCK_OPEN) {
         addToTrace(MOD_HWIF, "unlocking the connector");
         hardwareInteface_setHBridge(pwmNeg, pwmPos);
         lockTimer = Param::GetInt(Param::LockRunTime) / 30; /* in 30ms steps */
      } else if (lockRequest == LOCK_CLOSED && lockState != LOCK_CLOSED) {
         addToTrace(MOD_HWIF, "locking the connector");
         hardwareInteface_setHBridge(pwmPos, pwmNeg);
         lockTimer = Param::GetInt(Param::LockRunTime) / 30; /* in 30ms steps */
      }
   }

   if (lockTimer > 0) {
      bool done = false;
      if (hasFeedback()) {
         /* With feedback: stop early when the target position is confirmed */
         lockState = getLockState();
         done = (lockState == lockRequest);
      }
      if (done) {
         hardwareInteface_setHBridge(0, 0);
         lockTimer = 0;
         addToTrace(MOD_HWIF, "finished connector (un)locking");
      } else {
         Param::SetInt(Param::LockState, lockRequest == LOCK_OPEN ? LOCK_OPENING : LOCK_CLOSING);
         lockTimer--;
         if (lockTimer == 0) {
            /* Timer expired: stop motor unconditionally */
            hardwareInteface_setHBridge(0, 0);
            if (!hasFeedback()) {
               lockState = lockRequest; /* assume position reached */
            }
            addToTrace(MOD_HWIF, "finished connector (un)locking");
         }
      }
   } else {
      /* Motor not running: keep displaying the current state */
      if (hasFeedback()) {
         lockState = getLockState();
      }
   }

   Param::SetInt(Param::LockState, lockState);
}


/* C-compatible wrapper functions called from the rest of the firmware */

void hwIf_connectorLockActuatorTest(uint8_t kindOfControl)
{
   connectorLock.actuatorTest(kindOfControl);
}

void hwIf_handleLockRequests(void)
{
   connectorLock.handleLockRequests();
}

void hardwareInterface_triggerConnectorLocking(void)
{
   connectorLock.triggerLocking();
}

void hardwareInterface_triggerConnectorUnlocking(void)
{
   connectorLock.triggerUnlocking();
}

uint8_t hardwareInterface_isConnectorLocked(void)
{
   return connectorLock.isLocked() ? 1 : 0;
}
