#pragma once

// ============================================================
// FEEDBACK MODULES — всё про обратную связь одним include
//
// ⚠️ ЗАГОТОВКА, НИКУДА НЕ ПОДКЛЮЧЕНА (см. FeedbackSupervisor.h).
// Нигде в прошивке не включается; нужен тестам и будущему
// подключению в FlightController.
// ============================================================

#include "autopilot/feedback/FeedbackConfig.h"
#include "autopilot/feedback/FeedbackMath.h"
#include "autopilot/feedback/FlightSnapshot.h"
#include "autopilot/feedback/FeedbackOutput.h"
#include "autopilot/feedback/PhaseTargets.h"
#include "autopilot/feedback/SpeedEstimator.h"
#include "autopilot/feedback/AirborneDetector.h"
#include "autopilot/feedback/ControlEffectivenessEstimator.h"
#include "autopilot/feedback/AdaptiveRateController.h"
#include "autopilot/feedback/ControlDirectionGuard.h"
#include "autopilot/feedback/StallGuard.h"
#include "autopilot/feedback/TakeoffSequencer.h"
#include "autopilot/feedback/LandingSequencer.h"
#include "autopilot/feedback/FeedbackSupervisor.h"
