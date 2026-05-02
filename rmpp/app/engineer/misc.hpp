#pragma once

#include "misc/LED.hpp"
#include "misc/Buzzer.hpp"
#include "flashdb/FlashDB.hpp"
#include "rc/RC.hpp"
#include "referee/Referee.hpp"
#include "ui/UI.hpp"

inline LED led({});
inline Buzzer buzzer;
inline FlashDB flashdb("engineer");
inline RC rc({}, {});
inline Referee referee({});
inline UI ui({});
