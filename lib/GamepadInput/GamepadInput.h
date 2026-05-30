#pragma once

#include <Bluepad32.h>
#include <InputState.h>

namespace GamepadInput {

void begin();
void update();
InputState read();

}  // namespace GamepadInput
