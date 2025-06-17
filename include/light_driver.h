#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void light_driver_init(bool power);
void light_driver_set_power(bool power);

#ifdef __cplusplus
}
#endif
