#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "config.h"

void BT_SendString(const char *str);
void ProcessBluetoothCommand(char *cmd);
void SendIMUDataBT(void);

#endif /* BLUETOOTH_H */