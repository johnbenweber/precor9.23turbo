#pragma once

void ftmsBegin(bool enableControlServices);
void ftmsUpdateTreadmillData(float speedKph,
                             bool speedValid,
                             float inclinePercent,
                             bool inclineValid);
float ftmsInclinePercent();
