"""
Rear diff temperature: AEM 30-2012 NTC sensor read through the ADS1263 HAT.

Split out of bumble_service.py; logic unchanged.
"""

# --- Configuration ---
V_SOURCE = 3.3
R_PULLUP = 2200.0
REF = 5.08

# Calibration Table (resistance ohms -> temp C)
CALIBRATION_TABLE = {
    28136: -20, 15813: -10, 9319: 0, 5589: 10, 3476: 20,
    2230: 30, 1466: 40, 984: 50, 671: 60, 468: 70,
    332: 80, 239: 90, 175: 100, 129: 110, 97: 120,
    73: 130, 57: 140, 43: 150
}


def get_temp_from_resistance(r_measured):
    """Interpolates temperature from resistance value."""
    sorted_ohms = sorted(CALIBRATION_TABLE.keys(), reverse=True)
    if r_measured >= sorted_ohms[0]:
        return CALIBRATION_TABLE[sorted_ohms[0]]
    if r_measured <= sorted_ohms[-1]:
        return CALIBRATION_TABLE[sorted_ohms[-1]]
    for i in range(len(sorted_ohms) - 1):
        r_high = sorted_ohms[i]
        r_low = sorted_ohms[i+1]
        if r_low <= r_measured <= r_high:
            t_low_r = CALIBRATION_TABLE[r_high]
            t_high_r = CALIBRATION_TABLE[r_low]
            ratio = (r_measured - r_low) / (r_high - r_low)
            temp = t_high_r - (ratio * (t_high_r - t_low_r))
            return temp
    return None


class SensorReader:
    """Handles ADC sensor reading."""
    def __init__(self):
        # Imported here so the rest of the package loads off-Pi (RPi.GPIO, spidev).
        from r22b.adc import ads1263
        self.adc = ads1263.ADS1263()
        if self.adc.ADS1263_init_ADC1('ADS1263_400SPS') != -1:
            self.adc.ADS1263_SetMode(0)

    def read_temp(self) -> float:
        """Reads sensor and returns temp in Celsius."""
        try:
            adc_value = self.adc.ADS1263_GetChannalValue(0)
            v_out = adc_value * (REF / 0x7fffffff)
            if v_out >= (REF - 0.05):
                return -999.0
            r_sensor = (R_PULLUP * v_out) / (REF - v_out)
            return get_temp_from_resistance(r_sensor)
        except:
            return -999.0
