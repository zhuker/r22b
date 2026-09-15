def voltage_to_psi_150(voltage):
    """
    Calculates PSIg from sensor voltage for AEM 30-2130-150 (0-150 PSIg).
    
    Formula derived from AEM datasheet:
    0.5V = 0 PSI
    4.5V = 150 PSI
    Slope = 150 / 4.0 = 37.5 PSI/Volt
    
    Args:
        voltage (float): Sensor output voltage (typically 0.5V to 4.5V)
        
    Returns:
        float: Pressure in PSIg
    """
    # Linear transfer function: PSI = (Voltage - 0.5) * (150 / 4.0)
    return (voltage - 0.5) * 37.5

# Test with values from the datasheet table 
test_voltages = [0.5, 0.75, 1.5, 2.5, 3.75, 4.5] 
for v in test_voltages:
    print(f"{v}V -> {voltage_to_psi_150(v)} PSI")