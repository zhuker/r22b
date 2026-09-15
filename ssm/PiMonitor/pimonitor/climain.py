import os
import pickle
import re
import time
from PMXmlParser import PMXmlParser
from cu.PMCUContext import PMCUContext
from PMConnection import PMConnection

parser = PMXmlParser()


if os.path.isfile("data/data.pkl"):
    serializedDataFile = open("data/data.pkl", "rb")
    defined_parameters = pickle.load(serializedDataFile)
    serializedDataFile.close()
else:
    defined_parameters = parser.parse("logger_METRIC_EN_v352.xml")
    defined_parameters = sorted(defined_parameters, key=lambda x: x.get_id(), reverse=True)
    output = open("data/data.pkl", "wb")
    pickle.dump(defined_parameters, output, -1)
    output.close()


connection = PMConnection()
ser = connection.open()
print("ser", ser)
ecu_packet = connection.init(1)
print("ecu_packet", ecu_packet)
tcu_packet = connection.init(2)
print("tcu_packet", tcu_packet)


ecu_context = PMCUContext(ecu_packet, [1, 3])
ecu_parameters = ecu_context.match_parameters(defined_parameters)
ecu_switch_parameters = ecu_context.match_switch_parameters(defined_parameters)
ecu_calculated_parameters = ecu_context.match_calculated_parameters(defined_parameters, ecu_parameters)

# tcu_context = PMCUContext(tcu_packet, [2])
# tcu_parameters = tcu_context.match_parameters(defined_parameters)
# tcu_switch_parameters = tcu_context.match_switch_parameters(defined_parameters)
# tcu_calculated_parameters = tcu_context.match_calculated_parameters(defined_parameters, tcu_parameters)

print("ECU ROM ID: " + ecu_context.get_rom_id())
# print("TCU ROM ID: " + tcu_context.get_rom_id())

def stringSplitByNumbers(x):
    r = re.compile('(\d+)')
    l = r.split(x.get_id())
    return [int(y) if y.isdigit() else y for y in l]

supported_parameters = ecu_parameters + ecu_switch_parameters + ecu_calculated_parameters # + tcu_parameters + tcu_switch_parameters + tcu_calculated_parameters

supported_parameters = sorted(supported_parameters, key=stringSplitByNumbers)

engine_speed = None
air_intake_temperature = None
coolant_temperature = None
# P27 Fuel Tank Pressure P27
# P31 Fuel Temperature P31
# P35 Fuel Level P35
# P47 Fuel Pump Duty P47
fuel_tank_pressure = None
fuel_temperature = None
fuel_level = None
fuel_pump_duty = None
for p in supported_parameters:
    if p.get_id() == "P8":
        engine_speed = p
    elif p.get_id() == "P11":
        air_intake_temperature = p
    elif p.get_id() == "P2":
        coolant_temperature = p
    elif p.get_id() == "P27":
        fuel_tank_pressure = p
    elif p.get_id() == "P31":
        fuel_temperature = p
    elif p.get_id() == "P35":
        fuel_level = p
    elif p.get_id() == "P47":
        fuel_pump_duty = p
    
print(engine_speed._id, engine_speed._name, engine_speed._desc)
print(air_intake_temperature._id, air_intake_temperature._name, air_intake_temperature._desc)
print(coolant_temperature._id, coolant_temperature._name, coolant_temperature._desc)
print(fuel_tank_pressure._id, fuel_tank_pressure._name, fuel_tank_pressure._desc)
print(fuel_temperature._id, fuel_temperature._name, fuel_temperature._desc)
print(fuel_level._id, fuel_level._name, fuel_level._desc)
print(fuel_pump_duty._id, fuel_pump_duty._name, fuel_pump_duty._desc)


start = time.time()
while True:
    x = connection.read_parameters([engine_speed, air_intake_temperature, coolant_temperature, fuel_tank_pressure, fuel_temperature, fuel_level, fuel_pump_duty])
    engine_speed_value = engine_speed.get_value(x[0])
    air_intake_temperature_value = air_intake_temperature.get_value(x[1])
    coolant_temperature_value = coolant_temperature.get_value(x[2])
    fuel_tank_pressure_value = fuel_tank_pressure.get_value(x[3])
    fuel_temperature_value = fuel_temperature.get_value(x[4])
    fuel_level_value = fuel_level.get_value(x[5])
    fuel_pump_duty_value = fuel_pump_duty.get_value(x[6])

    air_intake_temperature_unit = air_intake_temperature.get_default_unit()
    coolant_temperature_unit = coolant_temperature.get_default_unit()
    fuel_tank_pressure_unit = fuel_tank_pressure.get_default_unit()
    fuel_temperature_unit = fuel_temperature.get_default_unit()
    fuel_level_unit = fuel_level.get_default_unit()
    fuel_pump_duty_unit = fuel_pump_duty.get_default_unit()


    elapsed = time.time() - start
    msec = int(elapsed * 1000)
    print(msec, f"RPM: {engine_speed_value}",
          f"air intake {air_intake_temperature_unit}: {air_intake_temperature_value} ", 
          f"coolant {coolant_temperature_unit}:", coolant_temperature_value, 
          f"fuel tank pressure {fuel_tank_pressure_unit}:", fuel_tank_pressure_value, 
          f"fuel temp {fuel_temperature_unit}:", fuel_temperature_value, 
          f"fuel level {fuel_level_unit}:", fuel_level_value, 
          f"fuel pump duty {fuel_pump_duty_unit}:", fuel_pump_duty_value)
    # time.sleep(0.01)
connection.close()
