import os
import pickle
import re
import time
from PMXmlParser import PMXmlParser
from cu.PMCUContext import PMCUContext
from PMConnection import PMConnection

print(os.path.abspath(os.path.curdir))
serializedDataFile = open("data/data_v370.pkl", "rb")
defined_parameters = pickle.load(serializedDataFile)
serializedDataFile.close()


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
    r = re.compile('(\\d+)')
    l = r.split(x.get_id())
    return [int(y) if y.isdigit() else y for y in l]

supported_parameters = ecu_parameters + ecu_switch_parameters + ecu_calculated_parameters # + tcu_parameters + tcu_switch_parameters + tcu_calculated_parameters
supported_parameters = sorted(supported_parameters, key=stringSplitByNumbers)

for p in supported_parameters:
    print(p._id, p.get_cu_type(), p._name, p._desc)