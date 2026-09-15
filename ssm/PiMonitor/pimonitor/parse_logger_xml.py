import pickle
from PMXmlParser import PMXmlParser
parser = PMXmlParser()
    
defined_parameters = parser.parse("logger_METRIC_EN_v370.xml")
defined_parameters = sorted(defined_parameters, key=lambda x: x.get_id(), reverse=True)
output = open("data/data_v370.pkl", "wb")
pickle.dump(defined_parameters, output, -1)
output.close()