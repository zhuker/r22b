import os
import pickle
import unittest

from pimonitor.cu.PMCUStandardParameter import PMCUStandardParameter


class MyTestCase(unittest.TestCase):
    def test_something(self):
        defined_parameters = None
        if os.path.isfile("data/data.pkl"):
            with open("data/data.pkl", "rb") as serializedDataFile:
                defined_parameters = pickle.load(serializedDataFile)
        self.assertIsNotNone(defined_parameters)
        for p in defined_parameters:
            if isinstance(p, PMCUStandardParameter):
                print(p._name, p._desc)




if __name__ == '__main__':
    unittest.main()
