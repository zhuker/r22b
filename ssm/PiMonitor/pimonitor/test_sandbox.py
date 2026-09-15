import unittest

from pimonitor.PMPacket import PMPacket


class MyTestCase(unittest.TestCase):
    def test_something(self):
        ecu_data = [255, 162, 16, 17, 61, 18, 89, 64, 6, 115, 250, 203, 166, 43, 129, 254, 168, 0, 130, 0, 96, 206, 84, 248, 177, 228, 128, 0, 0, 0, 0, 0, 0, 0, 220, 0, 0, 117, 30, 48, 192, 240, 34, 0, 0, 67, 251, 0, 241, 0, 0, 0, 0, 0, 0, 0, 240]
        ecu_dst = 240
        ecu_src = 16
        ecu_packet = PMPacket(ecu_dst, ecu_src, ecu_data)
        self.assertEqual(True, False)  # add assertion here


if __name__ == '__main__':
    unittest.main()
