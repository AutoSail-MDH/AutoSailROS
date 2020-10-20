import unittest
import sys
sys.path.append('../scripts/')
from src.ctrl import rudder_controller

rudder_controller.pid_()

class TestPID(unittest.TestCase):
    def test_course_straight(self):
        return
    
