# unit_tests.py
#Running the Test: python -m unittest unit_tests.py  or  python -m unittest -v unit_tests.py
# Unit tests for the CubeMars AK80-64 motor control application

import unittest
from unittest.mock import MagicMock, patch
import can

# Import application modules
from constants import *
from motor import MotorState, float_to_uint, uint_to_float, enter_mode, exit_mode, zero_position, pack_cmd, unpack_reply

class TestDataConversion(unittest.TestCase):
    """Unit tests for data conversion functions"""
    
    def test_float_to_uint_12bit(self):
        """Test conversion from float to 12-bit unsigned integer"""
        # Test minimum value
        self.assertEqual(float_to_uint(P_MIN, P_MIN, P_MAX, 12), 0)
        
        # Test maximum value
        self.assertEqual(float_to_uint(P_MAX, P_MIN, P_MAX, 12), 4095)
        
        # Test middle value
        middle = (P_MIN + P_MAX) / 2
        self.assertEqual(float_to_uint(middle, P_MIN, P_MAX, 12), 2047)  # Approximately half of 4095
    
    def test_float_to_uint_16bit(self):
        """Test conversion from float to 16-bit unsigned integer"""
        # Test minimum value
        self.assertEqual(float_to_uint(P_MIN, P_MIN, P_MAX, 16), 0)
        
        # Test maximum value
        self.assertEqual(float_to_uint(P_MAX, P_MIN, P_MAX, 16), 65535)
        
        # Test middle value
        middle = (P_MIN + P_MAX) / 2
        self.assertEqual(float_to_uint(middle, P_MIN, P_MAX, 16), 32767)  # Approximately half of 65535
    
    def test_uint_to_float_12bit(self):
        """Test conversion from 12-bit unsigned integer to float"""
        # Test minimum value
        self.assertAlmostEqual(uint_to_float(0, P_MIN, P_MAX, 12), P_MIN)
        
        # Test maximum value
        self.assertAlmostEqual(uint_to_float(4095, P_MIN, P_MAX, 12), P_MAX)
        
        # Test middle value
        self.assertAlmostEqual(uint_to_float(2047, P_MIN, P_MAX, 12), (P_MIN + P_MAX) / 2, places=2)
    
    def test_uint_to_float_16bit(self):
        """Test conversion from 16-bit unsigned integer to float"""
        # Test minimum value
        self.assertAlmostEqual(uint_to_float(0, P_MIN, P_MAX, 16), P_MIN)
        
        # Test maximum value
        self.assertAlmostEqual(uint_to_float(65535, P_MIN, P_MAX, 16), P_MAX)
        
        # Test middle value
        self.assertAlmostEqual(uint_to_float(32767, P_MIN, P_MAX, 16), (P_MIN + P_MAX) / 2, places=2)
    
    def test_roundtrip_conversion(self):
        """Test that converting from float to uint and back returns the original value"""
        original_values = [-12.5, -10.0, -5.0, 0.0, 5.0, 10.0, 12.5]
        
        for value in original_values:
            # 12-bit roundtrip
            uint_12 = float_to_uint(value, P_MIN, P_MAX, 12)
            float_12 = uint_to_float(uint_12, P_MIN, P_MAX, 12)
            self.assertAlmostEqual(value, float_12, places=2)
            
            # 16-bit roundtrip
            uint_16 = float_to_uint(value, P_MIN, P_MAX, 16)
            float_16 = uint_to_float(uint_16, P_MIN, P_MAX, 16)
            self.assertAlmostEqual(value, float_16, places=2)


class TestMotorState(unittest.TestCase):
    """Unit tests for MotorState class"""
    
    def test_motor_state_initialization(self):
        """Test that MotorState initializes with correct default values"""
        state = MotorState()
        
        # Check input values
        self.assertEqual(state.p_in, 0.0)
        self.assertEqual(state.v_in, 0.0)
        self.assertEqual(state.kp_in, 0.0)
        self.assertEqual(state.kd_in, 0.50)
        self.assertEqual(state.t_in, 0.0)
        
        # Check output values
        self.assertEqual(state.p_out, 0.0)
        self.assertEqual(state.v_out, 0.0)
        self.assertEqual(state.t_out, 0.0)


class TestMotorCommands(unittest.TestCase):
    """Unit tests for motor command functions"""
    
    def setUp(self):
        """Set up mock CAN bus for testing"""
        self.mock_bus = MagicMock()
        self.mock_bus.send = MagicMock(return_value=None)
        self.motor_state = MotorState()
    
    def test_enter_mode(self):
        """Test enter_mode function"""
        # Test with valid bus
        result = enter_mode(self.mock_bus)
        self.assertTrue(result)
        self.mock_bus.send.assert_called_once()
        
        # Check message content
        args, kwargs = self.mock_bus.send.call_args
        msg = args[0]
        self.assertEqual(msg.arbitration_id, CONTROLLER_ID)
        self.assertEqual(list(msg.data), [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        self.assertFalse(msg.is_extended_id)
        
        # Test with None bus
        self.mock_bus.reset_mock()
        result = enter_mode(None)
        self.assertFalse(result)
        self.mock_bus.send.assert_not_called()
    
    def test_exit_mode(self):
        """Test exit_mode function"""
        # Test with valid bus
        result = exit_mode(self.mock_bus)
        self.assertTrue(result)
        self.mock_bus.send.assert_called_once()
        
        # Check message content
        args, kwargs = self.mock_bus.send.call_args
        msg = args[0]
        self.assertEqual(msg.arbitration_id, CONTROLLER_ID)
        self.assertEqual(list(msg.data), [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        self.assertFalse(msg.is_extended_id)
        
        # Test with None bus
        self.mock_bus.reset_mock()
        result = exit_mode(None)
        self.assertFalse(result)
        self.mock_bus.send.assert_not_called()
    
    def test_zero_position(self):
        """Test zero_position function"""
        # Test with valid bus
        result = zero_position(self.mock_bus)
        self.assertTrue(result)
        self.mock_bus.send.assert_called_once()
        
        # Check message content
        args, kwargs = self.mock_bus.send.call_args
        msg = args[0]
        self.assertEqual(msg.arbitration_id, CONTROLLER_ID)
        self.assertEqual(list(msg.data), [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
        self.assertFalse(msg.is_extended_id)
        
        # Test with None bus
        self.mock_bus.reset_mock()
        result = zero_position(None)
        self.assertFalse(result)
        self.mock_bus.send.assert_not_called()
    
    def test_pack_cmd(self):
        """Test pack_cmd function"""
        # Set motor state values
        self.motor_state.p_in = 5.0
        self.motor_state.v_in = 2.0
        self.motor_state.kp_in = 100.0
        self.motor_state.kd_in = 2.0
        self.motor_state.t_in = 10.0
        
        # Test with valid bus
        result = pack_cmd(self.mock_bus, self.motor_state)
        self.assertTrue(result)
        self.mock_bus.send.assert_called_once()
        
        # Check that values are constrained properly
        self.motor_state.p_in = 20.0  # Above P_MAX
        self.motor_state.v_in = -10.0  # Below V_MIN
        result = pack_cmd(self.mock_bus, self.motor_state)
        self.assertTrue(result)
        
        # Test with None bus
        self.mock_bus.reset_mock()
        result = pack_cmd(None, self.motor_state)
        self.assertFalse(result)
        self.mock_bus.send.assert_not_called()
    
    def test_unpack_reply(self):
        """Test unpack_reply function"""
        # Create a mock CAN message
        mock_msg = MagicMock()
        
        # Position = 0.0 (middle of range)
        # Velocity = 0.0 (middle of range)
        # Torque = 0.0 (middle of range)
        mock_msg.data = bytearray([0x00, 0x80, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00])
        
        result = unpack_reply(mock_msg, self.motor_state)
        self.assertTrue(result)
        
        # Check that values were updated correctly (approximately)
        self.assertAlmostEqual(self.motor_state.p_out, 0.0, places=1)
        self.assertAlmostEqual(self.motor_state.v_out, 0.0, places=1)
        self.assertAlmostEqual(self.motor_state.t_out, 0.0, places=1)
        
        # Test with None message
        result = unpack_reply(None, self.motor_state)
        self.assertFalse(result)


if __name__ == "__main__":
    unittest.main()
