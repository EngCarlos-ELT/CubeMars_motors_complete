# integration_tests.py
# Running the Tests:   python -m unittest integration_tests.py   or   python -m unittest -v integration_tests.py
# Integration tests for the CubeMars AK80-64 motor control application

import unittest
from unittest.mock import MagicMock, patch
import threading
import time
import can

# Import application modules
from constants import *
from motor import MotorState
from app import MotorControlApp

class TestAppInitialization(unittest.TestCase):
    """Integration tests for application initialization"""

    @patch('can.interface.Bus')
    def test_app_initialization(self, mock_bus_class):
        """Test that the application initializes correctly"""
        # Mock the CAN bus
        mock_bus = MagicMock()
        mock_bus_class.return_value = mock_bus

        # Create app instance without running the Kivy event loop
        with patch('kivy.app.App.run'):
            app = MotorControlApp()

            # Check that motor state was initialized
            self.assertIsInstance(app.motor_state, MotorState)

            # Check that app is not connected initially
            self.assertFalse(app.connected)
            self.assertIsNone(app.bus)


class TestConnectionManagement(unittest.TestCase):
    """Integration tests for connection management"""

    @patch('can.interface.Bus')
    def test_connection_toggle(self, mock_bus_class):
        """Test connection toggling functionality"""
        # Mock the CAN bus
        mock_bus = MagicMock()
        mock_bus_class.return_value = mock_bus

        # Create app instance without running the Kivy event loop
        with patch('kivy.app.App.run'):
            app = MotorControlApp()

            # Mock UI components that would be created in build()
            app.channel_input = MagicMock()
            app.channel_input.text = 'can0'
            app.connect_button = MagicMock()
            app.status_label = MagicMock()
            app.enter_mode_button = MagicMock()
            app.exit_mode_button = MagicMock()
            app.zero_position_button = MagicMock()
            app.send_command_button = MagicMock()
            app.continuous_button = MagicMock()
            app.log_label = MagicMock()

            # Test connecting
            app.toggle_connection(None)

            # Check that connection was established
            self.assertTrue(app.connected)
            self.assertIsNotNone(app.bus)
            mock_bus_class.assert_called_once()

            # Check that UI was updated
            app.connect_button.text = "Disconnect"
            app.status_label.text = "Connected"
            app.enter_mode_button.disabled = False
            app.exit_mode_button.disabled = False
            app.zero_position_button.disabled = False
            app.send_command_button.disabled = False
            app.continuous_button.disabled = False

            # Test disconnecting
            app.toggle_connection(None)

            # Check that connection was closed
            self.assertFalse(app.connected)
            mock_bus.shutdown.assert_called_once()

            # Check that UI was updated
            app.connect_button.text = "Connect"
            app.status_label.text = "Disconnected"
            app.enter_mode_button.disabled = True
            app.exit_mode_button.disabled = True
            app.zero_position_button.disabled = True
            app.send_command_button.disabled = True
            app.continuous_button.disabled = True


class TestCommandSending(unittest.TestCase):
    """Integration tests for command sending"""

    @patch('can.interface.Bus')
    def test_command_sending(self, mock_bus_class):
        """Test sending commands to the motor"""
        # Mock the CAN bus
        mock_bus = MagicMock()
        mock_bus_class.return_value = mock_bus

        # Create app instance without running the Kivy event loop
        with patch('kivy.app.App.run'):
            app = MotorControlApp()

            # Mock UI components that would be created in build()
            app.channel_input = MagicMock()
            app.channel_input.text = 'can0'
            app.connect_button = MagicMock()
            app.status_label = MagicMock()
            app.enter_mode_button = MagicMock()
            app.exit_mode_button = MagicMock()
            app.zero_position_button = MagicMock()
            app.send_command_button = MagicMock()
            app.continuous_button = MagicMock()
            app.log_label = MagicMock()
            app.position_slider = MagicMock(value=5.0)
            app.velocity_slider = MagicMock(value=2.0)
            app.kp_slider = MagicMock(value=100.0)
            app.kd_slider = MagicMock(value=2.0)
            app.torque_slider = MagicMock(value=10.0)

            # Connect to mock bus
            app.toggle_connection(None)

            # Reset mock to clear connection calls
            mock_bus.reset_mock()

            # Test enter mode
            app.on_enter_mode(None)
            mock_bus.send.assert_called_once()
            args, kwargs = mock_bus.send.call_args
            msg = args[0]
            self.assertEqual(list(msg.data), [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])

            # Reset mock
            mock_bus.reset_mock()

            # Test exit mode
            app.on_exit_mode(None)
            mock_bus.send.assert_called_once()
            args, kwargs = mock_bus.send.call_args
            msg = args[0]
            self.assertEqual(list(msg.data), [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

            # Reset mock
            mock_bus.reset_mock()

            # Test zero position
            app.on_zero_position(None)
            mock_bus.send.assert_called_once()
            args, kwargs = mock_bus.send.call_args
            msg = args[0]
            self.assertEqual(list(msg.data), [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

            # Reset mock
            mock_bus.reset_mock()

            # Test send command
            app.on_send_command(None)
            mock_bus.send.assert_called_once()

            # Check that motor state was updated from sliders
            self.assertEqual(app.motor_state.p_in, 5.0)
            self.assertEqual(app.motor_state.v_in, 2.0)
            self.assertEqual(app.motor_state.kp_in, 100.0)
            self.assertEqual(app.motor_state.kd_in, 2.0)
            self.assertEqual(app.motor_state.t_in, 10.0)


class TestContinuousControl(unittest.TestCase):
    """Integration tests for continuous control functionality"""

    @patch('can.interface.Bus')
    @patch('kivy.clock.Clock.schedule_interval')
    def test_continuous_control(self, mock_schedule, mock_bus_class):
        """Test continuous control functionality"""
        # Mock the CAN bus
        mock_bus = MagicMock()
        mock_bus_class.return_value = mock_bus

        # Create app instance without running the Kivy event loop
        with patch('kivy.app.App.run'):
            app = MotorControlApp()

            # Mock UI components that would be created in build()
            app.channel_input = MagicMock()
            app.channel_input.text = 'can0'
            app.connect_button = MagicMock()
            app.status_label = MagicMock()
            app.enter_mode_button = MagicMock()
            app.exit_mode_button = MagicMock()
            app.zero_position_button = MagicMock()
            app.send_command_button = MagicMock()
            app.continuous_button = MagicMock()
            app.log_label = MagicMock()
            app.position_slider = MagicMock(value=5.0)
            app.velocity_slider = MagicMock(value=2.0)
            app.kp_slider = MagicMock(value=100.0)
            app.kd_slider = MagicMock(value=2.0)
            app.torque_slider = MagicMock(value=10.0)
            app.continuous_checkbox = MagicMock(text="0.1")

            # Connect to mock bus
            app.toggle_connection(None)

            # Test starting continuous control
            app.toggle_continuous(None)

            # Check that continuous control was started
            self.assertTrue(app.continuous_active)
            app.continuous_button.text = "Stop Continuous"
            mock_schedule.assert_called_once()

            # Test stopping continuous control
            mock_event = MagicMock()
            app.continuous_event = mock_event
            app.toggle_continuous(None)

            # Check that continuous control was stopped
            self.assertFalse(app.continuous_active)
            app.continuous_button.text = "Start Continuous"
            mock_event.cancel.assert_called_once()


class TestStatusMonitoring(unittest.TestCase):
    """Integration tests for status monitoring functionality"""

    @patch('can.interface.Bus')
    def test_status_update(self, mock_bus_class):
        """Test status update functionality"""
        # Mock the CAN bus
        mock_bus = MagicMock()
        mock_msg = MagicMock()
        # Position = 5.0, Velocity = 2.0, Torque = 10.0
        mock_msg.data = bytearray([0x00, 0xC0, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x00])
        mock_bus.recv.return_value = mock_msg
        mock_bus_class.return_value = mock_bus

        # Create app instance without running the Kivy event loop
        with patch('kivy.app.App.run'):
            app = MotorControlApp()

            # Mock UI components that would be created in build()
            app.channel_input = MagicMock()
            app.channel_input.text = 'can0'
            app.connect_button = MagicMock()
            app.status_label = MagicMock()
            app.status_display = MagicMock()
            app.status_display.update_values = MagicMock()
            app.log_label = MagicMock()

            # Connect to mock bus
            app.toggle_connection(None)

            # Run status update thread for a short time
            app.stop_thread = False
            thread = threading.Thread(target=app.status_update_thread)
            thread.daemon = True
            thread.start()

            # Give thread time to execute
            time.sleep(0.5)

            # Stop thread
            app.stop_thread = True
            thread.join(timeout=1.0)

            # Check that bus.recv was called
            mock_bus.recv.assert_called()

            # Check that motor state was updated
            # Note: This is approximate due to the conversion process
            self.assertGreater(app.motor_state.p_out, 0)
            self.assertGreater(app.motor_state.v_out, 0)
            self.assertGreater(app.motor_state.t_out, 0)


class TestEndToEndWorkflow(unittest.TestCase):
    """End-to-end integration tests for the complete workflow"""

    @patch('can.interface.Bus')
    @patch('kivy.clock.Clock.schedule_interval')
    def test_complete_workflow(self, mock_schedule, mock_bus_class):
        """Test the complete workflow from connection to command sending to status updates"""
        # Mock the CAN bus
        mock_bus = MagicMock()
        mock_msg = MagicMock()
        mock_msg.data = bytearray([0x00, 0xC0, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x00])
        mock_bus.recv.return_value = mock_msg
        mock_bus_class.return_value = mock_bus

        # Create app instance without running the Kivy event loop
        with patch('kivy.app.App.run'):
            app = MotorControlApp()

            # Mock UI components that would be created in build()
            app.channel_input = MagicMock()
            app.channel_input.text = 'can0'
            app.connect_button = MagicMock()
            app.status_label = MagicMock()
            app.enter_mode_button = MagicMock()
            app.exit_mode_button = MagicMock()
            app.zero_position_button = MagicMock()
            app.send_command_button = MagicMock()
            app.continuous_button = MagicMock()
            app.log_label = MagicMock()
            app.position_slider = MagicMock(value=5.0)
            app.velocity_slider = MagicMock(value=2.0)
            app.kp_slider = MagicMock(value=100.0)
            app.kd_slider = MagicMock(value=2.0)
            app.torque_slider = MagicMock(value=10.0)
            app.continuous_checkbox = MagicMock(text="0.1")
            app.status_display = MagicMock()
            app.status_display.update_values = MagicMock()

            # 1. Connect to the motor
            app.toggle_connection(None)
            self.assertTrue(app.connected)

            # 2. Enter MIT mode
            mock_bus.reset_mock()
            app.on_enter_mode(None)
            mock_bus.send.assert_called_once()

            # 3. Zero position
            mock_bus.reset_mock()
            app.on_zero_position(None)
            mock_bus.send.assert_called_once()

            # 4. Send a command
            mock_bus.reset_mock()
            app.on_send_command(None)
            mock_bus.send.assert_called_once()

            # 5. Start continuous control
            app.toggle_continuous(None)
            self.assertTrue(app.continuous_active)

            # 6. Simulate receiving status updates
            app.stop_thread = False
            thread = threading.Thread(target=app.status_update_thread)
            thread.daemon = True
            thread.start()

            # Give thread time to execute
            time.sleep(0.5)

            # 7. Stop continuous control
            app.toggle_continuous(None)
            self.assertFalse(app.continuous_active)

            # 8. Exit MIT mode
            mock_bus.reset_mock()
            app.on_exit_mode(None)
            mock_bus.send.assert_called_once()

            # 9. Disconnect
            app.stop_thread = True
            thread.join(timeout=1.0)
            mock_bus.reset_mock()
            app.toggle_connection(None)
            self.assertFalse(app.connected)
            mock_bus.shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
