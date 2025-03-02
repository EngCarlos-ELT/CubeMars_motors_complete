# app.py
# Main application class for the motor control GUI

import can
import threading
import time
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.clock import Clock
from kivy.core.window import Window

from constants import *
from motor import MotorState, detect_can_interface, enter_mode, exit_mode, zero_position, pack_cmd, unpack_reply
from ui_components import LabeledSlider, StatusDisplay

class MotorControlApp(App):
    def __init__(self, **kwargs):
        super(MotorControlApp, self).__init__(**kwargs)
        self.motor_state = MotorState()
        self.bus = None
        self.connected = False
        self.status_thread = None
        self.stop_thread = False

    def build(self):
        # Set window size
        Window.size = (800, 600)
        Window.minimum_width, Window.minimum_height = 800, 600

        # Main layout
        main_layout = BoxLayout(orientation='vertical', padding=10, spacing=10)

        # Title
        title_label = Label(
            text="CubeMars AK80-64 Motor Control",
            font_size=24,
            size_hint=(1, 0.1)
        )
        main_layout.add_widget(title_label)

        # Connection settings
        connection_layout = BoxLayout(orientation='horizontal', size_hint=(1, 0.1), spacing=10)

        # Auto-detect CAN interface
        self.can_interface, self.channel = detect_can_interface()
        self.bitrate = 1000000

        connection_layout.add_widget(Label(text=f"Interface: {self.can_interface}", size_hint=(0.25, 1)))

        self.channel_input = TextInput(
            text=self.channel,
            multiline=False,
            size_hint=(0.25, 1)
        )
        connection_layout.add_widget(self.channel_input)

        self.connect_button = Button(
            text="Connect",
            size_hint=(0.25, 1),
            background_color=(0.2, 0.7, 0.2, 1)
        )
        self.connect_button.bind(on_press=self.toggle_connection)
        connection_layout.add_widget(self.connect_button)

        self.status_label = Label(
            text="Disconnected",
            size_hint=(0.25, 1),
            color=(1, 0.3, 0.3, 1)
        )
        connection_layout.add_widget(self.status_label)

        main_layout.add_widget(connection_layout)

        # Control and status area
        control_status_layout = BoxLayout(orientation='horizontal', size_hint=(1, 0.6))

        # Control sliders
        sliders_layout = BoxLayout(orientation='horizontal', size_hint=(0.7, 1), spacing=5)

        self.position_slider = LabeledSlider(
            label_text="Position",
            min_value=P_MIN,
            max_value=P_MAX,
            value=0.0
        )
        sliders_layout.add_widget(self.position_slider)

        self.velocity_slider = LabeledSlider(
            label_text="Velocity",
            min_value=V_MIN,
            max_value=V_MAX,
            value=0.0
        )
        sliders_layout.add_widget(self.velocity_slider)

        self.kp_slider = LabeledSlider(
            label_text="Kp",
            min_value=KP_MIN,
            max_value=KP_MAX,
            value=0.0
        )
        sliders_layout.add_widget(self.kp_slider)

        self.kd_slider = LabeledSlider(
            label_text="Kd",
            min_value=KD_MIN,
            max_value=KD_MAX,
            value=0.5
        )
        sliders_layout.add_widget(self.kd_slider)

        self.torque_slider = LabeledSlider(
            label_text="Torque",
            min_value=T_MIN,
            max_value=T_MAX,
            value=0.0
        )
        sliders_layout.add_widget(self.torque_slider)

        control_status_layout.add_widget(sliders_layout)

        # Status display
        status_layout = BoxLayout(orientation='vertical', size_hint=(0.3, 1), spacing=10)

        status_title = Label(text="Motor Status", size_hint=(1, 0.1))
        status_layout.add_widget(status_title)

        self.status_display = StatusDisplay(size_hint=(1, 0.5))
        status_layout.add_widget(self.status_display)

        self.log_label = Label(
            text="Log: Ready",
            size_hint=(1, 0.4),
            text_size=(None, None),
            halign='left',
            valign='top'
        )
        status_layout.add_widget(self.log_label)

        control_status_layout.add_widget(status_layout)

        main_layout.add_widget(control_status_layout)

        # Control buttons
        buttons_layout = BoxLayout(orientation='horizontal', size_hint=(1, 0.1), spacing=10)

        self.enter_mode_button = Button(
            text="Enter MIT Mode",
            disabled=True
        )
        self.enter_mode_button.bind(on_press=self.on_enter_mode)
        buttons_layout.add_widget(self.enter_mode_button)

        self.exit_mode_button = Button(
            text="Exit MIT Mode",
            disabled=True
        )
        self.exit_mode_button.bind(on_press=self.on_exit_mode)
        buttons_layout.add_widget(self.exit_mode_button)

        self.zero_position_button = Button(
            text="Zero Position",
            disabled=True
        )
        self.zero_position_button.bind(on_press=self.on_zero_position)
        buttons_layout.add_widget(self.zero_position_button)

        self.send_command_button = Button(
            text="Send Command",
            disabled=True
        )
        self.send_command_button.bind(on_press=self.on_send_command)
        buttons_layout.add_widget(self.send_command_button)

        main_layout.add_widget(buttons_layout)

        # Continuous control
        continuous_layout = BoxLayout(orientation='horizontal', size_hint=(1, 0.1), spacing=10)

        self.continuous_label = Label(
            text="Continuous Control:",
            size_hint=(0.3, 1)
        )
        continuous_layout.add_widget(self.continuous_label)

        self.continuous_checkbox = TextInput(
            text="0.1",
            multiline=False,
            size_hint=(0.3, 1)
        )
        continuous_layout.add_widget(self.continuous_checkbox)

        self.continuous_button = Button(
            text="Start Continuous",
            disabled=True,
            size_hint=(0.4, 1)
        )
        self.continuous_button.bind(on_press=self.toggle_continuous)
        continuous_layout.add_widget(self.continuous_button)

        main_layout.add_widget(continuous_layout)

        # Initialize variables
        self.continuous_active = False
        self.continuous_event = None

        return main_layout

    def log(self, message):
        self.log_label.text = f"Log: {message}"
        print(message)

    def toggle_connection(self, instance):
        if not self.connected:
            # Try to connect
            try:
                channel = self.channel_input.text
                self.log(f"Connecting to {self.can_interface}:{channel}...")
                self.bus = can.interface.Bus(channel=channel, interface=self.can_interface, bitrate=self.bitrate)
                self.connected = True
                self.connect_button.text = "Disconnect"
                self.connect_button.background_color = (0.7, 0.2, 0.2, 1)
                self.status_label.text = "Connected"
                self.status_label.color = (0.2, 0.8, 0.2, 1)

                # Enable buttons
                self.enter_mode_button.disabled = False
                self.exit_mode_button.disabled = False
                self.zero_position_button.disabled = False
                self.send_command_button.disabled = False
                self.continuous_button.disabled = False

                # Start status thread
                self.stop_thread = False
                self.status_thread = threading.Thread(target=self.status_update_thread)
                self.status_thread.daemon = True
                self.status_thread.start()

                self.log("Connected successfully")
            except Exception as e:
                self.log(f"Connection error: {str(e)}")
        else:
            # Disconnect
            self.stop_continuous()
            self.stop_thread = True
            if self.status_thread:
                self.status_thread.join(timeout=1.0)

            if self.bus:
                try:
                    self.bus.shutdown()
                except:
                    pass
                self.bus = None

            self.connected = False
            self.connect_button.text = "Connect"
            self.connect_button.background_color = (0.2, 0.7, 0.2, 1)
            self.status_label.text = "Disconnected"
            self.status_label.color = (1, 0.3, 0.3, 1)

            # Disable buttons
            self.enter_mode_button.disabled = True
            self.exit_mode_button.disabled = True
            self.zero_position_button.disabled = True
            self.send_command_button.disabled = True
            self.continuous_button.disabled = True

            self.log("Disconnected")

    def on_enter_mode(self, instance):
        if self.connected and self.bus:
            if enter_mode(self.bus):
                self.log("Entered MIT Mode")
            else:
                self.log("Failed to enter MIT Mode")

    def on_exit_mode(self, instance):
        if self.connected and self.bus:
            if exit_mode(self.bus):
                self.log("Exited MIT Mode")
            else:
                self.log("Failed to exit MIT Mode")

    def on_zero_position(self, instance):
        if self.connected and self.bus:
            if zero_position(self.bus):
                self.log("Position zeroed")
            else:
                self.log("Failed to zero position")

    def on_send_command(self, instance):
        if self.connected and self.bus:
            # Update motor state from sliders
            self.motor_state.p_in = self.position_slider.value
            self.motor_state.v_in = self.velocity_slider.value
            self.motor_state.kp_in = self.kp_slider.value
            self.motor_state.kd_in = self.kd_slider.value
            self.motor_state.t_in = self.torque_slider.value

            if pack_cmd(self.bus, self.motor_state):
                self.log(f"Command sent: P={self.motor_state.p_in:.2f}, V={self.motor_state.v_in:.2f}, " +
                         f"Kp={self.motor_state.kp_in:.2f}, Kd={self.motor_state.kd_in:.2f}, T={self.motor_state.t_in:.2f}")
            else:
                self.log("Failed to send command")

    def toggle_continuous(self, instance):
        if self.continuous_active:
            self.stop_continuous()
        else:
            try:
                interval = float(self.continuous_checkbox.text)
                if interval < 0.01:
                    interval = 0.01

                self.continuous_active = True
                self.continuous_button.text = "Stop Continuous"
                self.continuous_button.background_color = (0.7, 0.2, 0.2, 1)

                # Schedule the continuous update
                self.continuous_event = Clock.schedule_interval(self.continuous_update, interval)
                self.log(f"Started continuous control at {interval}s interval")
            except ValueError:
                self.log("Invalid interval value")

    def stop_continuous(self):
        if self.continuous_active:
            if self.continuous_event:
                self.continuous_event.cancel()
                self.continuous_event = None

            self.continuous_active = False
            self.continuous_button.text = "Start Continuous"
            self.continuous_button.background_color = (0.2, 0.7, 0.2, 1)
            self.log("Stopped continuous control")

    def continuous_update(self, dt):
        if self.connected and self.bus:
            # Update motor state from sliders
            self.motor_state.p_in = self.position_slider.value
            self.motor_state.v_in = self.velocity_slider.value
            self.motor_state.kp_in = self.kp_slider.value
            self.motor_state.kd_in = self.kd_slider.value
            self.motor_state.t_in = self.torque_slider.value

            pack_cmd(self.bus, self.motor_state)

    def status_update_thread(self):
        while not self.stop_thread:
            if self.connected and self.bus:
                try:
                    msg = self.bus.recv(timeout=0.1)
                    if msg:
                        if unpack_reply(msg, self.motor_state):
                            # Use Clock to update UI from the main thread
                            Clock.schedule_once(lambda dt: self.update_status_display(), 0)
                except:
                    pass
            time.sleep(0.1)

    def update_status_display(self):
        self.status_display.update_values(
            self.motor_state.p_out,
            self.motor_state.v_out,
            self.motor_state.t_out
        )

    def on_stop(self):
        # Clean up resources
        self.stop_continuous()
        self.stop_thread = True
        if self.status_thread:
            self.status_thread.join(timeout=1.0)

        if self.bus:
            try:
                self.bus.shutdown()
            except:
                pass
