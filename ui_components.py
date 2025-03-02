# ui_components.py
# Custom UI components for the motor control application

from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.properties import StringProperty, NumericProperty

class LabeledSlider(BoxLayout):
    """A custom slider with a label and value display"""
    label_text = StringProperty('')
    min_value = NumericProperty(0)
    max_value = NumericProperty(100)
    value = NumericProperty(0)

    def __init__(self, **kwargs):
        super(LabeledSlider, self).__init__(**kwargs)
        self.orientation = 'vertical'

        self.label = Label(text=self.label_text, size_hint=(1, 0.2))
        self.slider = Slider(min=self.min_value, max=self.max_value, value=self.value, size_hint=(1, 0.6))
        self.value_label = Label(text=f"{self.value:.2f}", size_hint=(1, 0.2))

        self.add_widget(self.label)
        self.add_widget(self.slider)
        self.add_widget(self.value_label)

        self.slider.bind(value=self.on_slider_value)

    def on_slider_value(self, instance, value):
        self.value = value
        self.value_label.text = f"{value:.2f}"

class StatusDisplay(GridLayout):
    """A display for showing motor status values"""
    def __init__(self, **kwargs):
        super(StatusDisplay, self).__init__(**kwargs)
        self.cols = 2
        self.padding = 10
        self.spacing = 5

        self.add_widget(Label(text="Position:"))
        self.position_label = Label(text="0.00")
        self.add_widget(self.position_label)

        self.add_widget(Label(text="Velocity:"))
        self.velocity_label = Label(text="0.00")
        self.add_widget(self.velocity_label)

        self.add_widget(Label(text="Torque:"))
        self.torque_label = Label(text="0.00")
        self.add_widget(self.torque_label)

    def update_values(self, position, velocity, torque):
        self.position_label.text = f"{position:.2f}"
        self.velocity_label.text = f"{velocity:.2f}"
        self.torque_label.text = f"{torque:.2f}"
