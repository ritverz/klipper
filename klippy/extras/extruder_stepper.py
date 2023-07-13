# Code for supporting multiple steppers in single filament extruder.
#
# Copyright (C) 2019 Simo Apell <simo.apell@live.fi>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from kinematics import extruder

# NOTE: this class is loaded when an "extruder_stepper" section is defined in a config file.
#       See: https://www.klipper3d.org/Config_Reference.html?h=mcu#extruder_stepper

class PrinterExtruderStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        
        # NOTE: defines an extruder stepper from the config section.
        #       During initialization of the "ExtruderStepper" class,
        #       a stepper is created using the same "PrinterStepper"
        #       function used by "PrinterRail" (in stepper.py).
        #
        self.extruder_stepper = extruder.ExtruderStepper(config)
        
        self.extruder_name = config.get('extruder')
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
    def handle_connect(self):
        self.extruder_stepper.sync_to_extruder(self.extruder_name)
    def get_status(self, eventtime):
        return self.extruder_stepper.get_status(eventtime)

def load_config_prefix(config):
    return PrinterExtruderStepper(config)
