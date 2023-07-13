# Printer stepper support
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections
import chelper

class error(Exception):
    pass


######################################################################
# Steppers
######################################################################

MIN_BOTH_EDGE_DURATION = 0.000000200

# Interface to low-level mcu and chelper code
class MCU_stepper:
    def __init__(self, name, step_pin_params, dir_pin_params,
                 rotation_dist, steps_per_rotation,
                 step_pulse_duration=None, units_in_radians=False):
        self._name = name
        self._rotation_dist = rotation_dist
        self._steps_per_rotation = steps_per_rotation
        self._step_pulse_duration = step_pulse_duration
        self._units_in_radians = units_in_radians
        self._step_dist = rotation_dist / steps_per_rotation
        self._mcu = step_pin_params['chip']
        self._oid = oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)
        self._step_pin = step_pin_params['pin']
        self._invert_step = step_pin_params['invert']
        if dir_pin_params['chip'] is not self._mcu:
            raise self._mcu.get_printer().config_error(
                "Stepper dir pin must be on same mcu as step pin")
        self._dir_pin = dir_pin_params['pin']
        self._invert_dir = self._orig_invert_dir = dir_pin_params['invert']
        self._step_both_edge = self._req_step_both_edge = False
        self._mcu_position_offset = 0.
        self._reset_cmd_tag = self._get_position_cmd = None
        self._active_callbacks = []
        ffi_main, ffi_lib = chelper.get_ffi()
        self._stepqueue = ffi_main.gc(ffi_lib.stepcompress_alloc(oid),
                                      ffi_lib.stepcompress_free)
        ffi_lib.stepcompress_set_invert_sdir(self._stepqueue, self._invert_dir)
        self._mcu.register_stepqueue(self._stepqueue)
        self._stepper_kinematics = None
        self._itersolve_generate_steps = ffi_lib.itersolve_generate_steps
        self._itersolve_check_active = ffi_lib.itersolve_check_active
        self._trapq = ffi_main.NULL
        self._mcu.get_printer().register_event_handler('klippy:connect',
                                                       self._query_mcu_position)
    def get_mcu(self):
        return self._mcu
    def get_name(self, short=False):
        if short and self._name.startswith('stepper_'):
            return self._name[8:]
        return self._name
    def units_in_radians(self):
        # Returns true if distances are in radians instead of millimeters
        return self._units_in_radians
    def get_pulse_duration(self):
        return self._step_pulse_duration, self._step_both_edge
    def setup_default_pulse_duration(self, pulse_duration, step_both_edge):
        if self._step_pulse_duration is None:
            self._step_pulse_duration = pulse_duration
        self._req_step_both_edge = step_both_edge
    def setup_itersolve(self, alloc_func, *params):
        # NOTE: if for example the "alloc_func" is "polar_stepper_alloc"
        #       (from "kin_polar.c") that function will be called to
        #       produce a "stepper_kinematics" C object of type "r" or "a"
        #       ("radius" and "angle", respectively).
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        self.set_stepper_kinematics(sk)
    def _build_config(self):
        if self._step_pulse_duration is None:
            self._step_pulse_duration = .000002
        invert_step = self._invert_step
        sbe = int(self._mcu.get_constants().get('STEPPER_BOTH_EDGE', '0'))
        if (self._req_step_both_edge and sbe
            and self._step_pulse_duration <= MIN_BOTH_EDGE_DURATION):
            # Enable stepper optimized step on both edges
            self._step_both_edge = True
            self._step_pulse_duration = 0.
            invert_step = -1
        step_pulse_ticks = self._mcu.seconds_to_clock(self._step_pulse_duration)
        self._mcu.add_config_cmd(
            "config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d"
            " step_pulse_ticks=%u" % (self._oid, self._step_pin, self._dir_pin,
                                      invert_step, step_pulse_ticks))
        self._mcu.add_config_cmd("reset_step_clock oid=%d clock=0"
                                 % (self._oid,), on_restart=True)
        step_cmd_tag = self._mcu.lookup_command(
            "queue_step oid=%c interval=%u count=%hu add=%hi").get_command_tag()
        dir_cmd_tag = self._mcu.lookup_command(
            "set_next_step_dir oid=%c dir=%c").get_command_tag()
        self._reset_cmd_tag = self._mcu.lookup_command(
            "reset_step_clock oid=%c clock=%u").get_command_tag()
        self._get_position_cmd = self._mcu.lookup_query_command(
            "stepper_get_position oid=%c",
            "stepper_position oid=%c pos=%i", oid=self._oid)
        max_error = self._mcu.get_max_stepper_error()
        max_error_ticks = self._mcu.seconds_to_clock(max_error)
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.stepcompress_fill(self._stepqueue, max_error_ticks,
                                  step_cmd_tag, dir_cmd_tag)
    def get_oid(self):
        return self._oid
    def get_step_dist(self):
        return self._step_dist
    def get_rotation_distance(self):
        return self._rotation_dist, self._steps_per_rotation
    def set_rotation_distance(self, rotation_dist):
        mcu_pos = self.get_mcu_position()
        self._rotation_dist = rotation_dist
        self._step_dist = rotation_dist / self._steps_per_rotation
        self.set_stepper_kinematics(self._stepper_kinematics)
        self._set_mcu_position(mcu_pos)
    def get_dir_inverted(self):
        return self._invert_dir, self._orig_invert_dir
    def set_dir_inverted(self, invert_dir):
        invert_dir = not not invert_dir
        if invert_dir == self._invert_dir:
            return
        self._invert_dir = invert_dir
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.stepcompress_set_invert_sdir(self._stepqueue, invert_dir)
        self._mcu.get_printer().send_event("stepper:set_dir_inverted", self)
    def calc_position_from_coord(self, coord):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.itersolve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2])
    
    def set_position(self, coord):
        logging.info("\n\n" + f"MCU_stepper.set_position: setting coord={coord} on stepper={self._name}\n\n")
        # NOTE: reads current position from "get_commanded_position()",
        #       adds the "_mcu_position_offset" and converts to position
        #       dividing by "_step_dist".
        mcu_pos = self.get_mcu_position()

        sk = self._stepper_kinematics
        ffi_main, ffi_lib = chelper.get_ffi()

        # NOTE: "itersolve_set_position" sets "sk->commanded_pos" (at itersolve.c)
        logging.info("\n\n" + f"MCU_stepper.set_position: calling itersolve_set_position\n\n")
        ffi_lib.itersolve_set_position(sk, coord[0], coord[1], coord[2])

        # NOTE: "_set_mcu_position" uses "self.get_commanded_position" 
        #       and "itersolve_get_commanded_pos" to read "sk->commanded_pos" 
        #       (at itersolve.c), which has just been set above,
        #       and updates "self._mcu_position_offset".
        logging.info("\n\n" + f"MCU_stepper.set_position: calling _set_mcu_position\n\n")
        self._set_mcu_position(mcu_pos)
    
    def get_commanded_position(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        # NOTE: This probably reads the position previously
        #       set by "set_position"/"itersolve_set_position"
        return ffi_lib.itersolve_get_commanded_pos(self._stepper_kinematics)
    
    def get_mcu_position(self):
        mcu_pos_dist = self.get_commanded_position() + self._mcu_position_offset
        mcu_pos = mcu_pos_dist / self._step_dist
        # TODO: find out what "0.5" means:
        if mcu_pos >= 0.:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)
    
    def _set_mcu_position(self, mcu_pos):
        mcu_pos_dist = mcu_pos * self._step_dist
        # TODO: find out what "self._mcu_position_offset" is.
        # NOTE: "get_commanded_position" reads "sk->commanded_pos" using
        #       the "itersolve_get_commanded_pos" function (at itersolve.c)
        self._mcu_position_offset = mcu_pos_dist - self.get_commanded_position()
    
    def get_past_mcu_position(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        # NOTE: "Search history of moves to find a past position at a given clock"
        pos = ffi_lib.stepcompress_find_past_position(self._stepqueue, clock)
        return int(pos)
    
    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self._step_dist - self._mcu_position_offset
    def dump_steps(self, count, start_clock, end_clock):
        ffi_main, ffi_lib = chelper.get_ffi()
        data = ffi_main.new('struct pull_history_steps[]', count)
        count = ffi_lib.stepcompress_extract_old(self._stepqueue, data, count,
                                                 start_clock, end_clock)
        return (data, count)
    def get_stepper_kinematics(self):
        return self._stepper_kinematics
    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        mcu_pos = 0
        if old_sk is not None:
            mcu_pos = self.get_mcu_position()
        self._stepper_kinematics = sk
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.itersolve_set_stepcompress(sk, self._stepqueue, self._step_dist)
        self.set_trapq(self._trapq)
        self._set_mcu_position(mcu_pos)
        return old_sk
    
    def note_homing_end(self):

        ffi_main, ffi_lib = chelper.get_ffi()
        # NOTE: - stepcompress_reset:   "Reset the internal state of the stepcompress object"
        #       - stepcompress_flush:   "Flush pending steps"
        #       - queue_flush:          "Convert previously scheduled steps into commands for the mcu"
        ret = ffi_lib.stepcompress_reset(self._stepqueue, 0)
        if ret:
            raise error("Internal error in stepcompress")
        data = (self._reset_cmd_tag, self._oid, 0)
        ret = ffi_lib.stepcompress_queue_msg(self._stepqueue, data, len(data))
        if ret:
            raise error("Internal error in stepcompress")
        self._query_mcu_position()
    
    def _query_mcu_position(self):
        if self._mcu.is_fileoutput():
            return
        params = self._get_position_cmd.send([self._oid])
        last_pos = params['pos']
        if self._invert_dir:
            last_pos = -last_pos
        print_time = self._mcu.estimated_print_time(params['#receive_time'])
        clock = self._mcu.print_time_to_clock(print_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        ret = ffi_lib.stepcompress_set_last_position(self._stepqueue, clock,
                                                     last_pos)
        if ret:
            raise error("Internal error in stepcompress")
        self._set_mcu_position(last_pos)
        self._mcu.get_printer().send_event("stepper:sync_mcu_position", self)
    def get_trapq(self):
        return self._trapq
    def set_trapq(self, tq):
        ffi_main, ffi_lib = chelper.get_ffi()
        if tq is None:
            tq = ffi_main.NULL
        ffi_lib.itersolve_set_trapq(self._stepper_kinematics, tq)
        old_tq = self._trapq
        self._trapq = tq
        return old_tq
    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)
    def generate_steps(self, flush_time):
        # Check for activity if necessary
        if self._active_callbacks:
            sk = self._stepper_kinematics
            ret = self._itersolve_check_active(sk, flush_time)
            if ret:
                cbs = self._active_callbacks
                self._active_callbacks = []
                for cb in cbs:
                    cb(ret)
        # Generate step times for a range of moves on the trapq
        sk = self._stepper_kinematics
        ret = self._itersolve_generate_steps(sk, flush_time)
        if ret:
            raise error("Internal error in stepcompress")
    def is_active_axis(self, axis):
        ffi_main, ffi_lib = chelper.get_ffi()
        a = axis.encode()
        return ffi_lib.itersolve_is_active_axis(self._stepper_kinematics, a)
    
    def get_steppers(self):
        # NOTE: dummy method for "_handle_mcu_identify" at "probe_G38.py".
        return [self]


# Helper code to build a stepper object from a config section
def PrinterStepper(config, units_in_radians=False):
    printer = config.get_printer()
    name = config.get_name()  # NOTE: Example: "stepper_x".
    # Stepper definition
    ppins = printer.lookup_object('pins')
    step_pin = config.get('step_pin')
    step_pin_params = ppins.lookup_pin(step_pin, can_invert=True)
    dir_pin = config.get('dir_pin')
    dir_pin_params = ppins.lookup_pin(dir_pin, can_invert=True)
    rotation_dist, steps_per_rotation = parse_step_distance(
        config, units_in_radians, True)
    step_pulse_duration = config.getfloat('step_pulse_duration', None,
                                          minval=0., maxval=.001)
    mcu_stepper = MCU_stepper(name, step_pin_params, dir_pin_params,
                              rotation_dist, steps_per_rotation,
                              step_pulse_duration, units_in_radians)
    # Register with helper modules
    for mname in ['stepper_enable', 'force_move', 'motion_report']:
        m = printer.load_object(config, mname)
        m.register_stepper(config, mcu_stepper)
    return mcu_stepper

# Parse stepper gear_ratio config parameter
def parse_gear_ratio(config, note_valid):
    gear_ratio = config.getlists('gear_ratio', (), seps=(':', ','), count=2,
                                 parser=float, note_valid=note_valid)
    result = 1.
    for g1, g2 in gear_ratio:
        result *= g1 / g2
    return result

# Obtain "step distance" information from a config section
def parse_step_distance(config, units_in_radians=None, note_valid=False):
    if units_in_radians is None:
        # Caller doesn't know if units are in radians - infer it
        rd = config.get('rotation_distance', None, note_valid=False)
        gr = config.get('gear_ratio', None, note_valid=False)
        units_in_radians = rd is None and gr is not None
    if units_in_radians:
        rotation_dist = 2. * math.pi
        config.get('gear_ratio', note_valid=note_valid)
    else:
        rotation_dist = config.getfloat('rotation_distance', above=0.,
                                        note_valid=note_valid)
    # Newer config format with rotation_distance
    microsteps = config.getint('microsteps', minval=1, note_valid=note_valid)
    full_steps = config.getint('full_steps_per_rotation', 200, minval=1,
                               note_valid=note_valid)
    if full_steps % 4:
        raise config.error("full_steps_per_rotation invalid in section '%s'"
                           % (config.get_name(),))
    gearing = parse_gear_ratio(config, note_valid)
    return rotation_dist, full_steps * microsteps * gearing


######################################################################
# Stepper controlled rails
######################################################################

# A motor control "rail" with one (or more) steppers and one (or more)
# endstops.
# NOTE: in this branch, this class can also be used to create a stepper
#       for the ExtruderStepper class.
class PrinterRail:
    def __init__(self, config, need_position_minmax=True,
                 default_position_endstop=None, units_in_radians=False):
        # Primary stepper and endstop
        self.stepper_units_in_radians = units_in_radians
        
        # NOTE: List of MCU_stepper objects (setup by PrinterStepper).
        self.steppers = []
        
        # NOTE: list of tuples with elements: (mcu_endstop, name)
        #       mcu_endstop: likely an instance of MCU_endstop.
        #       name: name of the associated stepper.
        self.endstops = []
        
        # TODO: not sure what this is yet.
        self.endstop_map = {}

        # NOTE: add_extra_stepper creates a "stepper" object from the
        #       "PrinterStepper" function, and adds it to the "self.steppers" list.
        #       Internally, the PrinterStepper function instantiates an
        #       "MCU_stepper" class, registers it in several modules,
        #       and returns it.
        #       It then handles the "setup" of the associated
        #       endstop into an MCU_endstop class, and also adds
        #       the stepper to this class.
        self.add_extra_stepper(config)
        
        # NOTE: this grabs the first "MCU_stepper" item in the list,
        #       which was added by the call to "add_extra_stepper" above.
        mcu_stepper = self.steppers[0]
        
        # NOTE: The get_name function is inherited from the
        #       first stepper in the steppers list of the
        #       PrinterRail class. It thus keeps only the first
        #       one. I imagine something like this:
        #       Keep "stepper_x" from ["stepper_x", "stepper_x1"]
        #       The "mcu_stepper.get_name" function will return
        #       "stepper_x" unless "short=True", in which case it
        #       will return just "x".
        self.get_name = mcu_stepper.get_name
        # TODO: I don't know what these do yet.
        self.get_commanded_position = mcu_stepper.get_commanded_position
        self.calc_position_from_coord = mcu_stepper.calc_position_from_coord
        
        # Primary endstop position
        mcu_endstop = self.endstops[0][0]
        if hasattr(mcu_endstop, "get_position_endstop"):
            self.position_endstop = mcu_endstop.get_position_endstop()
        elif default_position_endstop is None:
            self.position_endstop = config.getfloat('position_endstop')
        else:
            self.position_endstop = config.getfloat(
                'position_endstop', default_position_endstop)
        # Axis range
        if need_position_minmax:
            self.position_min = config.getfloat('position_min', 0.)
            self.position_max = config.getfloat(
                'position_max', above=self.position_min)
        else:
            self.position_min = 0.
            self.position_max = self.position_endstop
        if (self.position_endstop < self.position_min
            or self.position_endstop > self.position_max):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % config.get_name())
        # Homing mechanics
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.second_homing_speed = config.getfloat(
            'second_homing_speed', self.homing_speed/2., above=0.)
        self.homing_retract_speed = config.getfloat(
            'homing_retract_speed', self.homing_speed, above=0.)
        self.homing_retract_dist = config.getfloat(
            'homing_retract_dist', 5., minval=0.)
        self.homing_positive_dir = config.getboolean(
            'homing_positive_dir', None)
        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'"
                    % (config.get_name(),))
            config.getboolean('homing_positive_dir', self.homing_positive_dir)
        elif ((self.homing_positive_dir
               and self.position_endstop == self.position_min)
              or (not self.homing_positive_dir
                  and self.position_endstop == self.position_max)):
            raise config.error(
                "Invalid homing_positive_dir / position_endstop in '%s'"
                % (config.get_name(),))
    def get_range(self):
        return self.position_min, self.position_max
    def get_homing_info(self):
        homing_info = collections.namedtuple('homing_info', [
            'speed', 'position_endstop', 'retract_speed', 'retract_dist',
            'positive_dir', 'second_homing_speed'])(
                self.homing_speed, self.position_endstop,
                self.homing_retract_speed, self.homing_retract_dist,
                self.homing_positive_dir, self.second_homing_speed)
        return homing_info
    def get_steppers(self):
        return list(self.steppers)
    
    def get_endstops(self):
        # NOTE: as commented below, endstops in this list are 
        #       likely instances of the MCU_endstop class.
        return list(self.endstops)
    
    def add_extra_stepper(self, config):
        # NOTE: use the PrinterStepper function to instantiate
        #       a new MCU_stepper class, then register it in several modules,
        #       and return it here.
        stepper = PrinterStepper(config, self.stepper_units_in_radians)
        self.steppers.append(stepper)

        # NOTE: Check if self.endstops has been populated, 
        #       initially its empty "[]".
        #       If and endstop has been added,
        #       and no 'endstop_pin' was defined in the config,
        #       then "use the primary endstop".
        if self.endstops and config.get('endstop_pin', None) is None:
            # No endstop defined - use primary endstop
            self.endstops[0][0].add_stepper(stepper)
            return
        
        endstop_pin = config.get('endstop_pin')
        printer = config.get_printer()

        # NOTE: Get object from pins.py
        ppins = printer.lookup_object('pins')

        # NOTE: calls a PrinterPins method from pins.py,
        #       which does "Pin to chip mapping".
        #       It returns a dict with some properties.
        pin_params = ppins.parse_pin(endstop_pin,
                                     can_invert=True,
                                     can_pullup=True)
        
        # Normalize pin name
        pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        
        
        # NOTE: get() method from dict:
        #       "Return the value for key if key is in the dictionary, else default."
        # NOTE: Though ".get" can take two arguments, and they have names in the docs,
        #       using the names will produce errors.
        endstop = self.endstop_map.get(pin_name, None)
        
        # Look for already-registered endstop
        if endstop is None:
            # New endstop, register it
            
            # TODO: I don't really get what this does yet.
            # NOTE: It uses "lookup_pin" which registers an active pin.
            #       It also calls the "setup_pin" method on a "chip" object. 
            #       The chip object comes from a call to "register_chip" elsewhere.
            #       In mcu.py, the MCU class passes itself to this method. 
            #       So... the chips may be MCUs.
            # NOTE: as commented in pins.py (L136), mcu_endstop declared
            #       here is likely an instance of the MCU_endstop class
            #       in "mcu.py".
            mcu_endstop = ppins.setup_pin(pin_type='endstop', pin_desc=endstop_pin)
            
            # NOTE: I don't really get what this does yet.
            #       Add the endstop to the "endstop_map" class dict.
            self.endstop_map[pin_name] = {'endstop': mcu_endstop,
                                          'invert': pin_params['invert'],
                                          'pullup': pin_params['pullup']}
            
            # TODO: I don't really get what this does yet.
            # NOTE: Add the endstop to the "endstops" class list.
            #       As commented above, mcu_endstop is likely
            #       an instance of the MCU_endstop class, as
            #       defined in "mcu.py".
            name = stepper.get_name(short=True)
            self.endstops.append((mcu_endstop, name))
            
            # Load the "query_endstops" module.
            query_endstops = printer.load_object(config, 'query_endstops')
            # Register the endstop there.
            query_endstops.register_endstop(mcu_endstop, name)
        else:
            # endstop already registered
            # NOTE: check if the invert or pull-up pins were
            #       configured differently, and raise an error
            #       if they are.
            mcu_endstop = endstop['endstop']
            changed_invert = pin_params['invert'] != endstop['invert']
            changed_pullup = pin_params['pullup'] != endstop['pullup']
            if changed_invert or changed_pullup:
                raise error("Pinter rail %s shared endstop pin %s "
                            "must specify the same pullup/invert settings" % (
                                self.get_name(), pin_name))
        # NOTE: call the "add_stepper" method from the
        #       MCU_endstop class, which in turn calls
        #       the "trsync.add_stepper" method from the
        #       MCU_trsync class, which simply appends
        #       "stepper" object to a list of steppers.
        mcu_endstop.add_stepper(stepper)
    
    def setup_itersolve(self, alloc_func, *params):
        for stepper in self.steppers:
            stepper.setup_itersolve(alloc_func, *params)
    def generate_steps(self, flush_time):
        for stepper in self.steppers:
            stepper.generate_steps(flush_time)
    def set_trapq(self, trapq):
        for stepper in self.steppers:
            stepper.set_trapq(trapq)
    def set_position(self, coord):
        for stepper in self.steppers:
            # NOTE: calls "MCU_stepper.set_position".
            stepper.set_position(coord)

# Wrapper for dual stepper motor support
def LookupMultiRail(config, need_position_minmax=True,
                    default_position_endstop=None, units_in_radians=False):
    rail = PrinterRail(config, need_position_minmax,
                       default_position_endstop, units_in_radians)
    for i in range(1, 99):
        if not config.has_section(config.get_name() + str(i)):
            break
        rail.add_extra_stepper(config.getsection(config.get_name() + str(i)))
    return rail
