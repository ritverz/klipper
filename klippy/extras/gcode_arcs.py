# adds support fro ARC commands via G2/G3
#
# Copyright (C) 2019  Aleksej Vasiljkovic <achmed21@gmail.com>
#
# function planArc() originates from https://github.com/MarlinFirmware/Marlin
# Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

# Coordinates created by this are converted into G1 commands.
#
# supports XY, XZ & YZ planes with remaining axis as helical

class ArcSupport:

    def __init__(self, config):
        """Support for gcode arc (G2/G3) commands.
        [gcode_arcs]
        #resolution: 1.0
        #   An arc will be split into segments. Each segment's length will
        #   equal the resolution in mm set above. Lower values will produce a
        #   finer arc, but also more work for your machine. Arcs smaller than
        #   the configured value will become straight lines. The default is
        #   1 mm.

        To start adding support for multi-axis, this reads the 'axis' parameter
        of the '[printer]' section in the config file:
        [printer]
        # ...
        axis: XYZ  # Optional: XYZ or XYZABC
        # ...
        """
        self.printer = config.get_printer()
        self.mm_per_arc_segment = config.getfloat('resolution', 1., above=0.0)

        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.gcode = self.printer.lookup_object('gcode')
        
        # Get amount of axes
        # NOTE: Amount of non-extruder axes: XYZ=3, XYZABC=6.
        self.axis_names = config.getsection("printer").get('axis', 'XYZ')  # "XYZ" / "XYZABC"
        self.axis_count = len(self.axis_names)

        # Enum
        self.ARC_PLANE_X_Y = 0
        self.ARC_PLANE_X_Z = 1
        self.ARC_PLANE_Y_Z = 2

        # Enum
        self.X_AXIS = 0
        self.Y_AXIS = 1
        self.Z_AXIS = 2
        self.E_AXIS = self.axis_count  # NOTE: Not used below.
        
        # Arc Move Clockwise.
        self.gcode.register_command("G2", self.cmd_G2)
        
        # Arc Move Counter-clockwise.
        self.gcode.register_command("G3", self.cmd_G3)
        
        # Arc Plane Select: G17 (XY plane), G18 (XZ plane), G19 (YZ plane).
        self.gcode.register_command("G17", self.cmd_G17)
        self.gcode.register_command("G18", self.cmd_G18)
        self.gcode.register_command("G19", self.cmd_G19)

        # This is a named tuple with elements: ('x', 'y', 'z', 'e', 'a', 'b', 'c')
        # Values default to None.
        self.Coord = self.gcode.Coord

        # backwards compatibility, prior implementation only supported XY
        self.plane = self.ARC_PLANE_X_Y

    def cmd_G2(self, gcmd):
        """Arc Move Clockwise: G2 [X<pos>] [Y<pos>] [Z<pos>] [E<pos>] [F<speed>] I<value> J<value>|I<value> K<value>|J<value> K<value>"""
        self._cmd_inner(gcmd, True)

    def cmd_G3(self, gcmd):
        """Arc Move Counter-clockwise: G3 [X<pos>] [Y<pos>] [Z<pos>] [E<pos>] [F<speed>] I<value> J<value>|I<value> K<value>|J<value> K<value>"""
        self._cmd_inner(gcmd, False)

    def cmd_G17(self, gcmd):
        """Arc Plane Select: G17 (XY plane)"""
        self.plane = self.ARC_PLANE_X_Y

    def cmd_G18(self, gcmd):
        """Arc Plane Select: G18 (XZ plane)"""
        self.plane = self.ARC_PLANE_X_Z

    def cmd_G19(self, gcmd):
        """Arc Plane Select: G19 (YZ plane)"""
        self.plane = self.ARC_PLANE_Y_Z

    def _cmd_inner(self, gcmd, clockwise):
        # The arc's path is planned in absolute coordinates.
        gcodestatus = self.gcode_move.get_status()
        if not gcodestatus['absolute_coordinates']:
            raise gcmd.error("G2/G3 does not support relative move mode")
        currentPos = gcodestatus['gcode_position']

        # Parse parameters
        asTarget = self.Coord(x=gcmd.get_float("X", currentPos[0]),
                              y=gcmd.get_float("Y", currentPos[1]),
                              z=gcmd.get_float("Z", currentPos[2]),
                              e=None)

        if gcmd.get_float("R", None) is not None:
            raise gcmd.error("G2/G3 does not support R moves")

        # determine the plane coordinates and the helical axis
        asPlanar = [ gcmd.get_float(a, 0.) for i,a in enumerate('IJ') ]
        axes = (self.X_AXIS, self.Y_AXIS, self.Z_AXIS)
        if self.plane == self.ARC_PLANE_X_Z:
            asPlanar = [ gcmd.get_float(a, 0.) for i,a in enumerate('IK') ]
            axes = (self.X_AXIS, self.Z_AXIS, self.Y_AXIS)
        elif self.plane == self.ARC_PLANE_Y_Z:
            asPlanar = [ gcmd.get_float(a, 0.) for i,a in enumerate('JK') ]
            axes = (self.Y_AXIS, self.Z_AXIS, self.X_AXIS)

        if not (asPlanar[0] or asPlanar[1]):
            raise gcmd.error("G2/G3 requires IJ, IK or JK parameters")

        asE = gcmd.get_float("E", None)
        asF = gcmd.get_float("F", None)

        # Build list of linear coordinates to move
        coords = self.planArc(currentPos=currentPos, 
                              targetPos=asTarget, 
                              offset=asPlanar,
                              clockwise=clockwise,
                              # Expand the axes list to pass its values to: "alpha_axis", "beta_axis", "helical_axis"
                              *axes)
        e_per_move = e_base = 0.
        if asE is not None:
            if gcodestatus['absolute_extrude']:
                e_base = currentPos[3]
            e_per_move = (asE - e_base) / len(coords)

        # Convert coords into G1 commands
        for coord in coords:
            g1_params = {'X': coord[0], 'Y': coord[1], 'Z': coord[2]}
            if e_per_move:
                g1_params['E'] = e_base + e_per_move
                if gcodestatus['absolute_extrude']:
                    e_base += e_per_move
            if asF is not None:
                g1_params['F'] = asF
            g1_gcmd = self.gcode.create_gcode_command("G1", "G1", g1_params)
            self.gcode_move.cmd_G1(g1_gcmd)

    # function planArc() originates from marlin plan_arc()
    # https://github.com/MarlinFirmware/Marlin
    #
    # The arc is approximated by generating many small linear segments.
    # The length of each segment is configured in MM_PER_ARC_SEGMENT
    # Arcs smaller then this value, will be a Line only
    #
    # alpha and beta axes are the current plane, helical axis is linear travel
    def planArc(self, currentPos, targetPos, offset, clockwise,
                alpha_axis, beta_axis, helical_axis):
        # todo: sometimes produces full circles

        # Radius vector from center to current location
        r_P = -offset[0]
        r_Q = -offset[1]

        # Determine angular travel
        center_P = currentPos[alpha_axis] - r_P
        center_Q = currentPos[beta_axis] - r_Q
        rt_Alpha = targetPos[alpha_axis] - center_P
        rt_Beta = targetPos[beta_axis] - center_Q
        angular_travel = math.atan2(r_P * rt_Beta - r_Q * rt_Alpha,
                                    r_P * rt_Alpha + r_Q * rt_Beta)
        if angular_travel < 0.:
            angular_travel += 2. * math.pi
        if clockwise:
            angular_travel -= 2. * math.pi

        if (angular_travel == 0.
            and currentPos[alpha_axis] == targetPos[alpha_axis]
            and currentPos[beta_axis] == targetPos[beta_axis]):
            # Make a circle if the angular rotation is 0 and the
            # target is current position
            angular_travel = 2. * math.pi

        # Determine number of segments
        linear_travel = targetPos[helical_axis] - currentPos[helical_axis]
        radius = math.hypot(r_P, r_Q)
        flat_mm = radius * angular_travel
        if linear_travel:
            mm_of_travel = math.hypot(flat_mm, linear_travel)
        else:
            mm_of_travel = math.fabs(flat_mm)
        segments = max(1., math.floor(mm_of_travel / self.mm_per_arc_segment))

        # Generate coordinates
        theta_per_segment = angular_travel / segments
        linear_per_segment = linear_travel / segments
        coords = []
        for i in range(1, int(segments)):
            dist_Helical = i * linear_per_segment
            cos_Ti = math.cos(i * theta_per_segment)
            sin_Ti = math.sin(i * theta_per_segment)
            r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti
            r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti

            # Coord is a named tuple with elements: ('x', 'y', 'z', 'e', 'a', 'b', 'c')
            # Its values default to None.
            # Coord doesn't support index assignment, create list.
            # NOTE: Using "axis_count" (e.g. can be "3" for an XYZ setup). Adding 1 to consider the Extruder axis. 
            #       This achieves backwardcompatibility.
            c = [None for i in range(self.axis_count + 1)]
            c[alpha_axis] = center_P + r_P
            c[beta_axis] = center_Q + r_Q
            c[helical_axis] = currentPos[helical_axis] + dist_Helical
            coords.append(self.Coord(*c))

        coords.append(targetPos)
        return coords

def load_config(config):
    return ArcSupport(config)
