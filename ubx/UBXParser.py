# Unique UBX sentences
import ubx.Sensors
from ubx.relposned import nav_relposned
from ubx.posllh import nav_posllh
from ubx.secuniqid import sec_uniqid

# Dictionaries of static data
import ubx.ClassID as ubc
import ubx.MessageID as ubm

# List of sensor serial numbers and allocations
from ubx.Sensors import UBLOX


class UBXParser():
    # Constructor
    def __init__(self):

        # Switch this on for verbose processing
        self.debug = 0

        # Timers for each sentence, check to see if data stale when calling this class
        self.posllh_TOW = ''
        self.relposned_TOW = ''

        # Properties used by calling program
        self.unique_id = ''
        self.longitude = 0
        self.latitude = 0
        self.altitude = 0
        self.horizontal_accuracy = 0
        self.vertical_accuracy = 0
        self.heading = 0

        # Status values, set when updated, reset from the calling program
        self.new_position = 0
        self.new_heading = 0

    def ubx_parser(self, byte3, byte4, ubx_payload):
        # For error checking and logging
        this_function = 'UBXParser:ubx_parser'
        # Check if a valid UBX class
        if byte3 in ubc.UBX_CLASS:
            # Check if class = NAV (x01)
            if ubc.UBX_CLASS[byte3] == 'NAV':
                # Check if a valid message
                if byte4 in ubm.UBX_NAV:
                    # Check for NAV-RELPOSNED (x3c)
                    if byte4 == b"\x3c":
                        # Now parse
                        self.heading, self.relposned_TOW = nav_relposned(ubx_payload)
                        # Now let the calling program know there is a new heading
                        self.new_heading = 1
                        if self.debug == 1:
                            print(f'{this_function} UBX-NAV-RELPOSNED {self.heading, self.relposned_TOW}')
                    # Check for NAV-POSLLH (x02)
                    elif byte4 == b"\x02":
                        # Now parse
                        self.longitude, self.latitude, self.altitude, \
                            self.horizontal_accuracy, self.vertical_accuracy,self.posllh_TOW = nav_posllh(ubx_payload)
                        # Now let the calling program know there is a new position
                        self.new_position = 1
                        if self.debug == 1:
                            print(f'{this_function} UBX-NAV-POSLLH {self.longitude, self.latitude, self.altitude, self.posllh_TOW}')
                    else:
                        print(f'{this_function} No message definition for {byte3, byte4}!!')
            # Check if class = SEC (x27)
            if ubc.UBX_CLASS[byte3] == 'SEC':
                if byte4 in ubm.UBX_SEC:
                    # Check for SEC-UNIQID (x03)
                    if byte4 == b"\x03":
                        # Now parse and save the unique ID as a property
                        self.unique_id = sec_uniqid(ubx_payload)

                        if self.debug == 1:
                            if ubx.Sensors.UBLOX[self.unique_id] == "MovingBase":
                                print(f'{this_function} UBX-SEC-UNIQID {self.unique_id} verified as moving base')
                            elif ubx.Sensors.UBLOX[self.unique_id] == "Heading":
                                print(f'{this_function} UBX-SEC-UNIQID {self.unique_id} verified as heading sensor')
                            elif ubx.Sensors.UBLOX[self.unique_id] == "Roll":
                                print(f'{this_function} UBX-SEC-UNIQID {self.unique_id} verified as roll sensor')
                            else:
                                print('Wrong Sensor')
                                exit(-1)
        else:
            print(f'{this_function} No class definition for {byte3}')
