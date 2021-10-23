""""
Main routine for HeadingSensor
Forked from the Comm module of SD-Node, written c. 2017
Takes a serial UBX input, parses and extracts heading information
creates a NMEA sentence to forward the data and
optionally, forwards to a UDP address:port
Tested with Python >=3.6

By: JOR
    v0.1    28AUG21     First draft
    v0.2    12SEP21     Read 2 x UBX and prints to screen
"""

import serial
import sys

# Utilities used by all UBX tools
from ubx.Utilities import ubx_crc, log_file_name
from ubx.UBXParser import UBXParser

# utilities for NMEA tools
from nmea.ths import ths
from nmea.hdt import hdt
from nmea.Utilities import udp_sender

# Utilities to find UBX GNSS
import SerialUtils


print('***** Heading Sensor *****')
print('Accepts mixed UBX-RELPOSNED, UBX_POSLLH from a serial port:')
print('1. Extracts heading, position, accuracy information and logs')
print('2. Outputs a NMEA sentence for other applications to use: only implemented to network in this version).')
print('3. Outputs to an IP address and port for other applications to use.')

# Find the com port for the heading sensor
heading_com_port = SerialUtils.serial_mapper('Heading')
print(f'COM Port {heading_com_port}')

# Instantiate an object to parse UBX
myUBX = UBXParser()

# Get a logfile name for UBX
ubx_log_file = log_file_name('.ubx')
# Flag for logging
logging = 1
# Set UDP multicast information
MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5007


# Configure the serial port
Serial_Port1 = serial.Serial(
    # For Windows
    port=heading_com_port,
    # For RPi
    #port='/dev/ttySC1',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=2
)
Serial_Port1.flushInput()

# Main Loop
try:
    # For error checking and logging
    this_function = 'HeadingSensor'

    print("press [ctrl][c] at any time to exit...")

    # Continuous loop until [ctrl][c]
    while True:
        # Read the first byte, if no byte, loop
        byte1 = Serial_Port1.read(1)
        if len(byte1) <1:
            break

        # Check for UBX header = xB5 and X62, Unicode = µb
        if byte1 == b"\xb5":
            byte2 = Serial_Port1.read(1)
            if len(byte2) < 1:
                break
            if byte2 == b"\x62":
                # Get the UBX class
                byte3 = Serial_Port1.read(1)
                # Get the UBX message
                byte4 = Serial_Port1.read(1)
                # Get the UBX payload length
                byte5and6 = Serial_Port1.read(2)
                # Calculate the length of the payload
                length_of_payload = int.from_bytes(byte5and6, "little", signed=False)
                # Read the buffer for the payload length
                ubx_payload = Serial_Port1.read(length_of_payload)
                # Last two bytes are 2*CRC, save them for later use
                ubx_crc_a = Serial_Port1.read(1)
                ubx_crc_b = Serial_Port1.read(1)
                # Calculate CRC using CLASS + MESSAGE + LENGTH + PAYLOAD
                payload_for_crc = byte3 + byte4 + byte5and6 + ubx_payload
                # If the CRC is good, proceed
                if ubx_crc(payload_for_crc,ubx_crc_a, ubx_crc_b):
                    # If logging flag is on, log
                    if logging == 1:
                        # Log the ubx bytes
                        payload_for_save = byte1 + byte2 + payload_for_crc + ubx_crc_a + ubx_crc_b
                        with open (ubx_log_file, 'ab') as file:
                            file.write(payload_for_save)
                    # Process the ubx bytes
                    myUBX.ubx_parser(byte3, byte4, ubx_payload)
                    # Now see if there are new values
                    if myUBX.new_heading:
                        # Get the heading as a float and round to two places
                        heading = round(myUBX.heading, 4)
                        # Convert heading from float to string
                        heading_string = str(heading)
                        # Create a NMEA sentence
                        nmea_full_ths = ths(heading_string, "A")
                        nmea_full_hdt = hdt(heading_string)
                        print(nmea_full_hdt)
                        # Processed the old heading, reset the flag
                        myUBX.new_heading = 0
                        # Send the heading to a multicast address
                        udp_sender(MCAST_GRP, MCAST_PORT, bytes(nmea_full_ths, 'utf-8'))
                        udp_sender(MCAST_GRP, MCAST_PORT, bytes(nmea_full_hdt, 'utf-8'))
                else:
                    print(f'{this_function} Bad CRC')

        # Check for NMEA0183, leading with a $ symbol
        elif byte1 == b"\x24":
            nmea_full_bytes = Serial_Port1.readline()
            nmea_full_string = nmea_full_bytes.decode("utf-8")
            print(f'{this_function} NMEA: Received {nmea_full_string[0:5]}')

        # Check for AIS, leading with a ! symbol
        elif byte1 == b"\x21":
            nmea_full_bytes = Serial_Port1.readline()
            nmea_full_string = nmea_full_bytes.decode("utf-8")
            print(f'{this_function} AIS: Received {nmea_full_string[0:5]}')

        # Check for RTCM corrections
        elif byte1 == b"\xd3":
            # Find the message length
            byte2and3 = Serial_Port1.read(2)
            # The first 6 bits are reserved, but always zero, so convert the first two bytes directly to int
            length_of_payload = int.from_bytes(byte2and3, "big", signed=False)
            # Read the payload from the buffer
            rtcm_payload = Serial_Port1.read(length_of_payload)
            # Locate the message ID and convert it to an INT, its 12 bits of 16 so divide by 16
            message_id_bytes = rtcm_payload[0:2]
            message_id_int = int.from_bytes(message_id_bytes, "big") / 16
            print(f'RTCM3: Received {str(message_id_int)}')
            # Finally extract the RTCM CRC
            rtcm_crc = Serial_Port1.read(3)
        else:
            print(f"{this_function} What is {byte1}")

except serial.SerialException as err:
    print(f"{this_function} Serial port error: {0}".format(err))
except OSError as err:
    print(f"{this_function} OS error: {0}".format(err))
except ValueError as err:
    print(f"{this_function} Value Error error: {0}".format(err))
except KeyboardInterrupt:
    print("\n" + "Caught keyboard interrupt, exiting")
    exit(0)
except:
    print(f"{this_function} Unexpected error:", sys.exc_info()[0])
finally:
    print("Exiting Main Thread")
    exit(0)

