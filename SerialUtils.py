from serial.tools.list_ports import comports
import serial
import sys

# Utilities used by all UBX tools
from ubx.Utilities import ubx_crc, log_file_name
from ubx.UBXParser import UBXParser
from ubx.secuniqid import sec_uniqid
import ubx.Sensors

# Instantiate an object to parse UBX
myUBX = UBXParser()

debug = 0

# Define what we are looking for as decimal numbers
ubx_vid = 5446
ubx_pid = 425


def serial_mapper(ubx_device_function):
    """
    Identify all the serial ports and iterate through them. In this version, also call the UBX serial_poller.

    Parameters:
        ubx_device_function : string
        A valid device function from the ubx/Sensors.py/UBLOX

    Returns:
        port_name : string
        This is the COM port the device was found on.
    """

    # For error checking and logging
    this_function = 'SerialUtilts:serial_mapper'
    # List the comm ports
    try:
        # Create a list of comports
        ports = comports()

        # Now iterate through all the ports to find a particular sensor type
        for port in ports:
            # Print all information for debug purposes
            if debug == 1:
                for port, desc, hwid in ports:
                    print(f'{this_function} Port: {port}')
                    print(f'{this_function} Description: {desc}')
                    print(f'{this_function} Hardware ID: {hwid}')
            # Find a UBX, the only sure way is a combination of VID and PID
            if port.vid == ubx_vid and port.pid == ubx_pid:
                # As you iterate, find the unique id of the UBX
                unique_id = serial_poller(port.name)
                # Find the device function, there could be up to three UBX sensors
                if ubx.Sensors.UBLOX[unique_id] == ubx_device_function:
                    print(f'{this_function} UBX-SEC-UNIQID {unique_id} verified as {ubx_device_function}')
                    return port.name
    except serial.SerialException as err:
        print("Serial port error: {0}".format(err))
    except OSError as err:
        print("OS error: {0}".format(err))
    except ValueError as err:
        print("Value Error error: {0}".format(err))
    except KeyboardInterrupt:
        print("\n" + "Caught keyboard interrupt, exiting")
        exit(0)
    except:
        print("Unexpected error:", sys.exc_info()[0])


def serial_poller(port_name):
    """
        Query a UBX device for its serial number

        Parameters:
            port_name : string
            This is the COM port the device to be polled is on.

        Returns:
            uniqueid : string
            This is the UBX unique id for the GNSS chip
        """

    try:
        # Configure the serial port
        Serial_Port1 = serial.Serial(
            port=port_name,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2
        )
        Serial_Port1.flushInput()

        # Find the serial number of the UBlox device, send the query, it will be the first sentence back
        ubx_sec_uniqid_query = b'\xB5\x62\x27\x03\x00\x00\x2A\xA5'
        Serial_Port1.write(ubx_sec_uniqid_query)

        # Continuous loop you find a serial number
        while True:
            # Read the first byte, if no byte, loop
            byte1 = Serial_Port1.read(1)
            if len(byte1) < 1:
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
                    if ubx_crc(payload_for_crc, ubx_crc_a, ubx_crc_b):
                        # Process the ubx bytes
                        myUBX.ubx_parser(byte3, byte4, ubx_payload)
                        unique_id = sec_uniqid(ubx_payload)
                        # Close the port before exit
                        Serial_Port1.close()
                        return unique_id

            # Check for NMEA0183, leading with a $ symbol
            if byte1 == b"\x24":
                nmea_full_bytes = Serial_Port1.readline()
                if debug == 1:
                    print('NMEA')

            # Check for AIS, leading with a ! symbol
            if byte1 == b"\x21":
                nmea_full_bytes = Serial_Port1.readline()
                if debug == 1:
                    print('AIS')

    except serial.SerialException as err:
        print("Serial port error: {0}".format(err))
    except OSError as err:
        print("OS error: {0}".format(err))
    except ValueError as err:
        print("Value Error error: {0}".format(err))
    except KeyboardInterrupt:
        print("\n" + "Caught keyboard interrupt, exiting")
        exit(0)
    except:
        print("SerialUtils: Unexpected error:", sys.exc_info()[0])

