import os
import platform
import numpy as np
from datetime import datetime

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames

class Battery:    
    """
    Klasse voor het uitlezen en interpreteren van batterijgegevens van de robot.

    De Battery-klasse ontvangt ruwe data via de bridge_manager en zet deze om naar
    bruikbare grootheden zoals spanning, stroom en batterijstatus. Ook kan een
    waarschuwing worden gegenereerd wanneer de batterij bijna leeg is.
    """

    # BMS switches off at 12000 mV (4 * 3.00 V)
    # That is very low. If stored in that state for a longer time, it will die
    # Set SW limits to 3.25 * 4 = 13000 mV
    #: test
    state_names = ["Empty", "Error", "Unknown", "Discharging", "Charging"]
    batterystate: int
    '''Huidige batterijstatus (bijv. Battery.CHARGE). Wordt aangepast bij elke `new_data`-oproep.'''
    oldstate: int
    '''Vorige batterijstatus, om veranderingen te kunnen detecteren.'''
    Voltage: int
    '''Laatst gemeten spanning in millivolt (mV).'''
    Current: int
    '''Laatst gemeten stroom in milliampère (mA).'''
    callback: callable
    '''Callbackfunctie die aangeroepen wordt wanneer nieuwe data succesvol verwerkt is.'''
    parent_name: str
    '''Naam van het bovenliggende hardwareonderdeel, bijv. "robot.body"'''
    instance_ENUM: int
    '''Enumwaarde die het specifieke batterijonderdeel aanduidt (bijv. SaraRobotPartNames.BATTERY).'''
    instance_name: str
    '''Samengestelde naam in de vorm "robot.body.battery"'''
    bridge_manager: object
    '''Bridge manager die verantwoordelijk is voor de communicatie met het fysieke onderdeel.'''
    
    EMPTY = 0
    ERROR = 1
    UNKNOWN = 2
    DISCHARGE = 3
    CHARGE = 4


    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """
        Initialiseert een Battery-object.

        Args:
            bridge_manager: Object dat commando’s naar de hardware verstuurt.
            parent_name (str): Naam van het bovenliggende hardwaredeel.
            instance_ENUM: Enum die aanduidt welk batterijonderdeel dit is.
        """
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.batterystate = Battery.ERROR
        self.oldstate = Battery.ERROR
        self.Voltage = 0
        self.Current = 0
        self.callback = None


    def print_state(self) -> None:
        """
        Print de huidige batterijstatus naar de console, inclusief spanning en stroom.
        """
        txt = "Battery    : Voltage {} mV, Current {} mA, State = ".format(
            self.Voltage, self.Current
        )
        txt += Battery.state_names[self.batterystate]
        print(txt)

    def check_not_empty(self) -> bool:
        """
        Controleert of de batterij nog voldoende lading heeft om veilig te gebruiken.

        Returns:
            bool: True als de status minimaal DISCHARGE is en de spanning boven 13000 mV ligt.
        """
        batteryFullEnough = self.batterystate >= Battery.DISCHARGE
        batteryFullEnough = batteryFullEnough and (self.Voltage >= 13000)
        return batteryFullEnough

    def new_data(self, data) -> None:
        """
        Verwerkt een nieuw data-pakketje met batterij-informatie.

        Args:
            data (bytes): Byte-array met gegevens van de batterijmodule.
        """
        try:
            new_byte_array_uint16 = data[3 : 3 + 4 * 2]
            new_byte_array_int16 = data[3 + (4 * 2) : -2]

            # hex_values = " ".join([format(x, "02X") for x in new_byte_array_uint16])
            # print("< " + hex_values)

            # hex_values = " ".join([format(x, "02X") for x in new_byte_array_int16])
            # print("< " + hex_values)

            unt16_array = np.frombuffer(new_byte_array_uint16, dtype=">u2")

            # DeviceType = unt16_array[0]
            # FW_Version = unt16_array[1]
            # HW_Version = unt16_array[2]
            BatteryState = unt16_array[3]

            # print(format(BatteryState, "02X"))

            int16_array = np.frombuffer(new_byte_array_int16, dtype=">i2")

            self.Temperature = int16_array[0]  # First 16-bit integer
            self.Current = int16_array[1]  # Third 16-bit integer
            self.Voltage = int16_array[2]  # Second 16-bit integer

            # First set to unknown because not all bits are implemented.
            self.batterystate = Battery.UNKNOWN

            if (BatteryState & 0x0F00) == 0x0100:
                self.batterystate = Battery.DISCHARGE

            if (BatteryState & 0x0F00) == 0x0500:
                self.batterystate = Battery.CHARGE

            if (BatteryState & 0x0F00) == 0x0C00:
                self.batterystate = Battery.EMPTY

            if self.batterystate != self.oldstate:
                self.print_state()

            self.oldstate = self.batterystate

            if not self.check_not_empty():
                print("WARNING : Battery is almost empty, recharge first.")

            if self.callback is not None:
                self.callback()

        except:
            print("Battery data processing error")
    
    def get_batterystate(self) -> int:
        """
        Haalt de huidige interne batterijstatus op.

        Returns:
            int: Eén van de gedefinieerde statussen (bijv. Battery.CHARGE).
            EMPTY = 0
            ERROR = 1
            UNKNOWN = 2
            DISCHARGE = 3
            CHARGE = 4
        """
        return self.batterystate
        
    def get_voltage(self) -> int:
        """
        Geeft de laatst gemeten spanning in millivolt.

        Returns:
            int: Batterijspanning (mV).
        """
        return self.Voltage

    def get_current(self) -> int:
        """
        Geeft de laatst gemeten stroom in milliampère.

        Returns:
            int: Batterijstroom (mA).
        """
        return self.Current

    def set_callback(self, callback) -> None:
        """
        Stelt een callbackfunctie in die wordt aangeroepen na het verwerken van nieuwe data.

        Args:
            callback (callable): Functie zonder argumenten die wordt uitgevoerd.
        """
        self.callback = callback
        return