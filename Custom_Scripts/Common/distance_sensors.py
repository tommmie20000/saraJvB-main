import os
import sys
import platform
import numpy as np

from Common.sara_common import body_parts_names
from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames


class DistanceSensors:
    """
    Klasse voor het uitlezen en verwerken van afstandsgegevens van de robot.

    De robot beschikt over meerdere afstandssensoren rondom de basis,
    inclusief cliffranddetectie. Deze klasse interpreteert de meetwaarden en
    detecteert botsingen of gevaren.
    """
    
    bridge_manager: object
    '''Communicatie-interface naar de robot.'''

    parent_name: str
    '''Naam van het bovenliggende robotonderdeel.'''

    instance_ENUM: int
    '''Enum die aangeeft welk onderdeel deze sensorset is.'''

    instance_name: str
    '''Volledige naam van het onderdeel, bijv. "robot.body.distance_sensors".'''

    sensors: np.ndarray
    '''Laatste gemeten waardes van de sensoren (13 stuks).'''

    sensors_prev: np.ndarray
    '''Vorige meetwaarden, gebruikt bij foutcorrectie.'''

    valid_data: bool
    '''True als de laatste meting succesvol is verwerkt.'''

    error_counter: int
    '''Teller voor foutieve berichten.'''

    rx_counter: int
    '''Aantal succesvolle datapunten ontvangen.'''

    callback: callable
    '''Functie die wordt aangeroepen na nieuwe meetdata.'''

    hoek_bottom: float = 22.5
    '''Hoekstap voor onderste sensorset (in graden).'''

    hoek_mid: float = 22.5
    '''Hoekstap voor middelste sensorset (in graden).'''
    
    #hoek_bottom = 22.5
    sensor_angles_bottom = np.array(
        [
            [-4 * hoek_bottom, -3 * hoek_bottom],
            [-3 * hoek_bottom, -2 * hoek_bottom],
            [-2 * hoek_bottom, -1 * hoek_bottom],
            [-1 * hoek_bottom, -0 * hoek_bottom],
            [0 * hoek_bottom, 1 * hoek_bottom],
            [1 * hoek_bottom, 2 * hoek_bottom],
            [2 * hoek_bottom, 3 * hoek_bottom],
            [3 * hoek_bottom, 4 * hoek_bottom],
        ]
    )
    '''Hoekbereiken van de onderste sensoren.'''


    #hoek_mid = 22.5
    sensor_angles_mid = np.array(
        [
            [-0.5 * hoek_mid, 0.5 * hoek_mid],
            [-1.5 * hoek_mid, -0.5 * hoek_mid],
            [0.5 * hoek_mid, 1.5 * hoek_mid],
        ]
    )
    '''Hoekbereiken van de middelste sensoren.'''

    
    def __init__(self, bridge_manager, parent_name, instance_ENUM) -> None:
        """
        Initialiseert het DistanceSensors-object.

        Args:
            bridge_manager (object): Interface voor communicatie.
            parent_name (str): Naam van het bovenliggende onderdeel.
            instance_ENUM (int): Enumwaarde voor het sensoronderdeel.
        """
        self.bridge_manager = bridge_manager
        self.parent_name = parent_name
        self.instance_ENUM = instance_ENUM
        self.instance_name = self.parent_name + "." + bodypart_to_string(instance_ENUM)

        print("Adding " + self.instance_name)

        self.sensors = np.ones(13) * 65295
        self.sensors_prev = self.sensors.copy()

        # Cliff sensors need to start at 0.
        self.sensors[10] = 0
        self.sensors[11] = 0

        self.valid_data = False
        self.error_counter = 0
        self.rx_counter = 0
        self.callback = None


    def new_data(self, data) -> None:
        """
        Verwerkt binnenkomende data van alle afstandssensoren.

        Args:
            data (bytes): Byte-array van de sensorwaarden.
        """
        try:
            datalength = data[2]

            assert datalength == 26, (
                self.full_bodypart_name + " data length not correct!"
            )

            for i in range(13):
                new_byte_array = data[3 + i * 2 : -2]

                uint8_array = np.frombuffer(new_byte_array, dtype=">u1")
                combined_int = int.from_bytes(new_byte_array[0:2], byteorder="big")
                self.sensors[i] = float(combined_int)

            if (self.sensors[8] == 0) or (self.sensors[9] == 0) or (self.sensors[10] == 0):
                # print("Invalid value for sensors 8, 9 or 10 detected!")
                self.sensors[8] = self.sensors_prev[8]
                self.sensors[9] = self.sensors_prev[9]
                self.sensors[10] = self.sensors_prev[10]

            self.valid_data = True
            self.error_counter = 0
            self.rx_counter += 1
            self.sensors_prev = self.sensors.copy()

        except:
            print("Distance sensors data processing error")

            self.error_counter += 1

            if self.error_counter > 3:
                self.valid_data = False

        # ------------------------------------------------------------------------
        # Cliff sensors
        # ------------------------------------------------------------------------
        if self.sensors[11] >= 40000:
            print("Left cliff sensor too large value!")

        if self.sensors[12] >= 40000:
            print("Right cliff sensor too large value!")

        if self.callback is not None:
            self.callback()

    def sensor_warning(self, threshold=30000) -> bool:
        """
        Geeft aan of er een waarschuwing is op basis van een drempelwaarde.

        Returns:
            bool: True als een van de sensors een waarde onder de drempel heeft.
        """

        if self.valid_data == False:
            return False
        else:
            return np.any(self.sensors[0:-2] < threshold)

    def sensor_collision(self, threshold=20000) -> bool:
        """
        Detecteert botsing op basis van drempelwaarde.

        Returns:
            bool: True als er waarschijnlijk een botsing is.
        """
        if self.valid_data == False:
            return False
        else:
            return np.any(self.sensors[0:-2] < threshold)

    def sensor_cliffwarning(self) -> bool:
        """
        Detecteert of één van de cliff-sensoren over de rand kijkt.

        Returns:
            bool: True als de cliffwaarde gevaarlijk hoog is.
        """
        if self.valid_data == False:
            return True
        else:
            return (self.sensors[11] >= 40000) or self.sensors[12] >= 40000

    def print_values(self) -> None:
        """
        Print alle sensorwaarden naar de console.
        """
        print("Distances  :", [int(value) for value in self.sensors])

    def get_all_values(self) -> np.ndarray:
        """
        Retourneert alle huidige sensorwaarden.

        Returns:
            np.ndarray: 13 sensorwaardes.
        """
        return self.sensors

    def get_rx_counter(self) -> int:
        """
        Geeft aan hoeveel keer er succesvol data is ontvangen.

        Returns:
            int: Aantal ontvangen berichten.
        """
        return self.rx_counter

    # --------------------------------------------------------------------------------
    # Basic checks where collision is
    # --------------------------------------------------------------------------------
    def is_collision_left(self, threshold=20000) -> bool:
        """Detecteert botsing links."""
        if self.valid_data == False:
            return False
        else:
            return (self.sensors[0] < threshold) or (self.sensors[1] < threshold)

    def is_collision_frontleft(self, threshold=20000) -> bool:
        """Detecteert botsing schuin linksvoor."""
        if self.valid_data == False:
            return False
        else:
            return self.sensors[2] < threshold

    def is_collision_front(self, threshold=20000) -> bool:
        """Detecteert botsing recht vooruit (inclusief midden- en cliffsensoren)."""
        if self.valid_data == False:
            return False
        else:
            return (
                (self.sensors[3] < threshold)
                or (self.sensors[4] < threshold)
                or (self.sensors[8] < threshold)
                or (self.sensors[9] < threshold)
                or (self.sensors[10] < threshold)
            )

    def is_collision_frontright(self, threshold=20000) -> bool:
        """Detecteert botsing schuin rechtsvoor."""
        if self.valid_data == False:
            return False
        else:
            return self.sensors[5] < threshold

    def is_collision_right(self, threshold=20000) -> bool:
        """Detecteert botsing rechts."""
        if self.valid_data == False:
            return False
        else:
            return (self.sensors[6] < threshold) or (self.sensors[7] < threshold)

    def set_callback(self, callback) -> None:
        """
        Stelt een callback in die wordt aangeroepen bij nieuwe data.

        Args:
            callback (callable): Functie zonder argumenten.
        """
        self.callback = callback
        return