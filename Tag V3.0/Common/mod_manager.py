import serial
import numpy as np
import threading


class ModManager:
    """
    Klasse voor communicatie met een ander serieel apparaat via Modbus-protocol.

    De ModManager opent één van twee mogelijke COM-poorten, leest data via een thread,
    verstuurt gegevens en berekent checksums.
    """

    port1: str
    '''Eerste COM-poort waarop geprobeerd wordt verbinding te maken.'''

    port2: str
    '''Alternatieve COM-poort als `port1` niet werkt.'''

    baudrate: int
    '''Baudrate voor seriële communicatie (standaard: 57600).'''

    timeout: int
    '''Timeout voor de seriële poort in seconden.'''

    serial_port: serial.Serial
    '''Het actieve seriële poortobject.'''

    receive_callback: callable
    '''Callbackfunctie voor binnenkomende data.'''

    receive_thread: threading.Thread
    '''Thread die inkomende data uitleest.'''

    receive_thread_stop: threading.Event
    '''Signaal om de ontvangstthread te stoppen.'''
    
    def __init__(self, port1, port2, baudrate=57600, timeout=1) -> None:
        """
        Initialiseert een ModManager-object.

        Args:
            port1 (str): Eerste COM-poort.
            port2 (str): Alternatieve COM-poort.
            baudrate (int): Baudrate voor de verbinding.
            timeout (int): Timeout voor seriële poortcommunicatie.
        """
        self.port1 = port1
        self.port2 = port2
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.receive_callback = None
        self.receive_thread = None
        self.receive_thread_stop = threading.Event()

    def open_port(self) -> None:
        """
        Probeert `port1` te openen. Als dat mislukt, probeert hij `port2`.

        Wordt de poort geopend, dan start ook de ontvangthread.
        """
        try:
            self.serial_port = serial.Serial(port=self.port1, baudrate=self.baudrate, timeout=self.timeout)
            print(f"Serial port {self.port1} opened successfully.")
            self.start_receive_thread()
        except serial.SerialException as e:
            print(f"{e}")

            try:
                self.serial_port = serial.Serial(port=self.port2, baudrate=self.baudrate, timeout=self.timeout)
                print(f"Serial port {self.port2} opened successfully.")
                self.start_receive_thread()
            except serial.SerialException as e:
                print(f"{e}")

    def close_port(self) -> None:
        """
        Sluit de actieve seriële poort en stopt de ontvangstthread.
        """
        if self.serial_port:
            self.receive_thread_stop.set()
            self.receive_thread.join()
            self.serial_port.close()
            print(f"Serial port closed.")

    def send_data(self, data) -> None:
        """
        Verstuurt een bytearray over de geopende seriële poort.

        Args:
            data (bytes): Gegevens om te verzenden.
        """
        if self.serial_port:
            try:
                data_bytes = bytes(data)
                self.serial_port.write(data_bytes)
                # print(f"Data sent: {data}")
            except serial.SerialException as e:
                assert False, f"Failed to send data: {e}"

        else:
            assert False, "Serial port is not open. Cannot send data."

    def set_receive_callback(self, callback) -> None:
        """
        Stelt de functie in die wordt aangeroepen bij binnenkomende data.

        Args:
            callback (callable): Functie zonder argumenten.
        """
        self.receive_callback = callback
        # self.stream_area = stream_area

    def start_receive_thread(self) -> None:
        """
        Start de achtergrondthread die data uitleest van de seriële poort.
        """
        self.receive_thread_stop.clear()
        self.receive_thread = threading.Thread(target=self.receive_serial_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def receive_serial_data(self) -> None:
        """
        Leest data van de seriële poort en geeft die door aan de callback.
        """
        while not self.receive_thread_stop.is_set():
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting)
                if self.receive_callback:
                    # self.receive_callback(data, self.stream_area)
                    self.receive_callback(data)

    def generate_modbus_crc(self, data) -> int:
        """
        Berekent een 16-bit Modbus CRC over de invoerdata.

        Args:
            data (np.ndarray): Byte-array van data.

        Returns:
            int: Modbus CRC-checksum.
        """
        # Convert the data array to uint8 if it's not already
        data = np.asarray(data, dtype=np.uint8)

        crc = 0xFFFF
        polynomial = 0xA001  # Modbus CRC-16 polynomial

        for byte in data:
            crc ^= int(byte)
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= polynomial
                else:
                    crc >>= 1

        # Swap bytes
        # crc = ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF)
        return crc

    def print_array_as_hex(self, data) -> None:
        """
        Print de opgegeven bytearray als hexadecimale waarden.

        Args:
            data (bytes): Gegevens om te printen.
        """
        hex_values = " ".join([format(x, "02X") for x in data])
        print("> " + hex_values)

    def send_enable(self, ID, enable) -> None:
        """
        Stuurt een commando om een onderdeel aan of uit te zetten.

        Args:
            ID (int): Doel-ID van het apparaat.
            enable (int): 1 om in te schakelen, 0 om uit te schakelen.
        """
        datalength = 2
        data = self.create_message(datalength)

        data[0] = 0x55
        data[1] = 0x01
        data[2] = datalength
        data[3] = ID
        data[4] = enable

        crc = self.generate_modbus_crc(data[:-2])  # skip empty CRC bytes

        print("> Modbus CRC: 0x", format(crc, "04X"))

        data[datalength + 3] = crc & 0xFF
        data[datalength + 4] = (crc >> 8) & 0xFF

        self.print_array_as_hex(data)

        self.send_data(data)

        return

    def create_message(self, datalength=0) -> bytearray:
        """
        Maakt een leeg commandoformaat van het juiste formaat.

        Args:
            datalength (int): Lengte van de payload.

        Returns:
            bytearray: Berichtenstructuur.
        """
        data = bytearray([0x00] * (5 + datalength))
        data[0] = 0x55

        return data

    def cmd_GetVersion(self) -> None:
        """
        Stuurt een versieverzoek naar het aangesloten apparaat.
        """
        datalength = 0
        data = self.create_message(datalength)

        data[0] = 0x55
        data[1] = 0x40
        data[2] = datalength

        crc = self.generate_modbus_crc(data[:-2])  # skip empty CRC bytes

        data[datalength + 3] = crc & 0xFF
        data[datalength + 4] = (crc >> 8) & 0xFF

        self.print_array_as_hex(data)

        self.send_data(data)

        return

    def cmd_Generic(self, cmd: int, datalength, payload: np.array) -> None:
        """
        Stuurt een willekeurig Modbus-achtig commando naar het apparaat.

        Args:
            cmd (int): Commando-ID.
            datalength (int): Lengte van de payload.
            payload (np.ndarray): Inhoud van het commando.
        """
        data = self.create_message(datalength)

        data[0] = 0x55
        data[1] = cmd & 0xFF
        data[2] = datalength & 0xFF

        for i in range(datalength):
            data[3 + i] = payload[i] & 0xFF

        crc = self.generate_modbus_crc(data[:-2])  # skip empty CRC bytes

        # print("> Modbus CRC: 0x", format(crc, '04X'))

        data[datalength + 3] = crc & 0xFF
        data[datalength + 4] = (crc >> 8) & 0xFF

        # self.print_array_as_hex(data)
        self.send_data(data)

        return

    def cmd_createCompassMoveCommand(self, cmd, angle, timeout) -> None:
        """
        Stuurt een commando voor een compassrotatie naar een specifieke hoek.

        Args:
            cmd (int): Commando-ID.
            angle (int): Doelhoek (0–360).
            timeout (int): Timeoutwaarde in seconden.
        """
        high = (int(angle) >> 8) & 0xFF
        low = int(angle) & 0xFF

        # Parameter #3 = timeout
        self.cmd_Generic(cmd, 3, np.array([high, low, timeout]))

        return
