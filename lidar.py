import usb.core
from lib import parser
from lib import constants as c


class Lidar(object):
    def __init__(self, id_vendor, id_product):
        self.__idVendor = id_vendor
        self.__idProduct = id_product
        self.__dev_TIM = None
        self.__distances = []
        self.__angles_deg = []

        self.__RSSI = []
        self.__mirror_support_right = []
        self.__mirror_support_left = []
        self.__angle_support_right = []
        self.__angle_support_left = []

    def connect(self):
        # find TIM- Device
        self.__dev_TIM = usb.core.find(idVendor=self.__idVendor, idProduct=self.__idProduct)

        # In case no TIM Device is found
        if self.__dev_TIM is None:
            raise ValueError("Please ensure that your TIM-Device is connected")

    def __write(self, msg):
        # write(endpoint, data)
        self.__dev_TIM.write(2 | usb.ENDPOINT_OUT, "\x02" + msg + "\x03\0")

    def __read(self):
        # read(endpoint, size_or_buffer, timeout)
        ret = self.__dev_TIM.read(1 | usb.ENDPOINT_IN, 65535, timeout=100)
        return ret

    def scan(self):
        self.__write(c.command["message"])
        raw_data = self.__read()
        raw_data = "".join(chr(i) for i in raw_data[1:-1])
        raw_data_arr = raw_data.split()

        '''
        type_of_command = raw_data_arr[0]
        command = raw_data_arr[1]
        version_number = parser.hex2dec(raw_data_arr[2])
        device_number = parser.hex2dec(raw_data_arr[3])
        serial_number = parser.hex2dec(raw_data_arr[4])
        device_status = parser.hex2dec(raw_data_arr[5] + raw_data_arr[6])
        telegram_counter = parser.hex2dec(raw_data_arr[7])
        scan_counter = parser.hex2dec(raw_data_arr[8])
        time_since_startup = parser.hex2dec(raw_data_arr[9])
        time_of_transmission = parser.hex2dec(raw_data_arr[10])
        input_status = parser.hex2dec(raw_data_arr[11] + raw_data_arr[12])
        output_status = parser.hex2dec(raw_data_arr[13] + raw_data_arr[14])
        # reserved_byte_A [raw_data_arr[15]
        '''
        scanning_frequency = parser.hex2dec(raw_data_arr[16]) / 100
        '''
        measurement_frequency = parser.hex2dec(raw_data_arr[17])
        number_of_encoders = parser.hex2dec(raw_data_arr[18])
        number_of_16_bit_channels = parser.hex2dec(raw_data_arr[19])
        measured_data_contents_DIST = raw_data_arr[20]
        scaling_factor_DIST = parser.hex2float(raw_data_arr[21])
        scaling_offset = parser.hex2float(raw_data_arr[22])
        '''
        starting_angle = parser.hex2int32(raw_data_arr[23]) / 10000
        angular_step_width = parser.hex2dec(raw_data_arr[24]) / 10000
        number_of_data = parser.hex2dec(raw_data_arr[25])

        time_delay = round(1/scanning_frequency, 3)
        offset = 90

        distances = []
        angles_deg = []

        # Get Data
        for i in range(number_of_data):
            value = parser.hex2dec(raw_data_arr[26 + i])

            if value < c.lidar_distances["minimum distance"]:  # Out of Range (Value not reflected back)
                value = c.lidar_distances["maximum distance"]

            # Left -> (0° - -90°)  & Right -> (0° - 90°)
            angle_deg = -1 * ((starting_angle - offset) + angular_step_width * i)

            distances.append(value)
            angles_deg.append(angle_deg)
            # print("Self Angles:" + str(self.__angles_deg))

        self.__distances = distances
        self.__angles_deg = angles_deg

        # print(self.__distances[135], self.__angles_deg[135])

        mirror_support_right = []
        angle_support_right = []
        mirror_support_left = []
        angle_support_left = []

        for i in range(len(self.__distances)):
            if self.__angles_deg[i] == 90:
                mirror_support_right.append(self.__distances[i])
                angle_support_right.append(self.__angles_deg[i])
            elif self.__angles_deg[i] == -90:
                mirror_support_left.append(self.__distances[i])
                angle_support_left.append(self.__angles_deg[i])

        self.__mirror_support_right = mirror_support_right
        self.__mirror_support_left = mirror_support_left
        self.__angle_support_right = angle_support_right
        self.__angle_support_left = angle_support_left


        # RSSI Data
        """
        number_of_8_bit_channels = parser.hex2dec(raw_data_arr[297])
        measured_data_contents_RSSI = raw_data_arr[298]
        scaling_factor_RSSI = parser.hex2float(raw_data_arr[299])
        scaling_offset = parser.hex2float(raw_data_arr[300])
        starting_angle_RSSI = parser.hex2int32(raw_data_arr[301])
        angular_step_width_RSSI = parser.hex2dec(raw_data_arr[302])
        """
        number_of_data_RSSI = parser.hex2dec(raw_data_arr[303])

        # print(number_of_data_RSSI)

        RSSI = []
        for i in range(number_of_data_RSSI):
            RSSI_value = parser.hex2dec(raw_data_arr[304 + i])
            RSSI.append(RSSI_value)

        # print(RSSI)
        self.__RSSI = RSSI

        return time_delay

    @property
    def get_RSSI(self):
        return self.__RSSI

    @property
    def get_distances(self):
        return self.__distances

    @property
    def get_angles(self):
        return self.__angles_deg

    @property
    def get_mirror_support_right(self):
        return self.__mirror_support_right

    @property
    def get_mirror_support_left(self):
        return self.__mirror_support_left
