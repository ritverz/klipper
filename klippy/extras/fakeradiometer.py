# Что делаем, если отвалился радиометр

import logging
import os
import random

import serial
from functools import reduce

try:
    from queue import Queue, Empty
except ImportError:
    from Queue import Queue, Empty


REPORT_TIME = 2.0

READ_DATA_COMMAND = b'\x53\x01\x01\x85'
CHANGE_GAIN_COMMAND = b'\x53\x01\x07\x91'
START_BYTE = b'\x53'

CHANGE_GAIN_OK = b'\x00'
CHANGE_GAIN_ERROR = b'\xFF'

DATA_ANSWER, GAIN_ANSWER = range(2)

PACKET_LENGTH = 8

K_KOEFF = 0.01
GAIN = 1
SERIAL_PORT = '/dev/serial/by-path/pci-0000:00:1d.0-usb-0:1.2:1.0-port0'
SERIAL_BAUD = 115200
SERIAL_TIMER = 0.1

GAIN_CHOICE = {x: x for x in (1, 2, 4, 8)}

SERIAL_BAUD_CHOICE = {x: x for x in (
    110, 300, 600, 1200, 2400, 4800, 9600, 14400,
    19200, 38400, 57600, 115200, 128000, 256000
)}


def get_data_from_queue(queue):
    data = b''
    while not queue.empty():
        try:
            data = queue.get_nowait()
        except Empty:
            pass
    return data


def check_crc(data):
    buffer_sum = reduce(lambda a, b: a + b, data).to_bytes(4, 'big')
    return buffer_sum[-1]


class Radiometer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name().split()[-1]
        self.printer.add_object(f'fakeradiometer {self.name}', self)

        self.sig = 0.0
        self.temp = 0.0
        self.min_temp = 0.0
        self.max_temp = 0.0

        self.serial = None
        self.main_timer = None
        self.read_timer = None
        self.read_buffer = b''
        self.read_queue = Queue()
        self.write_timer = None
        self.write_queue = Queue()

        self.answer_length = 0
        self.answer_type = DATA_ANSWER

        # Getting parameters from the config file.
        self.k_koeff = config.getfloat('k_koeff', default=K_KOEFF)
        self.serial_port = config.get('serial_port', default=SERIAL_PORT)

        self.serial_baud = config.getchoice(
            'serial_baud', choices=SERIAL_BAUD_CHOICE, default=SERIAL_BAUD
        )
        self.serial_baud = config.getint(
            'serial_baud', default=self.serial_baud
        )

        self.gain = config.getchoice('gain', choices=GAIN_CHOICE, default=GAIN)
        self.gain = config.get('gain', self.gain)

        if self.printer.get_start_args().get('debugoutput') is not None:
            return

        self.printer.register_event_handler(
            'klippy:connect',
            self._connect
        )

    # def handle_connect(self):
    #     self.reactor.update_timer(self.read_timer, self.reactor.NOW)
    #     self.reactor.update_timer(self.write_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME

    def _connect(self):
        with self.write_queue.mutex:
            self.write_queue.queue.clear()
        with self.read_queue.mutex:
            self.read_queue.queue.clear()

        # self.serial = serial.Serial(
        #     self.serial_port, self.serial_baud, timeout=0, write_timeout=0
        # )

        self.main_timer = self.reactor.register_timer(
            self._sample_radiometer, self.reactor.NOW
        )

        self.read_timer = self.reactor.register_timer(
            self._read_serial, self.reactor.NOW
        )
        self.write_timer = self.reactor.register_timer(
            self._write_serial, self.reactor.NOW
        )

    def _f_temp(self):
        # TODO: Выяснить как эта функция выглядит/рассчитывается.
        return 0.00001 * self.temp

    def _decode_data(self, data):
        # if data[7] == check_crc(data[:7]):
        if True:
            self.temp = self.k_koeff * (data[4] | data[5] << 8)
            self.sig = self._f_temp() * (data[2] | data[3] << 8)
            self.gain = data[6]
        else:
            logging.warning('Checksum error while serial reading.')

    def _sample_radiometer(self, eventtime):
        self.write_queue.put(READ_DATA_COMMAND)

        data = get_data_from_queue(self.read_queue)
        if data:
            self._decode_data(data)
            # self.gcode.respond_info(
            #     f'Temp: {self.temp:.2f} '
            #     f'Signal: {self.sig:.2f} '
            #     f'Gain: {self.gain}'
            # )
        else:
            logging.warning('Empty radiometer data.')

        mcu = self.printer.lookup_object('mcu')
        measured_time = self.reactor.monotonic()
        self._callback(mcu.estimated_print_time(measured_time), self.sig)

        return measured_time + REPORT_TIME

    def _write_serial(self, eventtime):
        data = get_data_from_queue(self.write_queue)
        # self.serial.write(data)
        return eventtime + SERIAL_TIMER

    def _read_serial(self, eventtime):
        # try:
        #    self.file_handle.seek(0)
        #    self.temp = float(self.file_handle.read())/1000.0
        # except Exception:
        #    logging.exception("temperature_host: Error reading data")
        #    self.temp = 0.0
        # return self.reactor.NEVER

        # if self.temp < self.min_temp:
        #    self.printer.invoke_shutdown(
        #        "HOST temperature %0.1f below minimum temperature of %0.1f."
        #        % (self.temp, self.min_temp,))
        # if self.temp > self.max_temp:
        #    self.printer.invoke_shutdown(
        #        "HOST temperature %0.1f above maximum temperature of %0.1f."
        #        % (self.temp, self.max_temp,))

        while True:
            # self.read_buffer += self.serial.read()
            self.read_buffer += os.urandom(1)

            if len(self.read_buffer):
                if len(self.read_buffer) == PACKET_LENGTH:
                    self.read_queue.put(self.read_buffer)
                    self.read_buffer = b''
                    break
            else:
                break

        # self.read_buffer += self.serial.read()
        # if len(self.read_buffer) == PACKET_LENGTH:
        #     self.read_queue.put(self.read_buffer)
        #     self.read_buffer = b''

        return eventtime + SERIAL_TIMER

    def get_status(self, eventtime):
        return {
            'sig': self.sig,
            'temp': self.temp,
            'gain': self.gain
        }


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, 'heaters')
    pheaters.add_sensor_factory('fakeradiometer', Radiometer)

