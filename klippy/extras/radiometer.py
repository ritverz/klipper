# sudo visudo
# username ALL = NOPASSWD: /usr/bin/rfcomm
# username ALL = NOPASSWD: /usr/bin/chmod
# klippy-env/bin/pip install pexpect
# sudo apt install bluez-tools ???


import logging
import re
import time
import pexpect
import serial

from functools import reduce

try:
    from queue import Queue, Empty
except ImportError:
    from Queue import Queue, Empty


REPORT_TIME = 2.0

START_BYTE = b'\x53'
ENDIAN = 'little'

READ_DATA_COMMAND = b'\x53\x01\x01\x55'
CHANGE_GAIN_COMMAND = b'\x07'

CHANGE_GAIN_OK = b'\x00'
CHANGE_GAIN_ERROR = b'\xFF'

DATA_RESPOND_LENGTH = 8
GAIN_RESPOND_LENGTH = 4
PACKET_HEADER_LENGTH = 2
PACKET_HEADER_CRC_LENGTH = 3

K_KOEFF = 1.0
GAIN = 1

RD_MAC_ADDRESS = '00:BA:55:57:17:B7'
RD_PIN_CODE = '1234'
RD_NAME = 'RADIOMETER'
PROMPT = '#'

SERIAL_PORT = '/dev/serial/by-path/pci-0000:00:1d.0-usb-0:1.2:1.0-port0'
SERIAL_BAUD = 115200
SERIAL_TIME = 0.1

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
        except Empty as em:
            logging.info(f'В очереди нет данных ({em.args})')
    return data


def calc_crc(data: bytes):
    overflow_sum = reduce(lambda a, b: a + b, data).to_bytes(4, ENDIAN)
    return overflow_sum[:1]  # Именно так, иначе будет возвращен int.


class Radiometer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name().split()[-1]
        self.printer.add_object(f'radiometer {self.name}', self)

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

        self.response_length = None
        self.start = True

        # Получение параметров из файла конфигурации.
        self.k_koeff = config.getfloat('k_koeff', default=K_KOEFF)
        self.serial_port = config.get('serial_port', default=SERIAL_PORT)
        self.rd_mac_address = config.get('mac_address', default=RD_MAC_ADDRESS)
        self.rd_pin_code = config.get('pin_code', default=RD_PIN_CODE)
        self.rd_name = config.get('name', default=RD_NAME)
        

        self.serial_baud = config.getchoice(
            'serial_baud', choices=SERIAL_BAUD_CHOICE, default=SERIAL_BAUD
        )
        self.serial_baud = config.getint(
            'serial_baud', default=self.serial_baud
        )

        self.gain = config.getchoice('gain', choices=GAIN_CHOICE, default=GAIN)
        self.gain = config.getint('gain', self.gain)

        if self.printer.get_start_args().get('debugoutput') is not None:
            return

        self.printer.register_event_handler(
            'klippy:connect',
            self._handle_connect
        )

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME

    def _clear_log(self, text):
        ansi_escape = re.compile(r'''
            \x1B        # ESC
                (?:     # 7-bit C1 Fe (except CSI)
                [@-Z\\-_]
            |           # or [ for CSI, followed by a control sequence
            \[
                [0-?]*  # Parameter bytes
                [ -/]*  # Intermediate bytes
                [@-~]   # Final byte
            )
        ''', re.VERBOSE)
        return ansi_escape.sub('', text).replace('\x01', '').replace('\x02', '')

    def _radiometer_connect(self):
        try:
            pexpect.run('rfkill unblock all')

            p = pexpect.spawn('bluetoothctl', encoding='utf-8')
            p.expect(PROMPT)

            p.sendline('power off')
            logging.warning('Power off bluetooth device')
            p.expect('Changing power off succeeded')

            p.sendline('power on')
            logging.warning('Power on bluetooth device')
            p.expect('Changing power on succeeded')

            p.sendline('scan on')
            p.expect(PROMPT)
            logging.warning(self._clear_log(p.before))
            time.sleep(10)

            p.sendline(f'remove {self.rd_mac_address}')
            p.expect(PROMPT)
            logging.warning(self._clear_log(p.before))
            
            while True:
                try:
                    p.sendline(f'pair {self.rd_mac_address}')
                    p.expect('Enter PIN code:')
                    logging.warning(self._clear_log(p.before))

                    p.sendline(self.rd_pin_code)
                    logging.warning(self._clear_log(p.before))
                    time.sleep(3)

                    child = pexpect.spawn('bt-device -l', timeout=None)
                    for line in child: 
                        logging.warning(f'NYAAAA: {line}')
                    child.close()

                except Exception as ex:
                    # logging.warning(f'Try to change power state {ex.args}')

                    p.sendline('power off')
                    logging.warning('Power off bluetooth device')
                    p.expect('Changing power off succeeded')

                    p.sendline('power on')
                    logging.warning('Power on bluetooth device')
                    p.expect('Changing power on succeeded')

                    continue
                else:
                    break
        
            p.sendline('quit')
            p.expect(pexpect.EOF)

            pexpect.run(f'sudo rfcomm release 0 {self.rd_mac_address} 1')
            pexpect.run(f'sudo rfcomm bind 0 {self.rd_mac_address} 1')
            pexpect.run('sudo chmod 777 /dev/rfcomm0')
        except Exception as ex:
            self.printer.invoke_shutdown(
                f'Критическая ошибка при попытке подключения к радиометру '
                f'{ex.args}'
            )

    def _open_serial(self):
        with self.write_queue.mutex:
            self.write_queue.queue.clear()

        with self.read_queue.mutex:
            self.read_queue.queue.clear()

        self._radiometer_connect()

        self.serial = serial.Serial(
            self.serial_port, self.serial_baud, timeout=0, write_timeout=0
        )

        self.main_timer = self.reactor.register_timer(
            self._sample_radiometer, self.reactor.NOW
        )

        self.read_timer = self.reactor.register_timer(
            self._read_serial, self.reactor.NOW
        )
        self.write_timer = self.reactor.register_timer(
            self._write_serial, self.reactor.NOW
        )

    def _handle_connect(self):
        self._open_serial()

    def _f_temp(self):
        # TODO: Выяснить как эта функция выглядит/рассчитывается.
        # return 0.00001 * self.temp
        return 1
    
    def _set_gain(self):
        data = CHANGE_GAIN_COMMAND + self.gain.to_bytes(1, ENDIAN)

        command = START_BYTE
        command += len(data).to_bytes(1, ENDIAN)
        command += data
        command += calc_crc(command)

        return command

    def _decode_data(self, data):
        data_len = len(data)
        data_body = data[:-1]
        crc = data[-1:]

        if crc == calc_crc(data_body):

            if data_len == DATA_RESPOND_LENGTH:
                temp = int.from_bytes(data[4:6], byteorder=ENDIAN, signed=True)
                self.temp = float(self.k_koeff * temp)

                sig = int.from_bytes(data[2:4], byteorder=ENDIAN, signed=False)
                self.sig = float(self._f_temp() * sig)
                
                self.gain = data[6]

            elif data_len == GAIN_RESPOND_LENGTH:
                respond = data[2:3]

                if respond == CHANGE_GAIN_OK:
                    self.gcode.respond_info(
                        f'Установлено значение усиления {self.gain}'
                    )
                elif respond == CHANGE_GAIN_ERROR:
                    self.gcode.respond_error(
                        'Радиометр не смог установить значение усиления'
                    )
                else:
                    logging.error(
                        'Ошибка при установлении усиления, радиометр не '
                        'прислал ответ или ответ не распознан'
                    )
            else:
                logging.error(
                        'Радиометр прислал ответ неправильной длины'
                    )
        else:
            logging.error(
                'Ошибка контрольной суммы при считывании данных радиометра'
            )

    def _sample_radiometer(self, eventtime: int):
     
        if self.start:
            self.write_queue.put(self._set_gain())
            self.start = False
        else:
            self.write_queue.put(READ_DATA_COMMAND)
       
        data = get_data_from_queue(self.read_queue)

        if data:
            self._decode_data(data)
        else:
            logging.warning('Нет ответа от радиометра')

        logging.warning(f'Receive {self.sig}')

        mcu = self.printer.lookup_object('mcu')
        measured_time = self.reactor.monotonic()
        self._callback(mcu.estimated_print_time(measured_time), self.sig)

        return measured_time + REPORT_TIME

    def _write_serial(self, eventtime):
        data = get_data_from_queue(self.write_queue)
        self.serial.write(data)
        return eventtime + SERIAL_TIME

    def _read_serial(self, eventtime):
        while True:
            self.read_buffer += self.serial.read()

            if len(self.read_buffer):
                logging.warning(f'Len {len(self.read_buffer)}')
                logging.warning(f'Raw {self.read_buffer}')
                # Считали стартовый байт и байт длины поля данных.
                if len(self.read_buffer) == PACKET_HEADER_LENGTH:
                    if self.read_buffer[:1] == START_BYTE:
                        # Добавили стартовый байт, байт длины и
                        # контрольную сумму.
                        data_length = self.read_buffer[1]
                        self.response_length = (
                            data_length + PACKET_HEADER_CRC_LENGTH
                        )
                    else:
                        logging.error(
                            'Ошибка в ответе радиометра, отсутствует стартовый '
                            'байт'
                        )
                        self.response_length = None
                        self.read_buffer = b''
                        continue

                if len(self.read_buffer) == self.response_length:
                    self.read_queue.put(self.read_buffer)
                    self.response_length = None
                    self.read_buffer = b''
                    break
            else:
                break

        return eventtime + SERIAL_TIME

    def get_status(self, eventtime):
        return {
            'sig': self.sig,
            'temp': round(self.temp, 1),
            'gain': self.gain
        }


def load_config(config):
    # Регистрируем радиометр в качестве датчика.
    pheaters = config.get_printer().load_object(config, 'heaters')
    pheaters.add_sensor_factory('radiometer', Radiometer)
