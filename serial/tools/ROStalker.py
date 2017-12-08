#!/usr/bin/env python
import rospy
import codecs
import os
import sys
import threading
from subprocess import call
import serial
from serial.tools.list_ports import comports
from serial.tools import hexlify_codec
import time
from std_msgs.msg import String

def set_rx_encoding(encoding, errors='replace'):
    """set encoding for received data"""
    input_encoding = encoding
    rx_decoder = codecs.getincrementaldecoder(encoding)(errors)
    return rx_decoder

def ask_for_port():
    """\
    Show a list of ports and ask the user for a choice. To make selection
    easier on systems with long device names, also allow the input of an
    index.
    """
    sys.stderr.write('\n--- Available ports:\n')
    ports = []
    for n, (port, desc, hwid) in enumerate(sorted(comports()), 1):
        sys.stderr.write('--- {:2}: {:20} {!r}\n'.format(n, port, desc))
        ports.append(port)
    while True:
        port = raw_input('--- Enter port index or full name: ')
        try:
            index = int(port) - 1
            if not 0 <= index < len(ports):
                sys.stderr.write('--- Invalid index!\n')
                continue
        except ValueError:
            pass
        else:
            port = ports[index]
        return port
def talker(serial_port):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10
    currentRecievedString = ""
    rx_decoder = set_rx_encoding('UTF-8')
    #here there be dragons
    rate.sleep()
    rate.sleep()
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        try:
            # read all that is there or wait for one byte
            data = serial_port.read(serial_port.in_waiting or 1)
            if data:
                #if self.raw:
                #    self.console.write_bytes(data)
                #else:
                #    text = self.rx_decoder.decode(data)
                #    for transformation in self.rx_transformations:
                #        text = transformation.rx(text)
                text = rx_decoder.decode(data)
                currentRecievedString += text
                if '\n' in text or '\r' in text or '\r\n' in text:
                    currentRecievedString = currentRecievedString.replace("\r", "").replace("\n", "")
                    rospy.loginfo(currentRecievedString)
                    pub.publish(currentRecievedString)
                    #if self.execute:
                    #    call("gnome-terminal -x "+self.currentRecievedString, shell=True)
                    currentRecievedString = ""               
        except serial.SerialException:
            raise       # XXX handle instead of re-raise?
        #  for legacy support, don't delete
        #  serial_port.write("benis\n")
        ##rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

def main(default_port=None, default_baudrate=9600, default_rts=None, default_dtr=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="Miniterm - A simple terminal program for the serial port.")

    parser.add_argument(
        "port",
        nargs='?',
        help="serial port name ('-' to show port list)",
        default=default_port)

    parser.add_argument(
        "baudrate",
        nargs='?',
        type=int,
        help="set baud rate, default: %(default)s",
        default=default_baudrate)

    group = parser.add_argument_group("port settings")

    group.add_argument(
        "--parity",
        choices=['N', 'E', 'O', 'S', 'M'],
        type=lambda c: c.upper(),
        help="set parity, one of {N E O S M}, default: N",
        default='N')

    group.add_argument(
        "--rtscts",
        action="store_true",
        help="enable RTS/CTS flow control (default off)",
        default=False)

    group.add_argument(
        "--xonxoff",
        action="store_true",
        help="enable software flow control (default off)",
        default=False)

    group.add_argument(
        "--rts",
        type=int,
        help="set initial RTS line state (possible values: 0, 1)",
        default=default_rts)

    group.add_argument(
        "--dtr",
        type=int,
        help="set initial DTR line state (possible values: 0, 1)",
        default=default_dtr)

    group.add_argument(
        "--ask",
        action="store_true",
        help="ask again for port when open fails",
        default=False)

    group = parser.add_argument_group("delay")
    group.add_argument(
        "-td","--transmitdelay",
        type=int,
        dest="transmit_delay",
        help="Set delay time in milliseconds for sending data",
        default=1
    )

    group = parser.add_argument_group("data handling")

    group.add_argument(
        "-e", "--echo",
        action="store_true",
        help="enable local echo (default off)",
        default=False)

    group.add_argument(
        "-ex", "--execute",
        action="store_true",
        help="enable executing recieved messages (dangerous)",
        default=False)

    group.add_argument(
        "--encoding",
        dest="serial_port_encoding",
        metavar="CODEC",
        help="set the encoding for the serial port (e.g. hexlify, Latin1, UTF-8), default: %(default)s",
        default='UTF-8')

    group.add_argument(
        "-f", "--filter",
        action="append",
        metavar="NAME",
        help="add text transformation",
        default=[])

    group.add_argument(
        "--eol",
        choices=['CR', 'LF', 'CRLF'],
        type=lambda c: c.upper(),
        help="end of line mode",
        default='CRLF')

    group.add_argument(
        "--raw",
        action="store_true",
        help="Do no apply any encodings/transformations",
        default=False)

    group = parser.add_argument_group("hotkeys")

    group.add_argument(
        "--exit-char",
        type=int,
        metavar='NUM',
        help="Unicode of special character that is used to exit the application, default: %(default)s",
        default=0x1d)  # GS/CTRL+]

    group.add_argument(
        "--menu-char",
        type=int,
        metavar='NUM',
        help="Unicode code of special character that is used to control miniterm (menu), default: %(default)s",
        default=0x14)  # Menu: CTRL+T

    group = parser.add_argument_group("diagnostics")

    group.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="suppress non-error messages",
        default=False)

    group.add_argument(
        "--develop",
        action="store_true",
        help="show Python traceback on error",
        default=False)

    args = parser.parse_args()
    while True:
        # no port given on command line -> ask user now
        if args.port is None or args.port == '-':
            try:
                args.port = ask_for_port()
            except KeyboardInterrupt:
                sys.stderr.write('\n')
                parser.error('user aborted and port is not given')
            else:
                if not args.port:
                    parser.error('port is not given')
        try:
            serial_instance = serial.serial_for_url(
                args.port,
                args.baudrate,
                parity=args.parity,
                rtscts=args.rtscts,
                xonxoff=args.xonxoff,
                do_not_open=True)

            if not hasattr(serial_instance, 'cancel_read'):
                # enable timeout for alive flag polling if cancel_read is not available
                serial_instance.timeout = 1

            if args.dtr is not None:
                if not args.quiet:
                    sys.stderr.write('--- forcing DTR {}\n'.format('active' if args.dtr else 'inactive'))
                serial_instance.dtr = args.dtr
            if args.rts is not None:
                if not args.quiet:
                    sys.stderr.write('--- forcing RTS {}\n'.format('active' if args.rts else 'inactive'))
                serial_instance.rts = args.rts

            serial_instance.open()
        except serial.SerialException as e:
            sys.stderr.write('could not open port {!r}: {}\n'.format(args.port, e))
            if args.develop:
                raise
            if not args.ask:
                sys.exit(1)
            else:
                args.port = '-'
        else:
            break

    
    talker(serial_port=serial_instance)
    
if __name__ == '__main__':
    try:
        main()
        #talker()
    except rospy.ROSInterruptException:
        pass

