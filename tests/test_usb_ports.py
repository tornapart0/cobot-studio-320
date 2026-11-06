import unittest
from cobot_studio.hardware import Controller, usb_serial_port


class UsbPortTests(unittest.TestCase):
    def test_usb_ports_recognized_and_bluetooth_rejected(self):
        for port in ('/dev/cu.usbserial-5AE20108941','/dev/cu.usbmodem101','/dev/cu.wchusbserial1','/dev/cu.SLAB_USBtoUART'):
            self.assertTrue(usb_serial_port(port))
        for port in ('/dev/cu.Bluetooth-Incoming-Port','/dev/cu.debug-console','/dev/cu.soundcoreSleepA10'):
            self.assertFalse(usb_serial_port(port))
            with self.assertRaisesRegex(ValueError,'robot USB port'):
                Controller().connect(port)

    def test_open_error_includes_port_and_actionable_context(self):
        def rejected(*args,**kwargs):
            raise OSError(22,'Invalid argument')
        controller=Controller(rejected)
        with self.assertRaisesRegex(ValueError,'Could not open /dev/cu.usbserial-test at 115200 baud'):
            controller.connect('/dev/cu.usbserial-test')
        self.assertIsNone(controller.robot)
