#!/usr/bin/env python
"""
Class for controling a RN2483 with python's Serial module
"""

import serial

class RN2483(object):
    def __init__(self, port):
        self._ser = serial.Serial(port, 57600, timeout=1)
        self.reset()
        print(self.command('sys get ver'))

    def command(self, cmdIn, check=False):
        self._ser.write(cmdIn+'\r\n')
        result = self._ser.readline().splitlines()[0]
        if check and result != 'ok':
            raise Exception('Cmd "%s" responded with "%s"'%(cmdIn, result))
        return result

    def reset(self):
        self.command('sys reset')

    def configLoRa(self,
        mod = 'lora',
        freq = None,
        pwr = -3, #-3 to 15
        sf = 'sf8', #7-12
        crc = 'on', #checksum
        iqi = 'off', #invert
        cr = '4/8', #4/5, 6, 7, 8
        wdt = 0, #0 is disabled
        sync = 0x12, #hex byte sync word
        bw = 125, #125, 250, 500 kHz bandwidth
    ):

        assert(mod == 'lora')
        assert(pwr >= -3 and pwr <= 15)
        assert(sf in ['sf7', 'sf8', 'sf9', 'sf10', 'sf11', 'sf12'])
        assert(crc in ['on', 'off'])
        assert(iqi in ['on', 'off'])
        assert(cr in ['4/5', '4/6', '4/7', '4/8'])
        assert(bw in [125, 250, 500])

        self.command('radio set mod %s'%mod, check=True)
        if freq is not None:
            self.command('radio set freq %d'%int(freq), check=True)
            assert(int(self.command('radio get freq')) == int(freq))
        self.command('radio set pwr %d'%pwr, check=True)
        self.command('radio set sf %s'%sf, check=True)
        self.command('radio set crc %s'%crc, check=True)
        self.command('radio set iqi %s'%iqi, check=True)
        self.command('radio set cr %s'%cr, check=True)
        self.command('radio set wdt %d'%int(wdt), check=True)
        self.command('radio set sync %s'%hex(sync)[2:], check=True)
        self.command('radio set bw %d'%int(bw), check=True)

    def enableCW(self):
        """
        Enable CW, remeber to reset after to use LoRa again.
        """
        self.command('radio cw on')

    def transmit(self, s):
        self.command('mac pause')
        out = ''.join(['%02x'%ord(c) for c in s])
        return self.command('radio tx %s'%out) == 'ok'

    #TODO receive one...

if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("--port", type="string", dest="port", help="TTY device node", default='/dev/ttyACM0')
    parser.add_option("--freq", type="float", dest="freq", help="Tranceiver frequency in Hz", default=868.1e6)
    parser.add_option("--bw", type="float", dest="bw", help="Tranceiver operational BW in Hz", default=250e3)
    parser.add_option("--pwr", type="int", dest="pwr", help="Transmit power in dB [-3 to 15]", default=-3)
    parser.add_option("--sf", type="int", dest="sf", help="Spread factor [7 to 12]", default=8)
    parser.add_option("--crc", action="store_true", dest="crc", help="Specify to use CRC")
    parser.add_option("--cr", type="string", dest="cr", help="Coding rate", default='4/8')
    parser.add_option("--sync", type="string", dest="sync", help="Sync word", default='0x12')
    parser.add_option("--cw", action="store_true", dest="cw", help="Transmit continuous wave")
    parser.add_option("--tx", type="string", dest="tx", help="Transmit the specified message", default=None)
    parser.add_option("--repeat", action="store_true", dest="repeat", help="Repeat transmission")
    (options, args) = parser.parse_args()

    rn2483 = RN2483(options.port)
    rn2483.configLoRa(
        freq=options.freq,
        bw=int(options.bw/1e3),
        pwr=options.pwr,
        sf='sf%d'%options.sf,
        crc='on' if options.crc else 'off',
        cr=options.cr,
        sync=eval(options.sync))

    if options.cw:
        rn2483.enableCW()
        exit(0)

    if options.tx:
        rn2483.transmit(options.tx) #transmit once
        while options.repeat: rn2483.transmit(options.tx)
