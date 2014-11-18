'''
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

'''

__author__ = "Javier D. Garcia-Lasheras"
__copyright__ = "Copyright 2014, CERN"
__license__ = "GPLv3 or later"
__version__ = "1.0"
__maintainer__ = "Javier D. Garcia-Lasheras"
__email__ = "javier@garcialasheras.com"
__status__ = "Development"


from ctypes import *
import os, errno, re, sys, struct


class SpecCard(Structure):
    ''' This Python structure matches with 
    the spec_private C one in speclib.c'''
    _fields_ = [
        ("bar0", 	c_void_p),
        ("bar4", 	c_void_p),
        ("vuart_base", 	c_uint),
    ]


class Spec():
    """A Class that creates SPEC objects. This includes basic controls
    and methods in order to handle a PCIe attached SPEC board.
    Access to a valid libspec.so is mandatory for a proper operation.
    NOTE: user write/access permission to the SPEC dev is mandatory.

    Attributes:
        bus, dev     bus and device for the requested SPEC device.
                     If one (or both) parameters are < 0,
                     takes first available card.
        libc         Path to a valid libspec.so shared library.
                     If empty, it points to /usr/lib/libspec.so
    """

    def __init__(self, bus=-1, dev=-1, libc='/usr/lib/libspec.so'):

        print 'filename is', __file__
        libc = os.path.join(os.path.dirname(__file__), 'libspec.so')
        if not os.path.isfile(libc):
            print('ERROR: libspec.so library not found')
            raise Exception()

        # Load library
        self.speclib = CDLL(libc)

        # redefine return types
        self.speclib.spec_open.restype = POINTER(SpecCard)
        self.speclib.spec_readl.restype = c_uint

        self.my_card = self.speclib.spec_open(c_int(bus), c_int(dev))

        print('a new SPEC object has been created')


    def __del__(self):
        '''Destroying the SPEC card object'''
        self.speclib.spec_close(self.my_card)
        print('The SPEC object has been destroyed')


    def specLoadBitstream(self, bitstream):
        '''Load a new bitstream into the SPEC throughout GN4124 interface.
         bitstream is the path to the .bin bitstream that must be loaded.
        '''
        
        if os.path.isfile(bitstream):
            print('the bitstream has been found...')
            gateware = create_string_buffer(bitstream.encode('utf-8'))
            gateware_ok = self.speclib.spec_load_bitstream(
                self.my_card, byref(gateware))
            if gateware_ok != 0:
                print('the bitstream has been successfully loaded')
            else:
                print('the bitstream loading has failed')
        else:
            print('cannot find the bitstream!!!')


    # 32 bit register operations in the Wishbone addressing space

    def specWriteL(self, address, data):
        '''Write a 32 bit integer data into the register at address'''
        self.speclib.spec_writel(self.my_card, c_uint(data), c_uint(address))


    def specReadL(self, address, hexformat=True):
        '''Read a 32 bit integer data from the register at address.
        Return an hex string if hexformat=True, an integer otherwise'''
        data = self.speclib.spec_readl(self.my_card, c_uint(address))
        if hexformat:
            data = hex(data)
        return data


    # bitwise register operations in the Wishbone addressing space 

    def specTestBit(self, address, offset):
        '''Test the bit placed at offset in the register at address.
        returns a nonzero result, 2**offset, if the bit is 1'''
        register = self.specReadL(address, hexformat=False)
        mask = 1 << offset
        return(register & mask)

     
    def specSetBit(self, address, offset):
        '''Set to 1 the bit placed at offset in the register at address'''
        register = self.specReadL(address, hexformat=False)
        mask = 1 << offset
        self.specWriteL(address, register | mask)


    def specClearBit(self, address, offset):
        '''Clear to 0 the bit placed at offset in the register at address'''
        register = self.specReadL(address, hexformat=False)
        mask = ~(1 << offset)
        self.specWriteL(address, register & mask)


    def specToggleBit(self, address, offset):
        '''Toggle/invert the bit placed at offset in the register at address'''
        register = self.specReadL(address, hexformat=False)
        mask = 1 << offset
        self.specWriteL(address, register ^ mask)



