#!/usr/bin/python

import time

from spec_libc import *
spec = Spec()

spec.specLoadBitstream('/tmp/upload/spec_top.bin')

#time.sleep(1)
#spec.specWriteL(0x00080,0x8)

#
#spec.specWriteL(0x82000,123)
#spec.specWriteL(0x82004,7500)
#spec.specWriteL(0x80000,0x9)
#spec.specWriteL(0x80000,0x8)
#
#time.sleep(1)
#
##Print Status Registers
#for addr in range(0x82C00, 0x82C20, 0x4):
#    print spec.specReadL(addr)
