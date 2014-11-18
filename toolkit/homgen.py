#!/usr/bin/env python

import sys

from numpy import *

def generate_body():
    result = arange(1024)

    for r in result[:-1]:
        print '        X"%08x",' % r
    print '        X"%08x"' % result[-1]

for line in sys.stdin:
    if '@TABLE_BODY' in line:
        generate_body()
    else:
        print line,
