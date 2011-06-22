#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: ADD.PY
Date: Wednesday, June 22 2011
Description: Utility script to add x to dx in prototype control loop.
"""

import sys
import json
import numpy
npa = numpy.array

x = json.loads(sys.argv[1])
dx = json.loads(sys.argv[2])

print list(npa(x) + npa(dx))

