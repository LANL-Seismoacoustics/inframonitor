#!/usr/bin/env sage -python
#
# RemoveEvents.py
#
# Usage:
# RemoveEvents.py <Input table ID> <Output table ID> <Max area of uncertainty>
#
# e.g., RemoveEvents.py 08_2011 08_2011_10000 10000 2,4,6,8,9,12,14,16,17,18,20,22,25,26,27,28,29,30,31,32,33,34,37,38,39,40,41,42,46,47,50,54,57,59,67,69,70
#
# Removes events from InfraMonitor 2 tables based on area of uncertainty region
#
# Stephen Arrowsmith (arrows@lanl.gov)
# Copyright (c) 2012, Los Alamos National Security, LLC
# All rights reserved.
# 
# Copyright 2012. Los Alamos National Security, LLC. This software was produced under U.S.
# Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is
# operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S.
# Government has rights to use, reproduce, and distribute this software.  NEITHER THE
# GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED,
# OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce
# derivative works, such modified software should be clearly marked, so as not to confuse it
# with the version available from LANL.
# 
# Additionally, redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# Redistributions of source code must retain the above copyright notice, this list
# 		  of conditions and the following disclaimer.
# Redistributions in binary form must reproduce the above copyright notice, this
# 	      list of conditions and the following disclaimer in the documentation and/or
# 	      other materials provided with the distribution.
# Neither the name of Los Alamos National Security, LLC, Los Alamos National
# 	      Laboratory, LANL, the U.S. Government, nor the names of its contributors may be
# 	      used to endorse or promote products derived from this software without specific
# 	      prior written permission.
# THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
# EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys, commands, pdb
import sage.interfaces.matlab as matlab
args = sys.argv

f1 = args[1]
f2 = args[2]
uncertainty = int(args[3])

d = commands.getoutput('pwd')

#evids = matlab.matlab.eval("PolyClean('" + d + '/' + f1 + ".IMpoly'," + str(uncertainty) + ")")
#evids = evids.lstrip('\nans =\n\n').rstrip('\n').split()
evids = args[4].split(',')

# Writing new .IMpoly file:
F1 = open(f1 + '.IMpoly','r')
F2 = open(f2 + '.IMpoly','w')
for line in F1:
	if evids.count(line.split()[0]) == 1:
		print >> F2, line.rstrip()
F1.close()
F2.close()

# Writing new .IMassoc file:
F1 = open(f1 + '.IMassoc','r')
F2 = open(f2 + '.IMassoc','w')
for line in F1:
	if evids.count(line.split()[2]) == 1:
		print >> F2, line.rstrip()
F1.close()
F2.close()

# Writing new .origin file:
F1 = open(f1 + '.origin','r')
F2 = open(f2 + '.origin','w')
for line in F1:
	if evids.count(line.split()[4]) == 1:
		print >> F2, line.rstrip()
F1.close()
F2.close()
