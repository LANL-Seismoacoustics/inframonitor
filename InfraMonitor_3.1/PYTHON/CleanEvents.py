#!/usr/bin/env sage -python
#
# CleanEvents.py
#
# e.g., CleanEvents.py 072407-082808_IM2.5
#
# Removes events containing duplicate arrivals
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

import sys, os, commands, pdb
args = sys.argv
f1 = args[1]

master_arrivals = []
new_events = []

F1 = open(f1 + '.IMassoc','r')
NEvents = int(commands.getoutput("tail -1 " + f1 + ".IMassoc").split()[2])
for i in range(1,NEvents+1):
	arrivals = commands.getoutput("awk '$3==" + str(i) + " {print $1, $2}' " + f1 + ".IMassoc").splitlines()
	
	# Breaking (and adding other arrivals) if any arrival is in master_arrivals:
	duplicate = 0
	for arrival in arrivals:
		if master_arrivals.count(arrival) > 0:
			# Event is a duplicate.
			duplicate = -1
	
	# Adding all arrivals to master_arrivals:
	for arrival in arrivals:
		master_arrivals.append(arrival)
	
	if (duplicate == -1):
		continue
	
	# Event is new. Storing event as new event:
	new_events.append(str(i))

# Writing new .IMpoly file:
F1 = open(f1 + '.IMpoly','r')
F2 = open(f1 + '.clean.IMpoly','w')
for line in F1:
	if new_events.count(line.split()[0]) == 1:
		print >> F2, line.rstrip()
F1.close()
F2.close()

# Writing new .IMassoc file:
F1 = open(f1 + '.IMassoc','r')
F2 = open(f1 + '.clean.IMassoc','w')
for line in F1:
	if new_events.count(line.split()[2]) == 1:
		print >> F2, line.rstrip()
F1.close()
F2.close()

# Writing new .origin file:
F1 = open(f1 + '.origin','r')
F2 = open(f1 + '.clean.origin','w')
i = 0
for line in F1:
	i = i + 1
	if new_events.count(str(i)) == 1:
		print >> F2, line.rstrip()
F1.close()
F2.close()
