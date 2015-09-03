#!/usr/bin/env python
#
# RobustWfdiscReader.py
#
# Reads in an incorrectly formatted wfdisc file and formats it with nnsa format
# (Requires the site file to have NA values inserted appropriately)
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

import optparse, pdb

def main():
	
	p = optparse.OptionParser()
	p.add_option('--inputfile','-i',default='NaN')
	p.add_option('--outputfile','-o',default='NaN')
	options, arguments = p.parse_args()
	if (options.inputfile == 'NaN'):
		print "Invalid Arguments: Type RobustWfdiscReader.py -h for help"
		return
	
	f_in = open(options.inputfile,'r')
	f_out = open(options.outputfile,'w')

	for line in f_in:
	
		line = line.split()
	
		# Cutting off the end (assuming it is lddate)
		if (len(line) > 20):
			line = line[0:20]
	
		outputLine = '%-6s %-8s %17.5f %9d %8d %8d %17.5f %8d %11.7f %16.6f %16.6f %-6s %-1s %-2s %-1s %-64s %-32s %10d %8d %-17s'\
		 % (line[0],line[1],float(line[2]),int(line[3]),int(line[4]),int(line[5]),float(line[6]),int(line[7]),float(line[8]),float(line[9]),float(line[10]),line[11],line[12],line[13],\
		line[14],line[15],line[16],int(line[17]),int(line[18]),line[19])
		print >> f_out, outputLine

if __name__ == '__main__':
	main()
