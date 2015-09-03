#!/usr/bin/env sage -python
#
# MakeSite.py
#
# Makes a site file for an input wfdisc file
#
# e.g.,
# MakeSite.py -i utahOut.wfdisc -p /Users/arrows/Documents/CODE/PYTHON/MyModules
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

import os, commands, sys, optparse, pdb

def main():
	
	# Add sensors to refsta to populate refsta field:
	refsta = {'BGU1':'BGU1','BGU2':'BGU1','BGU3':'BGU1','BGU4':'BGU1',\
			  'BRP1':'BRP1','BRP2':'BRP1','BRP3':'BRP1','BRP4':'BRP1',\
			  'FSU1':'FSU1','FSU2':'FSU1','FSU3':'FSU1','FSU4':'FSU1'}
	
	# Reading Arguments:
	p = optparse.OptionParser()
	p.add_option('--inputfile','-i',default='NaN')
	p.add_option('--path','-p',default='NaN')
	options, arguments = p.parse_args()
	if (options.inputfile == 'NaN' or options.path == 'NaN'):
		print "Invalid Arguments: Type MakeSite.py -h for help"
		return
	
	wfdisc_file = options.inputfile
	path = options.path
	
	sys.path.append(options.path)
	import pysacio
	
	f1 = open(wfdisc_file,'r')
	f3 = open(wfdisc_file.replace('.wfdisc','.site2'),'w')

	processed_arrays = []
	for a in f1:
	
		sacFile = a.split()[16]
		print sacFile
		
		[hf, hi, ss, hs, ok] = pysacio.ReadSacHeader(sacFile)
		stnLat = hf[31]; stnLon = hf[32]; stnElev = hf[33]/1000
		print >> f3, ss.split()[0], '-1', '-1', stnLat, stnLon, stnElev

	f1.close(); f3.close()

	f = open(wfdisc_file.replace('.wfdisc','.site2'),'r')
	f_out = open(wfdisc_file.replace('.wfdisc','.site'),'w')
	for line in f:
		line = line.split()
		#pdb.set_trace()
		try:
			outputLine = '%-6s %8d %8d %9.4f %9.4f %9.4f %-50s %-4s %-6s %9.4f %9.4f %-17s'\
		 	% (line[0],int(line[1]),int(line[2]),float(line[3]),float(line[4]),float(line[5]),line[0],'ar',refsta[line[0]],0.0,0.0,'13-DEC-94')
		except:
			outputLine = '%-6s %8d %8d %9.4f %9.4f %9.4f %-50s %-4s %-6s %9.4f %9.4f %-17s'\
		 	% (line[0],int(line[1]),int(line[2]),float(line[3]),float(line[4]),float(line[5]),line[0],'ar','-',0.0,0.0,'13-DEC-94')
		print >> f_out, outputLine

	f.close(); f_out.close()
	
	os.system('rm ' + wfdisc_file.replace('.wfdisc','.site2'))

if __name__ == '__main__':
	main()
