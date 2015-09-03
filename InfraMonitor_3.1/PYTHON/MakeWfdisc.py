#!/usr/bin/env python
#
# MakeWfdisc.py
#
# Makes a wfdisc file for a set of SAC files
#
# Usage: MakeWfdisc.py -i sac_file_extension -o wfdisc_file -p path
#
# e.g.,
# MakeWfdisc -i sac -o Airport.wfdisc -p /Users/arrows/Documents/CODE/PYTHON/MyModules
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

# Importing modules:
import sys, os, commands, time, datetime, calendar, pdb, optparse

def main():
	
	try:
		p = optparse.OptionParser()
	except:
		print "Invalid Arguments: Type MakeWfdisc.py -h for help"
		return
	
	p.add_option('--inputfileextension','-i',default='NaN')
	p.add_option('--outputfile','-o',default='NaN')
	p.add_option('--path','-p',default='NaN')
	options, arguments = p.parse_args()
	if (options.inputfileextension == 'NaN' or options.outputfile == 'NaN' or options.path == 'NaN'):
		print "Invalid Arguments: Type MakeWfdisc.py -h for help"
		return
	
	sac_files = options.inputfileextension
	sac_files = commands.getoutput('ls *.' + sac_files).splitlines()
	wfdisc_file = options.outputfile
	path = options.path
	
	sys.path.append(path)
	import pysacio

	def julday(hi):
	
		date = commands.getoutput('calday ' + str(hi[1]) + ' ' + str(hi[0])).splitlines()[2]
		day = date.split()[3]; month = date.split()[2]
		return day, month

	def unixtime(hi):
	
		[d,m] = julday(hi)
		t = str(hi[0]) + '-' + str(m) + '-' + str(d) + ' ' + str(hi[2]) + ':' + \
			str(hi[3]) + ':' + str(hi[4])
		t = int(calendar.timegm(time.strptime(t,'%Y-%m-%d %H:%M:%S')))
		return t

	# Writing out temporary wfdisc file:
	f = open(wfdisc_file + '_tmp','w')
	i = 0
	for sac_file in sac_files:
		i = i + 1
		[hf, hi, ss, hs, ok] = pysacio.ReadSacHeader(sac_file)
	#	pdb.set_trace()
		try:
			print >> f, '%5s%3s  %16s %8s %8s %8s %17s %8s %11s%s%s%s' % \
				(hs[0],hs[19][0:7],str(unixtime(hi)) + '.' + str(hi[5]) + '00',\
				str(i),'-1','-1',str(unixtime(hi) + int(hi[9]*hf[0])) + '.' + str(hi[5]) + '00',\
				hi[9],int(1/hf[0]),'                0        -1.000000 REF-TE - t4 - .                                                                ',\
				sac_file,'                                632       -1  1235421064.33805')
		except:
			pdb.set_trace()
			continue

	f.close()
	
	# Formatting wfdisc file:
	f = open(wfdisc_file + '_tmp','r')
	f_out = open(wfdisc_file,'w')

	for line in f:
		line = line.split()
		outputLine = '%-6s %-8s %17.5f %8d %8d %8d %17.5f %8d %11.7f %16.6f %16.6f %-6s %-1s %-2s %-1s %-64s %-32s %10d %8d %-17s'\
		 % (line[0],line[1],float(line[2]),int(line[3]),int(line[4]),int(line[5]),float(line[6]),int(line[7]),float(line[8]),float(line[9]),float(line[10]),line[11],line[12],line[13],\
		line[14],line[15],line[16],int(line[17]),int(line[18]),line[19])
		print >> f_out, outputLine

	f.close(); f_out.close()

	os.system('rm ' + wfdisc_file + '_tmp')

if __name__ == '__main__':
	main()
