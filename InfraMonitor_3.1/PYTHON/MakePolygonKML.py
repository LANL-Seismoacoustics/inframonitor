#!/usr/bin/env python
#
# MakePolygonKML.py
#
# Writes a Google Earth kml file for an input IMpoly table
#
# Example:
# MakePolygonKML -i UTAH.IMpoly -o UTAH.kml
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
	
	# Reading Arguments:
	p = optparse.OptionParser()
	p.add_option('--inputfile','-i',default='NaN')
	p.add_option('--outputfile','-o',default='IM2Polygon.kml')
	options, arguments = p.parse_args()
	if (options.inputfile == 'NaN'):
		print "Invalid Arguments: Type IM2.py -h for help"
		retu
	
	# Writing KML file header:
	f1 = open(options.outputfile,'w')
	
	header = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Infrasound Event Polygons</name>
    <description>
       from InfraMonitor2
    </description>
    <Style id="redLinePoly">
      <LineStyle>
        <color>ff0000ff</color>
        <width>5</width>
      </LineStyle>
    </Style>'''
    
	print >> f1, header
	
	i = 0
	f2 = open(options.inputfile,'r')
	CurrentPolyID = -999
	for line in f2:
		line = line.split()
		if (int(line[0]) != CurrentPolyID):
			i = i + 1
			if (i != 1):
				print >> f1, '''        </coordinates>
      </LineString>
    </Placemark>'''
			CurrentPolyID = int(line[0])
			print >> f1, '''    <Placemark>
	  <name>''' + str(CurrentPolyID) + '''</name>
	  <styleUrl>#redLinePoly</styleUrl>
	  <LineString>
		<extrude>1</extrude>
		<tessellate>1</tessellate>
		<altitudeMode>relativeToGround</altitudeMode>
		<coordinates>'''
			print >> f1, '          ' + str(line[3]) + "," + str(line[2]) + ",0"
		else:
			print >> f1, '          ' + str(line[3]) + "," + str(line[2]) + ",0"
	
	print >> f1, '''        </coordinates>
      </LineString>
    </Placemark>'''
	print >> f1, '''  </Document>
</kml>'''
	
	f1.close()
	f2.close()

if __name__ == '__main__':
	main()
