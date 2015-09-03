#!/usr/bin/env python
#
# RunInfraMonitor.py
#
# Runs InfraMonitor in pipeline mode
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

import datetime, time, calendar, os, commands, optparse, pdb

def makeTables(startTime,endTime,inputTable,elements):
	'''Extracts the relevant lines from wfdisc, site files for the given startTime, endTime
	Where: startTime and endTime have format mmddyyyy HH:MM:SS
	e.g., startTime = '08/04/2011 00:00:00' 
	      endTime = '08/14/2011 23:59:59' '''
	
	# Splitting up elements for generating the wfdisc file:
	elementsSeparated = []
	for element in elements:
		element = element.split(',')
		for individualElement in element:
			elementsSeparated.append(individualElement)

	# Generate temporary wfdisc and site file relevant for the run:

	# (a) wfdisc file:
	try:
		startTimeEpoch = calendar.timegm(time.strptime(startTime,'%m/%d/%Y %H:%M:%S'))
		endTimeEpoch = calendar.timegm(time.strptime(endTime,'%m/%d/%Y %H:%M:%S'))
		f = open(inputTable + '.wfdisc','r')
		f_out = open(inputTable + '_out_tmp.wfdisc','w')
		for line in f:
			if (elementsSeparated.count(line.split()[0]) > 0):
				if ((float(line.split()[2]) <= startTimeEpoch and startTimeEpoch <= float(line.split()[6])) or (float(line.split()[2]) <= endTimeEpoch and endTimeEpoch <= float(line.split()[6]))):
					# Print it out and continue
					print >> f_out, line.rstrip()
					continue
				elif ((startTimeEpoch <= float(line.split()[2]) and float(line.split()[2]) <= endTimeEpoch) or (startTimeEpoch <= float(line.split()[6]) and float(line.split()[6]) <= endTimeEpoch)):
					print >> f_out, line.rstrip()
	except:
		print '---------------------------------------------------------------------------------------------------------------'
		print 'There is a problem with your wfdisc file'
		print 'Please use the Python script RobustWfdiscReader.py in the InfraMonitor PYTHON folder to reformat your wfdisc'
		print 'file'
		print '<<Entering debug mode now to enable you to inspect the workspace. Enter q to return to your unix prompt>>'
		print '---------------------------------------------------------------------------------------------------------------'
		pdb.set_trace()
	
	# Sorting wfdisc file:
	f.close(); f_out.close()
	f = open(inputTable + '_out.wfdisc','w')
	wfdisc = commands.getoutput('cat ' + inputTable + '_out_tmp.wfdisc').splitlines()
	wfdisc.sort()
	for line in wfdisc:
		print >> f, line.rstrip()
	f.close()
	os.system('rm ' + inputTable + '_out_tmp.wfdisc')
	
	# (b) site file:
	try:
		startDate = int(datetime.datetime.strptime(startTime.split()[0],'%m/%d/%Y').strftime('%Y%j'))
		f = open(inputTable + '.site','r')
		f_out = open(inputTable + '_out.site','w')
		for line in f:
			if (elementsSeparated.count(line.split()[0]) > 0):
				#if (int(line.split()[1]) < startDate & startDate < int(line.split()[2])):
					print >> f_out, line.rstrip()
		f.close(); f_out.close()
	except:
		print '---------------------------------------------------------------------------------------------------------------'
		print 'There is a problem with your site file'
		print 'Please use the Python script RobustSiteReader.py in the InfraMonitor PYTHON folder to reformat your wfdisc'
		print 'file'
		print '<<Entering debug mode now to enable you to inspect the workspace. Enter q to return to your unix prompt>>'
		print '---------------------------------------------------------------------------------------------------------------'
		pdb.set_trace()

def processData(startDate,endDate,inputTable,elementFile,tableFormat,fband,minArrays,matlabPath,inframonitorPath):
	'''Processes data with InfraMonitor
	e.g., RunInfraMonitor.processData('08/05/2011','08/14/2011','korea','elements.txt')'''
	
	# Reading element file:
	elements = []
	f = open(elementFile,'r')
	for line in f:
		elements.append(line.rstrip())
	
	# Sorting elements:
	elements.sort()
		
	if (elements == []):
		print '---------------------------------------------------------------------------------------------------------------'
		print 'Your elements file is empty or does not exist'
		print 'Please refer to the Users Manual for an example of what should be in here'
		print '<<Entering debug mode now to enable you to inspect the workspace. Enter q to return to your unix prompt>>'
		print '---------------------------------------------------------------------------------------------------------------'
		pdb.set_trace()
	
	try:
		startDate = startDate.split('/')
		startDate = datetime.date(int(startDate[2]),int(startDate[0]),int(startDate[1]))
		endDate = endDate.split('/')
		endDate = datetime.date(int(endDate[2]),int(endDate[0]),int(endDate[1]))
		ndays = (endDate - startDate).days
	except:
		print '---------------------------------------------------------------------------------------------------------------'
		print 'There is a problem with the dates you provided'
		print 'Check the format of your start and end dates is mm/dd/yyyy'
		print '<<Entering debug mode now to enable you to inspect the workspace. Enter q to return to your unix prompt>>'
		print '---------------------------------------------------------------------------------------------------------------'
		pdb.set_trace()
	
	# Splits the durations into individual days, processing each day separately:
	for i in range(0, ndays+1):
		
		# Extracting the day to process (x):
		x = startDate + datetime.timedelta(days=i)
		x = str(x).split('-')[1] + '/' + str(x).split('-')[2] + '/' + str(x).split('-')[0]
		
		# Generating the wfdisc and site file:
		makeTables(x + ' 00:00:00',x + ' 23:59:59',inputTable,elements)
		
		if (not(os.path.exists(inputTable + "_out.wfdisc")) or not(os.path.exists(inputTable + "_out.site"))):
			print '---------------------------------------------------------------------------------------------------------------'
			print 'Temporary wfdisc files for day ' + str(i) + ' were not formed'
			print 'Please use the Python scripts RobustWfdiscReader.py and RobustSiteReader.py in the InfraMonitor'
			print 'PYTHON folder to reformat your wfdisc and site files'
			print '<<Entering debug mode now to enable you to inspect the workspace. Enter q to return to your unix prompt>>'
			print '---------------------------------------------------------------------------------------------------------------'
			pdb.set_trace()
		
		# Generating an InfraMonitor input file:
		try:
			fband = fband.split(',')[0] + ' ' + fband.split(',')[1]
		except:
			print '---------------------------------------------------------------------------------------------------------------'
			print 'The frequency band input is formatted incorrectly'
			print '<<Entering debug mode now to enable you to inspect the workspace. Enter q to return to your unix prompt>>'
			print '---------------------------------------------------------------------------------------------------------------'
			pdb.set_trace()
		
		f = open('matlabInput.m','w')
		print >> f, "addpath(genpath('" + inframonitorPath + "'))"
		print >> f, "InfraMonitor3b2('.','" + inputTable + "_out.wfdisc',[" + fband + "],2," + minArrays + ",'" + tableFormat + "');"
		print >> f, 'quit'
		f.close()
		
		# Running InfraMonitor:
		os.system(matlabPath + ' -nojvm -nodisplay < matlabInput.m')
		
		# Cleaning up:
		os.system('mv ' + inputTable + '_out.arrival ' + x.replace('/','_') + '.arrival')
		os.system('mv ' + inputTable + '_out.detect ' + x.replace('/','_') + '.detect')
		os.system('mv ' + inputTable + '_out.origin ' + x.replace('/','_') + '.origin')
		os.system('mv ' + inputTable + '_out.IMassoc ' + x.replace('/','_') + '.IMassoc')
		os.system('mv ' + inputTable + '_out.IMpoly ' + x.replace('/','_') + '.IMpoly')
		os.system('rm ' + inputTable + '_out.wfdisc')
		os.system('rm ' + inputTable + '_out.site')

def inElementFile(line,elementFile,tableName):
	# Returns True if any of the elements is in the wfdisc file, False otherwise
	elements = line.split(',')
	for element in elements:
		element = element.rstrip()
		nlines = commands.getoutput("grep " + element + " " + tableName + ".wfdisc | wc | awk '{print $1}'")
		if (int(nlines) > 0):
			return True
		else:
			continue
	return False

def main():
	
	try:
		p = optparse.OptionParser()
	except:
		print "Invalid Arguments: Type RunInfraMonitor.py -h for help"
		return
	
	p.add_option('--tableName','-t',default='NaN',help="The name of the wfdisc file you want to process, without any extension")
	p.add_option('--tableFormat','-T',default='NaN',help="The format of the wfdisc and corresponding site files (use either css3.0 or nnsa)")
	p.add_option('--startDate','-s',default='NaN',help="The start date for processing (format=mm/dd/yyyy e.g., 08/15/2011)")
	p.add_option('--endDate','-e',default='NaN',help="The end date for processing (format=mm/dd/yyyy e.g., 08/15/2011)")
	p.add_option('--elementFile','-f',default='NaN',help="The element file (see Users Guide for more information)")
	p.add_option('--gridBounds','-g',default='NaN',help="Lat/lon bounds defining the geographic grid, e.g., 33,46,121,145")
	p.add_option('--gridSpacing','-G',default='NaN',help="The grid spacing in degrees, e.g., 0.1")
	p.add_option('--frequencyBand','-F',default='NaN',help="The frequency band, e.g., 1,5")
	p.add_option('--minArrays','-m',default='NaN',help="The minimum number of arrays for event association, e.g., 3")
	p.add_option('--matlabPath','-M',default='NaN',help="The full path to your matlab executable e.g., /Applications/MATLAB_R2011b.app/bin/matlab")
	p.add_option('--inframonitorPath','-i',default='NaN',help="The full path to your InfraMonitor folder e.g., /home/arrows/InfraMonitor_3.1")
	
	options, arguments = p.parse_args()
	if (options.tableName == 'NaN' or options.tableFormat == 'NaN' or options.startDate == 'NaN' or options.endDate == 'NaN' or options.elementFile == 'NaN' or options.gridBounds == 'NaN' or options.gridSpacing == 'NaN' or options.frequencyBand == 'NaN' or options.minArrays == 'NaN' or options.matlabPath == 'NaN' or options.inframonitorPath == 'NaN'):
		print "Invalid Arguments: Type MakeWfdisc.py -h for help"
		return
	
	# Making the grid file:
	try:
		gridBounds = options.gridBounds.split(',')[0] + ' ' + options.gridBounds.split(',')[1] + ' ' + options.gridBounds.split(',')[2] + ' ' + options.gridBounds.split(',')[3]
		gridSpacing = options.gridSpacing + ' ' + options.gridSpacing
		f_elements = open(options.elementFile,'r')
		lats = []; lons = []
		for line in f_elements:
			if inElementFile(line,options.elementFile,options.tableName):
				element = line.split(',')[0]
				lats.append(commands.getoutput("grep " + element + " " + options.tableName + ".site | head -1").split()[3])
				lons.append(commands.getoutput("grep " + element + " " + options.tableName + ".site | head -1").split()[4])
	except:
		print '---------------------------------------------------------------------------------------------------------------'
		print 'There was a problem with making the grid file'
		print 'Please check your input grid bounds has commas separating the values, e.g., 33,46,121,145'
		print 'and that the grid spacing is a floating point number, e.g., 0.1'
		print 'and check your elementFile. Does it exist in the working directory? Is it formatted properly (see Users Guide)'
		print '<<Entering debug mode now to enable you to inspect the workspace. Enter q to return to your unix prompt>>'
		print '---------------------------------------------------------------------------------------------------------------'
		pdb.set_trace()
	
	f = open('matlabInput.m','w')
	print >> f, "addpath(genpath('" + options.inframonitorPath + "'))"
	print >> f, "MakeGrid('" + options.tableName + "_out.mat',[" + gridBounds + "],[" + gridSpacing + "]," + str(lats).replace("'","").replace(",","") + "," + str(lons).replace("'","").replace(",","") + ");"
	print >> f, "quit"
	f.close()
	
	os.system(options.matlabPath + ' -nojvm -nodisplay < matlabInput.m')
	
	if (not(os.path.exists(options.tableName + "_out.mat"))):
		print '---------------------------------------------------------------------------------------------------------------'
		print 'Failed to generate the grid file in Matlab'
		print 'Check that your Matlab and InfraMonitor paths are correct'
		print 'Check that matlabInput.m has been created'
		print 'To help you debug this problem, try running matlabInput.m directly inside Matlab'
		print '<<Entering debug mode now to enable you to inspect the workspace. Enter q to return to your unix prompt>>'
		print '---------------------------------------------------------------------------------------------------------------'
		pdb.set_trace()
	
	# Processing the waveform data
	processData(options.startDate,options.endDate,options.tableName,options.elementFile,options.tableFormat,options.frequencyBand,options.minArrays,options.matlabPath,options.inframonitorPath)

if __name__ == '__main__':
	main()
