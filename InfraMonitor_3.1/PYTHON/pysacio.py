###############################################################################
# This is a set of python functions for reading and writing
#   sac files. 
#
# Version: 0.80, November, 2001
#  Author: Charles J. Ammon, Penn State, Saint Louis University
#
# You would call these functions from a python script.
# To use them, place the file pysacio.py in your python path
# and include the command 'import pysacio' at the top of your
# script. Then prepend the name of the macro package (pysacio)
# on the routines. For example, to use "ReadSacFile", you 
# would call "pysacio.ReadSacFile".
#
###############################################################################
#
import struct,array,os,string
#
###############################################################################
#
# These dictionaries are for SAC header access
#   use the function GetHvalue('delta',hf,hi,hs)
#   it will return a float, integer, or string
#
fdict = {'delta':0, 'depmin':1, 'depmax':2, 'scale':3,   \
	 'odelta':4, 'b':5, 'e':6, 'o':7, 'a':8, 't0':10,\
	 't1':11,'t2':12,'t3':13,'t4':14,'t5':15,'t6':16,\
	 't7':17,'t8':18,'t9':19,'stla':31,'stlo':32,    \
	 'stel':33,'stdp':34,'evla':35,'evlo':36,'evdp':38,\
	 'user0':40,'user1':41,'user2':42,'user3':43,\
	 'user4':44,'user5':45,'user6':46,'user7':47,\
	 'user8':48,'user9':49,'dist':50,'az':51,'baz':52,\
	 'gcarc':53,'depmen':56,'cmpaz':57,'cmpinc':58}
#
idict = {'nzyear':0, 'nzjday':1, 'nzhour':2, 'nzmin':3, \
	 'nzsec':4, 'nzmsec':5, 'nvhdr':6, 'npts':9, \
	 'iftype':15,'idep':16,'iztype':17,'iinst':19,\
	 'istreg':20,'ievreg':21,'ievtype':22,'iqual':23,\
	 'isynth':24,'leven':35,'lpspol':36,'lovrok':37,\
	 'lcalda':38}

sdict = {'kstnm':0,'kevnm':1,'khole':2, 'ko':3,'ka':4,\
		'kt0':5,'kt1':6,'kt2':7,'kt3':8,'kt4':9,\
		'kt5':10,'kt6':14,'kt7':12,'kt8':13,\
		'kt9':14,'kf':15,'kuser0':16,'kuser1':17,\
		'kuser2':18,'kcmpnm':19,'knetwk':20,\
		'kdatrd':21,'kinst':22}
#
###############################################################################
#
#   access a sac file header values using a string
#    the type returned depends on the what you ask for
#
#   usage: dt = GetHvalue('delta',hf,hi,hs)
#
#     returns the string 'NULL' if nothing matches the item
#
#
def GetHvalue(item,hf,hi,hs):
	#
	# it's trivial to search each dictionary with the key and return
	#   the value that matches the key
	#
	key = string.lower(item) # convert the item to lower case
	#
	if fdict.has_key(key):
		index = fdict[key]
		return(hf[index])
	elif idict.has_key(key):
		index = idict[key]
		return(hi[index])
	elif sdict.has_key(key):
		index = sdict[key]
		return(hs[index])
	else:
		return('NULL')
#
###############################################################################
#
#   alter a sac file header values using a string and a value
#    
#
#   usage: SetHvalue('delta',1.0, hf,hi,ss,hs)
#
#     sets the dt value to 1.0 
#
#
def SetHvalue(item,value,hf,hi,ss,hs):
	#
	# it's trivial to search each dictionary with the key and return
	#   the value that matches the key
	#
	key = string.lower(item) # convert the item to lower case
	#
	ok = 0
	if fdict.has_key(key):
		index = fdict[key]
		hf[index] = float(value)
		ok = 1
	elif idict.has_key(key):
		index = idict[key]
		hi[index] = int(value)
		ok = 1
	elif sdict.has_key(key):
		index = sdict[key]
		print 'Sorry, can\'t do the strings yet.'
		ok = 0
	return(ok)
#
#
###############################################################################
#
# Check for a valid SAC file (returns 1 if SAC File, 0 if not a SAC File)
#
#  Right now the checks are very basic (what else can you do?)
#    I check for a positive dt and npts, i could use the version, but
#    this isn't even listed in the ASCII definition of the header...
#    implements a file-size check where npts*4 + headerbytes (632)
#    is compared with the true size of the file.
#
#  usage:   ok = IsSACfile(name,hf,hi,hs)
#
#           if ok = 1, it is a SAC file, if ok = 0, it's not
#   
def IsSACfile(name,hf,hi,hs):
	#
	ok = 1
	#
	# size check info
	#
	npts = GetHvalue('npts',hf,hi,hs)
	st = os.stat(name) #file's size = st[6] 
	sizecheck = st[6] - (632 + 4 * npts)
	#
	# get the SAC file version number
	#
	version = GetHvalue('nvhdr',hf,hi,hs)
	#
	# if any of these conditions are true,
	#   the file is NOT a SAC file
	#
	if GetHvalue('delta',hf,hi,hs) <= 0:
		ok = 0
	elif sizecheck != 0:
		ok = 0
	elif ((version < 0) or (version > 20) ):
		ok = 0
	#
	return(ok)
#
#
###############################################################################
#
# Check for a valid SAC file (returns 1 if SAC File, 0 if not a SAC File)
#
#  Right now the checks are very basic (what else can you do?)
#    I check for a positive npts, i could use the version, but
#    this isn't even listed in the ASCII definition of the header...
#    implements a file-size check where npts*4 + headerbytes (632)
#    is compared with the true size of the file.
#
#  usage:   ok = IsXYSACfile(name,hf,hi,hs)
#
#           if ok = 1, it is a SAC file, if ok = 0, it's not
#   
def IsXYSACfile(name,hf,hi,hs):
	#
	ok = 1
	#
	# size check info
	#
	npts = GetHvalue('npts',hf,hi,hs)
	st = os.stat(name) #file's size = st[6] 
	sizecheck = st[6] - (632 + 2 * 4 * npts)
	#
	# get the SAC file version number
	#
	version = GetHvalue('nvhdr',hf,hi,hs)
	#
	# if any of these conditions are true,
	#   the file is NOT a SAC file
	#
	if sizecheck != 0:
		ok = 0
	elif ((version < 0) or (version > 20) ):
		ok = 0
	#
	return(ok)
#
###############################################################################
#
#  usage:   [hf, hi, ss, hs, ok] = ReadSacHeader(file_name)
#  returns:
#          The header values (70 floats, 40 integers, 
#          192 characters) are returned in an array of floats,
#          an array of integers, a buffer of bytes, 
#          and a list of strings.
#          if ok = 1, it succeeded, if ok = 0, it failed
#
def ReadSacHeader(fname='mysacfile'):
	#
	hf = array.array('f') # allocate the array for header floats
	hi = array.array('l') # allocate the array for header ints
	hs = ('s') # allocate a space for strings
	ss = 's'
	#--------------------------------------------------------------
	# open the file
	#
	try:
		f = open(fname,'r')
		#--------------------------------------------------------------
		# parse the header
		#
		# The sac header has 70 floats, then 40 integers, then 192 bytes
		#    in strings. Store them in array (an convert the char to a
		#    list). That's a total of 632 bytes.
		#--------------------------------------------------------------
		hf.fromfile(f,70)     # read in the float values
		hi.fromfile(f,40)     # read in the int values
		ss = f.read(192)      # strings in one long buffer
		#
		f.close()
		#
		# break the string values up into a list called hs
		sfmt = '8s16s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s'
		hs = struct.unpack(sfmt,ss)
		#
		ok = IsSACfile(fname,hf,hi,hs)
		#
		#--------------------------------------------------------------
	except:
		# make sure we close the file
		ok = 0
		if f.closed == 0: # zero means the file is open
			f.close()
	#--------------------------------------------------------------
	#
	return([hf, hi, ss, hs, ok])
#
###############################################################################
#
#  usage:   ok = WriteSacHeader(file_name,hf, hi, ss, hs)
# 
#
def WriteSacHeader(fname,hf, hi, ss, hs):
	#--------------------------------------------------------------
	# open the file
	#
	try:
		f = open(fname,'r+') # open file for modification
		f.seek(0,0) # set pointer to the file beginning
		# write the header
		hf.tofile(f)
		hi.tofile(f)
		f.write(ss)
		f.close()
		#
		#--------------------------------------------------------------
	except:
		# make sure we close the file
		if f.closed == 0: # zero means the file is open
			f.close()
	#--------------------------------------------------------------
#
#
###############################################################################
#
#  usage:   [hf, hi, ss, hs, seis, ok] = ReadSacFile(file_name)
#  returns:
#          The header values (70 floats, 40 integers, 
#          192 characters) are returned in an array of floats,
#          an array of integers, a buffer of bytes, 
#          and a list of strings.
#          seis is an array of floats (the seismogram)
#          if ok = 1, it succeeded, if ok = 0, it failed
#
def ReadSacFile(fname='mysacfile'):
	#
	seis = array.array('f') # allocate the array for the points
	hf = array.array('f') # allocate the array for header floats
	hi = array.array('l') # allocate the array for header ints
	hs = ('s') # allocate a space for strings
	ss = 's'
	#--------------------------------------------------------------
	# open the file
	#
	try:
		f = open(fname,'r')
		#--------------------------------------------------------------
		# parse the header
		#
		# The sac header has 70 floats, then 40 integers, then 192 bytes
		#    in strings. Store them in array (an convert the char to a
		#    list). That's a total of 632 bytes.
		#--------------------------------------------------------------
		hf.fromfile(f,70)     # read in the float values
		hi.fromfile(f,40)     # read in the int values
		ss = f.read(192)      # strings in one long buffer
		#
		# break the string values up into a list called hs
		sfmt = '8s16s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s'
		hs = struct.unpack(sfmt,ss)
		#
		# only continue if it is a SAC file
		#
		#
		ok = IsSACfile(fname,hf,hi,hs)
		if ok:
			#--------------------------------------------------------------
			# read in the seismogram points
			#--------------------------------------------------------------
			npts = hi[9]  # you just have to know it's in the 10th place
			#             # actually, it's in the SAC manual
			#
			mBytes = npts * 4
			#
			seis = array.array('f')
			seis.fromfile(f,npts) # the data are now in s
		#
		f.close()
		#
		#--------------------------------------------------------------
	except:
		# make sure we close the file
		if f.closed == 0: # zero means the file is open
			f.close()
		ok = 0
	#--------------------------------------------------------------
	#
	return([hf, hi, ss, hs, seis, ok])
#
#
###############################################################################
#
#  usage:   [hf, hi, ss, hs, seis, ok] = ReadSacFile(file_name)
#  returns:
#          The header values (70 floats, 40 integers, 
#          192 characters) are returned in an array of floats,
#          an array of integers, a buffer of bytes, 
#          and a list of strings.
#          seis is an array of floats (the seismogram)
#          if ok = 1, it succeeded, if ok = 0, it failed
#
def ReadXYSacFile(fname='mysacfile'):
	#
	x = array.array('f')
	y = array.array('f')
	hf = array.array('f') # allocate the array for header floats
	hi = array.array('l') # allocate the array for header ints
	hs = ('s') # allocate a space for strings
	ss = 's'
	#--------------------------------------------------------------
	# open the file
	#
	try:
		f = open(fname,'r')
		#--------------------------------------------------------------
		# parse the header
		#
		# The sac header has 70 floats, then 40 integers, then 192 bytes
		#    in strings. Store them in array (an convert the char to a
		#    list). That's a total of 632 bytes.
		#--------------------------------------------------------------
		hf.fromfile(f,70)     # read in the float values
		hi.fromfile(f,40)     # read in the int values
		ss = f.read(192)      # strings in one long buffer
		#
		# break the string values up into a list called hs
		sfmt = '8s16s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s8s'
		hs = struct.unpack(sfmt,ss)
		#
		# only continue if it is a SAC file
		#
		#
		ok = IsXYSACfile(fname,hf,hi,hs)
		if ok:
			#--------------------------------------------------------------
			# read in the seismogram points
			#--------------------------------------------------------------
			npts = hi[9]  # you just have to know it's in the 10th place
			#             # actually, it's in the SAC manual
			#
			mBytes = npts * 4
			#
			y.fromfile(f,npts) # the data are now in s
			x.fromfile(f,npts) # the data are now in s
		#
		f.close()
		#
		#--------------------------------------------------------------
	except:
		# make sure we close the file
		if f.closed == 0: # zero means the file is open
			f.close()
		ok = 0
	#--------------------------------------------------------------
	#
	return([hf, hi, ss, hs, x, y, ok])
#
###############################################################################
#
#   this is set up to accept the header in two arrays and a string
#       and the data as an array.
#
#   usage: ok = WriteSacFile('test',hf, hi, ss, hs, seis)
#
#          if ok = 1, it succeeded, if ok = 0, it failed
#
#
def WriteSacBinary(ofname, hf, hi, ss, hs, seis):
	try:
		f = open(ofname,'wb+')
		hf.tofile(f)
		hi.tofile(f)
		f.write(ss)
		seis.tofile(f)
		f.close()
		ok = 1
	except:
		print 'Error writing file ', ofname
		f.close()
		ok = 0
	return(ok)
#
###############################################################################
#
#  These are some simply utility routines to print out header
#     values if they are not equal to the 'undefined'
#     value of -12345
#	
###############################################################################
def PrintIValue(label='=', value=-12345):
	if value != -12345:
		print label, value
###############################################################################
def PrintFValue(label='=', value=-12345.0):
	if value != -12345.0:
		print '%s %.8g' % (label, value)
###############################################################################
def PrintSValue(label='=', value='-12345'):
	if value != '-12345':
		print label, value
#
###############################################################################
#
# a simple function to list common header values
#
def ListStdValues(hf,hi,hs,s): # h is a header list, s is a float list
	#
	# Seismogram Info:
	#
	nzyear = GetHvalue('nzyear',hf,hi,hs)
	nzjday = GetHvalue('nzjday',hf,hi,hs)
	[month, date, ok] = tutil.monthdate(nzyear, nzjday)
	print '%s %2.2d/%2.2d/%d (%d) %d:%d:%d.%d' % ('\nReference Time = ',    \
				      month, date, \
			              GetHvalue('nzyear',hf,hi,hs), \
	                              GetHvalue('nzjday',hf,hi,hs), \
	                              GetHvalue('nzhour',hf,hi,hs), \
	                              GetHvalue('nzmin',hf,hi,hs),  \
	                              GetHvalue('nzsec',hf,hi,hs),  \
	                              GetHvalue('nzmsec',hf,hi,hs))
	PrintIValue('Npts  = ',GetHvalue('npts',hf,hi,hs))
	PrintFValue('Delta = ',  GetHvalue('delta',hf,hi,hs)  )
	PrintFValue('Begin = ',  GetHvalue('b',hf,hi,hs)  )
	PrintFValue('End   = ',  GetHvalue('e',hf,hi,hs)  )
	PrintFValue('Min   = ',  GetHvalue('depmin',hf,hi,hs)  )
	PrintFValue('Mean  = ',  GetHvalue('depmen',hf,hi,hs)  )
	PrintFValue('Max   = ',  GetHvalue('depmax',hf,hi,hs)  )
	#
	PrintIValue('Header Version = ',GetHvalue('nvhdr',hf,hi,hs))
	#
	# station Info:
	#
	PrintSValue('Station = ',     GetHvalue('kstnm',hf,hi,hs))
	PrintSValue('Channel = ',     GetHvalue('kcmpnm',hf,hi,hs))
	PrintFValue('Station Lat  = ',GetHvalue('stla',hf,hi,hs))
	PrintFValue('Station Lon  = ',GetHvalue('stlo',hf,hi,hs))
	PrintFValue('Station Elev = ',GetHvalue('stel',hf,hi,hs))
	#
	# Event Info:
	#
	PrintSValue('Event       = ',GetHvalue('kevnm',hf,hi,hs))
	PrintFValue('Event Lat   = ',GetHvalue('evla',hf,hi,hs))
	PrintFValue('Event Lon   = ',GetHvalue('evlo',hf,hi,hs))
	PrintFValue('Event Depth = ',GetHvalue('evdp',hf,hi,hs))
	PrintFValue('Origin Time = ',GetHvalue('o',hf,hi,hs))
	#
	PrintFValue('Azimuth        = ',GetHvalue('az',hf,hi,hs))
	PrintFValue('Back Azimuth   = ',GetHvalue('baz',hf,hi,hs))
	PrintFValue('Distance (km)  = ',GetHvalue('dist',hf,hi,hs))
	PrintFValue('Distance (deg) = ',GetHvalue('gcarc',hf,hi,hs))
#
###############################################################################
#

#
def GetHvalueFromFile(thePath,theItem):
	#
	#  Read in the Header
	#
	[hf, hi, ss, hs, ok] = ReadSacHeader(thePath)
	#
	if ok:
		return(GetHvalue(theItem,hf, hi, hs))
	else:
		print "Problem in GeHvalueFromFile."
		return(-12345)
