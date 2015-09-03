function data = waveread ( filename, datatype, byteoffset, num )
%WAVEREAD	Read waveform data file.
%	DATA = WAVEREAD(FILENAME,DATATYPE,BYTEOFFSET,NUM) reads NUM data values
%	from a single FILENAME using format DATATYPE skipping BYTEOFFSET bytes
%	from the beginning of the file.
%
%	DATATYPE may be any of the following:
%
%	    DATATYPE    DESCRIPTION
%	    --------    -------------------------------------------------------
%	    cc          Compressed (variable length)
%	    e1          Compressed (variable length)
%	    a2          AFTAC gain-ranged (2 bytes long)
%	    g2          NORESS gain-ranged (2 bytes long)
%	    t4          SUN single precision real (4 bytes long)
%	    t8          SUN double precision real (8 bytes long)
%	    s4          SUN integer (4 bytes long)
%	    s3          SUN integer (3 bytes long)
%	    s2          SUN integer (2 bytes long)
%	    s1          SUN integer (1 byte long)
%	    f8          VAX double precision real (8 bytes long)
%	    f4          VAX single precision real (4 bytes long)
%	    i4          VAX or PC integer (4 bytes long)
%	    i3          VAX or PC integer (3 bytes long)
%	    i2          VAX or PC integer (2 bytes long)
%	    i1          VAX or PC integer (1 byte long)
%	    p8          PC double precision real (8 bytes long)
%	    p4          PC single precision real (4 bytes long)
%	    a0          ASCII single precision (15 characters long)
%	    a#          ASCII single precision (15 characters long)
%	    b0          ASCII double precision (24 characters long)
%	    b#          ASCII double precision (24 characters long)
%	    c0          ASCII integer (12 characters long)
%	    c#          ASCII integer (12 characters long)
%	    ascii       any ASCII format (whitespace separated)
%
%	or DATATYPE may be any valid matlab precision argument (see FREAD).
%	If DATATYPE does not match one of the above, then the file is opened
%	in 'native' mode and DATATYPE is passed to FREAD unchanged as the
%	precision argument.
%
%	DATATYPE defaults to 'ascii', BYTEOFFSET defaults to 0, and NUM
%	defaults to inf.
%
%	The output DATA is a column vector.
%
%	See also FOPEN, FREAD, FSCANF.
% Copyright (c) 2012, Los Alamos National Security, LLC
% All rights reserved.
% 
% Copyright 2012. Los Alamos National Security, LLC. This software was produced under U.S.
% Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is
% operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S.
% Government has rights to use, reproduce, and distribute this software.  NEITHER THE
% GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED,
% OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce
% derivative works, such modified software should be clearly marked, so as not to confuse it
% with the version available from LANL.
% 
% Additionally, redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% ·         Redistributions of source code must retain the above copyright notice, this list
% 		  of conditions and the following disclaimer.
% ·         Redistributions in binary form must reproduce the above copyright notice, this
% 	      list of conditions and the following disclaimer in the documentation and/or
% 	      other materials provided with the distribution.
% ·         Neither the name of Los Alamos National Security, LLC, Los Alamos National
% 	      Laboratory, LANL, the U.S. Government, nor the names of its contributors may be
% 	      used to endorse or promote products derived from this software without specific
% 	      prior written permission.
% THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
% EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
% INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
% LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
% OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
% WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

%	MatSeis 1.8
%	Mark Harris, mharris@sandia.gov
%	Copyright (c) 1996-2004 Sandia National Laboratories. All rights reserved.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check arguments.
%
if nargin < 1
  data = [];
  return;
else
  if ( exist ('ms_deblank') )
    filename = ms_deblank ( filename(1,:) );
  end
end
if nargin < 2
  datatype = 'ascii';
elseif isempty(datatype)
  datatype = 'ascii';
else
  if ( exist ('ms_deblank') )
    datatype = ms_deblank ( datatype(1,:) );
  end
end
if nargin < 3
  byteoffset = 0;
else
  byteoffset = byteoffset(1);
end
if nargin < 4
  num = inf;
else
  num = num(1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read specified datatype.
%
switch datatype

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS cc: Canadian Compressed (variable length)
  %
  case 'cc'
    data = read_cc ( filename, byteoffset, num );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS e1: Compressed (variable length)
  %
  case 'e1'
    data = read_e1 ( filename, byteoffset, num );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS a2: AFTAC gain-ranged (2 bytes long)
  %
  case 'a2'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-le', 'uint16' );
    data = cnv_a2 ( data );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS g2: NORESS gain-ranged (2 bytes long)
  %
  case 'g2'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-be', 'uint16' );
    data = cnv_g2 ( data );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS t8: SUN double precision real (8 bytes long)
  %
  case 't8'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-be', 'float64' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS t4: SUN single precision real (4 bytes long)
  %
  case 't4'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-be', 'float32' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS s4: SUN integer (4 bytes long)
  %
  case 's4'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-be', 'int32' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS s3: SUN integer (3 bytes long)
  %
  case 's3'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-be', 'bit24' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS s2: SUN integer (2 bytes long)
  %
  case 's2'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-be', 'int16' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS s1: SUN integer (1 byte long)
  %
  case 's1'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-be', 'int8' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS f8: VAX double precision real (8 bytes long)
  %
  case 'f8'
    data = read_file ( filename, byteoffset, num, 'rb', 'vaxd', 'float64' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS f4: VAX single precision real (4 bytes long)
  %
  case 'f4'
    data = read_file ( filename, byteoffset, num, 'rb', 'vaxd', 'float32' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS i4: VAX integer (4 bytes long)
  %
  case 'i4'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-le', 'int32' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS i3: VAX integer (3 bytes long)
  %
  case 'i3'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-le', 'bit24' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS i2: VAX integer (2 bytes long)
  %
  case 'i2'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-le', 'int16' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS i1: VAX integer (1 byte long)
  %
  case 'i1'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-le', 'int8' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS p8: PC double precision real (8 bytes long)
  %
  case 'p8'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-le', 'float64' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS p4: PC single precision real (4 bytes long)
  %
  case 'p4'
    data = read_file ( filename, byteoffset, num, 'rb', 'ieee-le', 'float32' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS a0 & a#: ASCII single precision (15 characters long)
  %
  case {'a0','a#'}
    data = read_file ( filename, byteoffset, num, 'rt', 'native', '%f' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS b0 & b#: ASCII double precision (24 characters long)
  %
  case {'b0','b#'}
    data = read_file ( filename, byteoffset, num, 'rt', 'native', '%f' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % CSS c0 & c#: ASCII integer (12 characters long)
  %
  case {'c0','c#'}
    data = read_file ( filename, byteoffset, num, 'rt', 'native', '%f' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % ascii: any ASCII format
  %
  case 'ascii'
    data = read_file ( filename, byteoffset, num, 'rt', 'native', '%f' );

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % native matlab precision
  %
  otherwise
    data = read_file ( filename, byteoffset, num, 'rb', 'native', datatype );

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                              SUBFUNCTION                               %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function data = read_file ( filename, byteoffset, num, ...
                            permission, machineformat, precision )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read file.
%
% 1) Open the file using permission and machineformat.
% 2) Seek to byteoffset from the beginning of the file.
% 3) Read the file, with fread if binary, fscanf if ascii.
% 4) Close the file.
%
fid = fopen ( filename, permission, machineformat );
if fid == -1
  disp ( [ 'Could not open data file: ''' filename '''' ] );
  data = [];
  return;
end

if byteoffset > 0
  fseek ( fid, byteoffset, 'bof' );
end

switch permission
case 'rb'
  data = fread ( fid, num, precision );
case 'rt'
  data = fscanf ( fid, precision, num );
end
fclose ( fid );
