function l = WriteArrivals(l,i)
%
% i is the array number
% l is the starting ARID
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

global InfraConfig

fid = fopen([InfraConfig.db.dir InfraConfig.db.name '.arrival'],'a');

Schema_File = which('Schema.mat');
Schema = load(Schema_File);

if (i == 0)

    % Looping over each array:
    for i = 1:numel(InfraConfig.array.arr)

        stns = InfraConfig.trace.name(InfraConfig.array.trc{i});

        % Looping over each arrival at the i'th array:
        for j = 1:size(InfraConfig.array.arr{i},1)

            % ----------------------------------------------------
            % Appears to be depreciated:
            if (InfraConfig.array.arr{i}(j,5) == 1)
                continue
            end
            % ----------------------------------------------------

            time = iepoch(InfraConfig.array.arr{i}(j,1));
            azimuth = InfraConfig.array.arr{i}(j,3);
            delaz = InfraConfig.array.arr{i}(j,6);
            slow = InfraConfig.array.arr{i}(j,7);
            delslow = InfraConfig.array.arr{i}(j,8);
            chan = InfraConfig.db.wfdisc{2}{i};

            try
                if (numel(find(InfraConfig.array.select{i} == j)) > 0)  % Appears to be depreciated!
                    for k = 1:numel(stns)
                        %keyboard
                        l = l + 1;
                        %fprintf(fid,Schema.arrival,stns{k},time,l,chan,azimuth,delaz,slow,delslow);
                        fprintf(fid,'%-6s %17.5f %8d %8d %8d %8d %-8s %-8s %-1s %6.3f %7.2f %7.2f %7.2f %7.2f %7.2f %7.3f %10.1f %7.2f %7.2f %-1s %-2s %10.2f %-1s %-15s %8d %-17s\n',...
                            stns{k},time,1,-1,-1,-1,chan,'-','-',-1.0,azimuth,delaz,slow,delslow,-1.0,...
                            -1.0,-1.0,-1.0,-999.0,'-','-',-1.0,'-','InfraMonitor3',-1,date);
                    end
                end
            catch
                for k = 1:numel(stns)
                    l = l + 1;
                    %fprintf(fid,Schema.arrival,stns{k},time,l,chan,azimuth,delaz,slow,delslow);

                    %^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

                    fprintf(fid,'%-6s %17.5f %8d %8d %8d %8d %-8s %-8s %-1s %6.3f %7.2f %7.2f %7.2f %7.2f %7.2f %7.3f %10.1f %7.2f %7.2f %-1s %-2s %10.2f %-1s %-15s %8d %-17s\n',...
                        stns{k},time,1,-1,-1,-1,chan,'-','-',-1.0,azimuth,delaz,slow,delslow,-1.0,...
                        -1.0,-1.0,-1.0,-999.0,'-','-',-1.0,'-','InfraMonitor3',-1,date);

                    %^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                end
            end

        end

    end

    fclose(fid);

else

    stns = InfraConfig.trace.name(InfraConfig.array.trc{i});

    for j = 1:size(InfraConfig.array.arr{i},1)

        if (InfraConfig.array.arr{i}(j,5) == 1)
            continue
        end

        time = iepoch(InfraConfig.array.arr{i}(j,1));
        azimuth = InfraConfig.array.arr{i}(j,3);
        delaz = InfraConfig.array.arr{i}(j,6);
        slow = InfraConfig.array.arr{i}(j,7);
        delslow = InfraConfig.array.arr{i}(j,8);
        chan = InfraConfig.db.wfdisc{2}{i};

        try
            if (numel(find(InfraConfig.array.select{i} == j)) > 0)
                for k = 1:numel(stns)
                    l = l + 1;
                    %fprintf(fid,Schema.arrival,stns{k},time,l,chan,azimuth,delaz,slow,delslow);
                    fprintf(fid,'%-6s %17.5f %8d %8d %8d %8d %-8s %-8s %-1s %6.3f %7.2f %7.2f %7.2f %7.2f %7.2f %7.3f %10.1f %7.2f %7.2f %-1s %-2s %10.2f %-1s %-15s %8d %-17s\n',...
                        stns{k},time,1,-1,-1,-1,chan,'-','-',-1.0,azimuth,delaz,slow,delslow,-1.0,...
                        -1.0,-1.0,-1.0,-999.0,'-','-',-1.0,'-','InfraMonitor3',-1,date);
                end
            end
        catch
            for k = 1:numel(stns)
                l = l + 1;
                %fprintf(fid,Schema.arrival,stns{k},time,l,chan,azimuth,delaz,slow,delslow);
                fprintf(fid,'%-6s %17.5f %8d %8d %8d %8d %-8s %-8s %-1s %6.3f %7.2f %7.2f %7.2f %7.2f %7.2f %7.3f %10.1f %7.2f %7.2f %-1s %-2s %10.2f %-1s %-15s %8d %-17s\n',...
                    stns{k},time,1,-1,-1,-1,chan,'-','-',-1.0,azimuth,delaz,slow,delslow,-1.0,...
                    -1.0,-1.0,-1.0,-999.0,'-','-',-1.0,'-','InfraMonitor3',-1,date);
            end
        end

    end

    fclose(fid);

end
