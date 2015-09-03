function varargout = readenv(filename)
% readenv
%
% e.g., [c,v,u,z] = readenv('DLIAR2005012512.env');
% Then save as a mat file: e.g., save('DLIAR.mat','c','v','u','z')
%
%  Description: A script to read an G2S enviromental record
%     output by the G2S extractgcp command line appliance
%     into memory.
%
%  Author: Douglas P. Drob
%          NRL Code 7643
%
%  Date: Updated - January 19, 2007
%
% ======================================================================

% [file_name, mach_path]=uigetfile('*.env', 'File Selector');
% filename=strcat(mach_path,file_name);

fid = fopen(filename,'r','ieee-le');

nr = fread(fid,1,'integer*4');      %[pnt] # of range point in profiles
nz = fread(fid,1,'integer*4');      %[pnt] # altitude points in profile

lat = fread(fid,nr,'real*8');       %[deg] latitude profile grid
lon = fread(fid,nr,'real*8');       %[deg] longitude profile grid
baz = fread(fid,nr,'real*8');       %[deg] longitude profile grid
rng = fread(fid,nr,'real*8');       %[m] range from source of grid
alt = fread(fid,nr,'real*8');       %[m] height of air/ground/sea
ai = fread(fid,nz,'real*8');        %[km] Altitudes for 2d grid

ti = fread(fid,[nz,nr],'real*8');   %[k] Temperature
di = fread(fid,[nz,nr],'real*8');   %[g/cm3] density
pr = fread(fid,[nz,nr],'real*8');   %[hPa] pressure
ui = fread(fid,[nz,nr],'real*8');   %[m/s] Along track wind
vi = fread(fid,[nz,nr],'real*8');   %[m/s] Cross track wind
wi = fread(fid,[nz,nr],'real*8');   %[m/s] vertical wind

tj = fread(fid,[nz,nr],'real*8');   %[k] Temperature (parallel path)
dj = fread(fid,[nz,nr],'real*8');   %[g/cm3] density (parallel path)
pj = fread(fid,[nz,nr],'real*8');   %[hPa] pressure (parallel path)
uj = fread(fid,[nz,nr],'real*8');   %[m/s] Along track wind (parallel path)
vj = fread(fid,[nz,nr],'real*8');   %[m/s] Cross track wind (parallel path)
wj = fread(fid,[nz,nr],'real*8');   %[m/s] vertical wind (parallel path)

% ===================================================
% calculate some quantities from the specifications
% ===================================================

si = sqrt( 1.4 * 0.1 * pr ./ di);
sj = sqrt( 1.4 * 0.1 * pj ./ dj);

hgt = ai(:);

% figure;
% plot(ui(:,1),hgt,'r',ui(:,nr),hgt,'g')
% xlabel('Velocity (m/s)');
% ylabel('Altitude (km)');
% title('Along Track Wind');
% axis([-70 70 0 130]);
%
% figure;
% plot(vi(:,1),hgt,vi(:,nr),hgt)
% xlabel('Velocity (m/s)');
% ylabel('Altitude (km)');
% title('Cross Track Wind');
% axis([-50 50 0 130]);

% % Temporary addition (04/03/10):
% I = mean(regexp(filename,'-'));

% figure;
% plot(si(:,1),hgt,':',si(:,nr),hgt,':')
% hold on
% plot(si(:,1)+ui(:,1),hgt,si(:,nr)+ui(:,nr),hgt)
% axis([250 400 0 130]);
% hold off;
% xlabel('Velocity (m/s)');
% ylabel('Altitude (km)');
% title(['Sound Velocity: ' filename(I)]);

if (nargout == 0)
    % Temporary modification to take the central profile in v_eff only:
    figure;
    hold on
    plot(si(:,ceil(nr/2))+ui(:,ceil(nr/2)),hgt)
    axis([250 400 0 130]);
    hold off;
    xlabel('Velocity (m/s)');
    ylabel('Altitude (km)');
    title(['Sound Velocity: ' filename(I)]);
else
%     varargout{1} = si(:,ceil(nr/2));   % Isotropic sound speed (m/s)
%     varargout{2} = ui(:,ceil(nr/2));   % Along-track wind (m/s)
%     varargout{3} = vi(:,ceil(nr/2));   % Cross-track wind (m/s)
    varargout{1} = si(:,1);   % Isotropic sound speed (m/s)
    varargout{2} = ui(:,1);   % Along-track wind (m/s)
    varargout{3} = vi(:,1);   % Cross-track wind (m/s)
    varargout{4} = hgt;                % Altitude (km)
end

fclose(fid);
