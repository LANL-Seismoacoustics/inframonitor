% TauP
% Version 2.0, 17-April-2010
%
% Files
%   AzDev              - AzDev - Computes the azimuthal deviation for a given ray
%   BouncePoints       - BouncePoints - Computes the bounce points for a given ray
%   ClosestEigenray    - ClosestEigenray - Outputs the closest eigenray to evloc from a set of
%   eigenray           - eigenray - Computes eigenrays between given event and receiver locations,
%   Generate_met       - Generate_met.m
%   GetBPCoords        - GetBPCoords - Outputs the (lat,lon) coordinates of bounce points in the
%   GetWind            - GetWind - Converts zonal and meridional wind components into wind
%   GetWindRaypath     - GetWindRaypath - Converts wind vector (with speed u0 and direction d)
%   InfraMapRaySummary - InfraMapRaySummary - Provides summary metrics for 3D ray paths from
%   km2degsc           - km2degsc - Converts horizontal distance in km to distance in degrees
%   PlotInfraMapRays   - PlotInfraMapRays - Plots 3D ray paths from InfraMap propagation run .mat
%   plotmet            - plotmet - Plots meteorological data
%   plottaup           - plottaup - Plots all/selected raypaths in 3D and source location (evloc).
%   ProcSounding       - ProcSounding - Processes atmospheric sounding data obtained from the
%   ResampleSounding   - ResampleSounding - Resamples atmospheric sounding data onto a regularly
%   RotateXY           - RotateXY - Converts coordinates from (range, transverse offset) to (ray(I).x
%   runtaup            - runtaup - Computes multiple raypaths using the Tau-P method (shooting
%   runtaup_backup     - runtaup.m
%   taup               - taup - Computes a single raypath using the Tau-P method of Drob et al. (2010)
%	taup_old		   - taup_old - Computes a single raypath using the Tau-P method of Garces et al. (1998)
%   ZMax               - ZMax - Computes the maximum elevation for a given ray
