% In the name of GOD 
% 990921_M.h.Aghanoori_982015004

clc,clear,close all
% % define Geodetic variables example Isfahan city
lon=51.680374;lat=32.661343;h=84;
% % Semimajor axis (equatorial radius)
a=6378137;
% % Semiminor axis b = a(1-f)~6356752.3142 m
b=6356752.3142;
% Eccentricity
% e=sqrt((a^2-b^2)/a^2);
e=0.08181919;
% normal radius
RN=a/(1-e^2*sind(lat)^2)^(1/2);
%  Conversion From Geodetic to Rectangular Coordinates
XeYeZe=[(RN+h)*cosd(lat)*cosd(lon)
        (RN+h)*cosd(lat)*sind(lon)
        (RN*(1-e^2)+h)*sind(lat)];
% Display results    
disp('Xe of Isfahan City')
disp(vpa(XeYeZe(1)))
disp('Ye of Isfahan City')
disp(vpa(XeYeZe(2)))
disp('Ze of Isfahan City')
disp(vpa(XeYeZe(3)))