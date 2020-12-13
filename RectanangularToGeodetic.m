% In the name of GOD 
% 990921_M.h.Aghanoori_982015004
clc,clear,close all

Xe=3332699;Ye=4216959;Ze=3422445;
% % Semimajor axis (equatorial radius)
a=6378137;
% % Semiminor axis b = a(1-f)~6356752.3142 m
b=6356752.3142;
e=0.08181919;
Pe=sqrt(Xe^2+Ye^2);
% 1st step
h0=0;
% 2nd step. Choose an arbitrary value of latitude (or GPS output)
lat0=atand(Ze/(Pe*(1-e^2)));
% 3rd step. The geodetic longitude is calculated
lon=atand(Ye/Xe);
% 4th step. Starting from i ¼ 1; iterate as follows
R_N(1)=a/((1-(e^2*sind(lat0)^2))^(1/2));
h(1)=(Pe/(cosd(lat0)))-R_N(1);
lat(1)=atand((Ze/(sqrt(Xe^2+Ye^2)))*((R_N(1)+h(1))/((R_N(1)*(1-e^2)+h(1)))));
error=abs(h(1)-h0)+abs(lat(1)-lat0);
i=2;
% Compare ui; ui1 and hi; hi1; if convergence has been achieved then stop,
% otherwise repeat step d using the new values.
while error>1e-8
R_N(i)=a/((1-(e^2*sind(lat(i-1))^2))^(1/2));
h(i)=(Pe/(cosd(lat(i-1))))-R_N(i);
lat(i)=atand((Ze/(sqrt(Xe^2+Ye^2)))*((R_N(i)+h(i))/((R_N(i)*(1-e^2)+h(i)))));
error=abs(h(i)-h(i-1))+abs(lat(1)-lat(i-1));
i=i+1;
end
r=sqrt(Xe^2+Ye^2+Ze^2);
phip=asind(Ze/r);
disp('longtitude of Isfahan city')
disp(vpa(lon))
disp('latitude of Isfahan city')
disp(vpa(lat(i-1)))
% disp('latitude of Isfahan city')
% disp(vpa(phip))
disp('h')
disp(vpa(h(i-1)))