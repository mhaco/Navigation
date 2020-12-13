% In the name of GOD 
% 990921_M.h.Aghanoori_982015004

clc,clear,close all
% load INS_LLF
%% Earth components
a=6378137;
e=0.08181919;
we=7.292115*10^-5;
% e=0;we=0;
a1=9.7803267714;
a2=0.0052790414;
a3=0.0000232718;
a4=-0.0000030876910891;
a5=0.0000000043977311;
a6=0.0000000000007211;
%%
% initial position in x y and z
x=[0 0 0];
% bias values, these are accelerometer and gyroscope biases
bp= 0;%.54*pi/180;
bq=-12*pi/180;
br=-.1*pi/180;
bfx = 0;
bfy = 0;
bfz = 0;


% IMU location specifier
r_imu=[-.5/12 -3/12 1/12]'*0; % Currently set to 0. Offset effects are not currently considered
r_GPS=[1.5, 0 ,0 ]; % This is the location of the GPS wrt CG, this is very important
%rotation matrix ------------------------------------------------------
phi= x(1);
theta= x(2);
psi = x(3);
%initialize velocity
vx = 0;
vy = 0;
vz = 0;
%set sample time
dt = .02;
tf=size(A,2);
% tf2=size(A,1);
%roation matrix body to inertial
%% 3rd step of Mechnization calculate & update of Rotation matrix
Vl(:,1)=[Ve(1);Vn(1);Vu(1)];
R_b_l1=[cosd(y)*cosd(r)-sind(y)*sind(p)*sind(r),...
    -sind(y)*cosd(p),cosd(y)*sind(r)+sind(y)*sind(p)*cos(r);...
    sind(y)*cosd(r)+cosd(y)*sind(p)*sind(r),...
    cosd(y)*cosd(p),sind(y)*sind(r)-cosd(y)*sind(p)*cosd(r);...
    -cosd(p)*sind(r),sind(p),cosd(p)*cosd(r)];

q4=0.5*sqrt(1+R_b_l1(1,1)+R_b_l1(2,2)+R_b_l1(3,3));
q1=0.25*(R_b_l1(3,2)-R_b_l1(2,3))/q4;
q2=0.25*(R_b_l1(1,3)-R_b_l1(3,1))/q4;
q3=0.25*(R_b_l1(2,1)-R_b_l1(1,2))/q4;
q(:,1)=[q1;q2;q3;q4];
%%
% initialize x hat
% Note carefull the order the states appear in, this can be arbitrary, but
% we must stick to it along the entire code
%x_hat=[d_lat d_lon d_h, d_Ve d_Vn d_Vu, d_p d_r d_A, d_wx d_wy d_wz, d_fx d_fy d_fz]
xhat = [ 0       0     0    0     0    0    0   0   0    0      0   0     0     0   0]';

% noise params process noise 
% [d_lat d_lon d_h, d_Ve d_Vn d_Vu, d_p d_r d_A, d_wx d_wy d_wz,  d_fx    d_fy  d_fz]
Q = diag([.1 .1 .1   .1    .1  .1   .8  .8  .8 0.0001 .0001 .0001 .0001 .0001 .0001]);

% noise params, measurement noise
% measurements are GPS position and Accelerometer
R = diag([9 9 9 3 3 3]);
%R=diag([10 10 10 8 8 8]);
%R=diag([0.01 0.01 0.01 0.01 0.01 0.01]);
%R = diag([10 10 10 8 8 8]);
%R = 0.01*R;

% Initialize P, the covariance matrix
P = diag([30 30 30 3 3 3 .1 .1 .1 .1 .1 .1 .1 .1 .1 .1]);
Pdot=P*0;
%% normal test
t(1)=0;dt=0.01;time=1000;
Ve(1)=5;Vn(1)=5;Vu(1)=5;
dVe(1)=0.1;dVn(1)=0.1;dVu(1)=0.1;
r(1)=3;p(1)=2;y(1)=1;
dr(1)=0.3;dp(1)=0.2;dA(1)=0.1;
lat(1)=31;lon(1)=52;h(1)=82;
dlat(1)=0.001;dlon(1)=0.0001;dh(1)=0.00001;
N(1)=0;E(1)=0;U(1)=h(1);
% fb=[0;0;0];
% 1st step of Mechanization RAW measurment DATA
fb=zeros(3,time*100);
beta_fx=0.1;beta_fy=0.1;beta_fz=0.1;
var_fx=0;var_fy=0;var_fz=0;
dfx(1)=0;dfy(1)=0;dfz(1)=0;
% fb(3,:)=9.7758966251266699032385076861829;
% W=[0;0;0];
W=ones(3,time*100)*1e-100;
beta_wx=0.02;beta_wy=0.03;beta_wz=0.001;
var_wx=0;var_wy=0;var_wz=0;
dwx(1)=0.1;dwy(1)=0.01;dwz(1)=0.001;
White= randi([-1 1]);
%% 3rd step of Mechnization calculate & update of Rotation matrix
Vl(:,1)=[Ve(1);Vn(1);Vu(1)];
R_b_l1=[cosd(y)*cosd(r)-sind(y)*sind(p)*sind(r),...
    -sind(y)*cosd(p),cosd(y)*sind(r)+sind(y)*sind(p)*cos(r);...
    sind(y)*cosd(r)+cosd(y)*sind(p)*sind(r),...
    cosd(y)*cosd(p),sind(y)*sind(r)-cosd(y)*sind(p)*cosd(r);...
    -cosd(p)*sind(r),sind(p),cosd(p)*cosd(r)];

q4=0.5*sqrt(1+R_b_l1(1,1)+R_b_l1(2,2)+R_b_l1(3,3));
q1=0.25*(R_b_l1(3,2)-R_b_l1(2,3))/q4;
q2=0.25*(R_b_l1(1,3)-R_b_l1(3,1))/q4;
q3=0.25*(R_b_l1(2,1)-R_b_l1(1,2))/q4;
q(:,1)=[q1;q2;q3;q4];
%% UPDATING
for i=1:length(fb(1,:))
wx(i)=W(1,i);
wy(i)=W(2,i);
wz(i)=W(3,i);
fx(i)=fb(1,i);
fy(i)=fb(2,i);
fz(i)=fb(3,i);

dfx(i+1)=dfx(i)+0.5*(-beta_fx*dfx(i)+(sqrt(2*beta_fx*var_fx^2))*White)*dt;
dfy(i+1)=dfy(i)+0.5*(-beta_fy*dfy(i)+(sqrt(2*beta_fy*var_fy^2))*White)*dt;
dfz(i+1)=dfz(i)+0.5*(-beta_fz*dfz(i)+(sqrt(2*beta_fz*var_fz^2))*White)*dt;
dwx(i+1)=dwx(i)+0.5*(-beta_wx*dwx(i)+(sqrt(2*beta_wx*var_wx^2))*White)*dt;
dwy(i+1)=dwy(i)+0.5*(-beta_wy*dwy(i)+(sqrt(2*beta_wy*var_wy^2))*White)*dt;
dwz(i+1)=dwz(i)+0.5*(-beta_wz*dwz(i)+(sqrt(2*beta_wz*var_wz^2))*White)*dt;

RN=a/((1-e^2*sind(lat(i))^2)^(1/2));
RM=(a*(1-e^2))/((1-e^2*sind(lat(i))^2)^(3/2));
Sb=[0 wz(i) -wy(i) wx(i);
    -wz(i) 0 wx(i) wy(i)
    wy(i) -wx(i) 0 wz(i)
    -wx(i) -wy(i) -wz(i) 0]*dt;
teta=sqrt((wx(i)*dt)^2+(wy(i)*dt)^2+(wz(i)*dt)^2);
q(:,i+1)=q(:,i)+(0.5*(2*(cos(teta/2)-1)*eye(4)+(2/teta)*sin(teta/2)*Sb))*q(:,i);
q1=q(1,i+1);
q2=q(2,i+1);
q3=q(3,i+1);
q4=q(4,i+1);
Rbl=[q1^2-q2^2-q3^2+q4^2 2*(q1*q2-q3*q4) 2*(q1*q3+q2*q4)
     2*(q1*q2+q3*q4) -q1^2+q2^2-q3^2+q4^2 2*(q2*q3-q1*q4)
     2*(q1*q3-q2*q4) 2*(q2*q3+q1*q4) -q1^2-q2^2+q3^2+q4^2];

dp(i+1)=dp(i)+0.5*(((dVn(i))/(RM+h(i)))+Rbl(1,1)*(dwx(i)+dwx(i+1))...
    +Rbl(1,2)*(dwy(i)+dwy(i+1)) +Rbl(1,3)*(dwz(i)+dwz(i+1)))*dt;
p(i+1)=round(asind(Rbl(3,2)),1)-dp(i+1);

dr(i+1)=dr(i)+0.5*((-(dVe(i))/(RN+h(i)))+Rbl(2,1)*(dwx(i)+dwx(i+1))...
    +Rbl(2,2)*(dwy(i)+dwy(i+1))+Rbl(2,3)*(dwz(i)+dwz(i+1)))*dt;
r(i+1)=round(-atand(Rbl(3,1)/Rbl(3,3)),1)-dr(i+1);

dA(i+1)=dA(i)+0.5*(((-tand(lat(i))*(dVe(i)))/(RN+h(i)))+...
    Rbl(3,1)*(dwx(i)+dwx(i+1))+Rbl(3,2)*(dwy(i)+dwy(i+1))+...
    Rbl(3,3)*(dwz(i)+dwz(i+1)))*dt;
y(i+1)=round(-atand(Rbl(1,2)/Rbl(2,2)),1)-dA(i+1);

omelel=[0 (-Ve(i)*tand(lat(i)))/(RN+h(i)) Ve(i)/(RN+h(i))
        (Ve(i)*tand(lat(i)))/(RN+h(i)) 0 Vn(i)/(RM+h(i))
        -Ve(i)/(RN+h(i)) -Vn(i)/(RM+h(i)) 0];
omeeil=[0 -we*sind(lat(i)) we*cosd(lat(i))
        we*sind(lat(i)) 0 0
        -we*cosd(lat(i)) 0 0];
    
g=a1*(1+a2*sind(lat(i))^2+a3*sind(lat(i))^4)+((a4+a5*sind(lat(i))^2)*h(i))+a6*h(i)^2;
gl=[0;0;-g];

dvbl=Rbl*fb(:,i)*dt;

dvl(:,i+1)=dvbl-(2*omeeil+omelel)*Vl(:,i)*dt+gl*dt;
Vl(:,i+1)=Vl(:,i)+0.5*(dvl(:,i)+dvl(:,i+1));

fe(i)=Rbl(1,1)*fx(i)+Rbl(1,2)*fy(i)+...
    Rbl(1,3)*fz(i);
fn(i)=Rbl(2,1)*fx(i)+Rbl(2,2)*fy(i)+...
    Rbl(2,3)*fz(i);
fu(i)=Rbl(3,1)*fx(i)+Rbl(3,2)*fy(i)+...
    Rbl(3,3)*fz(i);

dVe(i+1)=dVe(i)+0.5*(fu(i)*(dr(i)+dr(i+1))-fn(i)*(dA(i)+dA(i+1)))+...
    Rbl(1,1)*dfx(i)+Rbl(1,2)*dfy(i)+Rbl(1,3)*dfz(i)*dt;
Ve(i+1)=Vl(1,i+1)-dVe(i+1);

dVn(i+1)=dVn(i)+0.5*(-fu(i)*(dp(i)+dp(i+1))+fe(i)*(dA(i)+dA(i+1))+...
    Rbl(2,1)*dfx(i)+Rbl(2,2)*dfy(i)+Rbl(2,3)*dfz(i))*dt;
Vn(i+1)=Vl(2,i+1)-dVn(i+1);

dVu(i+1)=dVu(i)+0.5*(fn(i)*(dp(i)+dp(i+1))-fe(i)*(dr(i)+dr(i+1))+...
    Rbl(3,1)*dfx(i)+Rbl(3,2)*dfy(i)+Rbl(3,3)*dfz(i))*dt;
Vu(i+1)=Vl(3,i+1)-dVu(i+1);

dlat(i+1)=dlat(i)+0.5*((dVn(i)+Vn(i+1))/(RN+h(i)))*dt;
lat(i+1)=(lat(i)+0.5*((Vn(i)+Vn(i+1))/(RN+h(i)))*dt)-dlat(i+1);

dlon(i+1)=dlon(i)+0.5*((dVe(i)+dVe(i+1))/((RN+h(i))*cos(dlat(i))))*dt;
lon(i+1)=(lon(i)+0.5*((Ve(i)+Ve(i+1))/((RN+h(i))*cos(lat(i))))*dt)-dlon(i+1);


dh(i+1)=dh(i)+0.5*(dVu(i)+dVu(i+1))*dt;
h(i+1)=(h(i)+0.5*(Vu(i)+Vu(i+1))*dt)-dh(i+1);

E(i+1)=E(i)+0.5*(Ve(i)+Ve(i+1))*dt;
N(i+1)=N(i)+0.5*(Vn(i)+Vn(i+1))*dt;
U(i+1)=U(i)+0.5*(Vu(i)+Vu(i+1))*dt;
t(i+1)=t(i)+dt;
if h(i+1)<=0
    break
end
end
%%
% save('INS_LLF.mat','lat','lon','h','E','N','U','Ve','Vn','Vu','r','p','y');
subplot(3,4,1)
plot(t,lat,'g','linewidth',1)
xlabel('Time(s)');ylabel('Latitude');
grid
%%
subplot(3,4,5)
plot(t,lon,'g','linewidth',1)
xlabel('Time(s)');ylabel('longtitude');
grid
%%
subplot(3,4,9)
plot(t,h,'g','linewidth',1)
xlabel('Time(s)');ylabel('h');
grid
%%
subplot(3,4,2)
plot(t,E,'r','linewidth',1)
xlabel('Time(s)');ylabel('E');
grid
%%
subplot(3,4,6)
plot(t,N,'r','linewidth',1)
xlabel('Time(s)');ylabel('N');
grid
%%
subplot(3,4,10)
plot(t,U,'r','linewidth',1)
xlabel('Time(s)');ylabel('U');
grid
%%
subplot(3,4,3)
plot(t,Ve,'c','linewidth',1)
xlabel('Time(s)');ylabel('Ve');
grid
%%
subplot(3,4,7)
plot(t,Vn,'c','linewidth',1)
xlabel('Time(s)');ylabel('Vn');
grid
%%
subplot(3,4,11)
plot(t,Vu,'c','linewidth',1)
xlabel('Time(s)');ylabel('Vu');
grid
%%
subplot(3,4,4)
plot(t,r,'linewidth',1)
xlabel('Time(s)');ylabel('roll');
grid
%%
subplot(3,4,8)
plot(t,p,'linewidth',1)
xlabel('Time(s)');ylabel('pich');
grid
%%
subplot(3,4,12)
plot(t,y,'linewidth',1)
xlabel('Time(s)');ylabel('yaw');
grid