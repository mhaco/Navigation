% In the name of GOD 
% 990921_M.h.Aghanoori_982015004

clc,clear,close all
load check
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
%% normal test
t(1)=0;d_t=0.01;time=1000;
Ve(1)=5;Vn(1)=5;Vu(1)=5;
d_Ve(1)=0.1;d_Vn(1)=0.1;d_Vu(1)=0.1;
r(1)=3;p(1)=2;y(1)=1;
d_r(1)=0.3;d_p(1)=0.2;d_A(1)=0.1;
lat(1)=31;lon(1)=52;h(1)=82;
d_lat(1)=0.001;d_lon(1)=0.01;d_h(1)=0.00001;
N(1)=0;E(1)=0;U(1)=h(1);
% fb=[0;0;0];
% 1st step of Mechanization RAW measurment DATA
Fb=zeros(3,time*100);
beta_Fx=0.1;beta_Fy=0.1;beta_Fz=0.1;
var_Fx=0;var_Fy=0;var_Fz=0;
d_Fx(1)=0;d_Fy(1)=0;d_Fz(1)=0;
% fb(3,:)=9.7758966251266699032385076861829;
% W=[0;0;0];
W=ones(3,time*100)*1e-100;
beta_Wx=0.02;beta_Wy=0.03;beta_Wz=0.001;
var_Wx=0;var_Wy=0;var_Wz=0;
d_Wx(1)=0.1;d_Wy(1)=0.01;d_Wz(1)=0.001;
White= randi([-1 1]);
%%
% initial position in x y and z

%roation matrix body to inertial
%% 3rd step of Mechnization calculate & update of Rotation matrix
Vl(:,1)=[Ve(1);Vn(1);Vu(1)];
R_b_l1=[cosd(y)*cosd(r)-sind(y)*sind(p)*sind(r),...
    -sind(y)*cosd(p),cosd(y)*sind(r)+sind(y)*sind(p)*cos(r);...
    sind(y)*cosd(r)+cosd(y)*sind(p)*sind(r),...
    cosd(y)*cosd(p),sind(y)*sind(r)-cosd(y)*sind(p)*cosd(r);...
    -cosd(p)*sind(r),sind(p),cosd(p)*cosd(r)];
%  Quaternion
q4 = 0.5*sqrt(1+R_b_l1(1,1)+R_b_l1(2,2)+R_b_l1(3,3));
q1 = 0.25*(R_b_l1(3,2)-R_b_l1(2,3))/q4;
q2 = 0.25*(R_b_l1(1,3)-R_b_l1(3,1))/q4;
q3 = 0.25*(R_b_l1(2,1)-R_b_l1(1,2))/q4;
q(:,1)=[q1;q2;q3;q4];
%%
% initialize x hat
% Note carefull the order the states appear in, this can be arbitrary, but
% we must stick to it along the entire code

%x_hat=[d_lat d_lon d_h, d_Ve d_Vn d_Vu, d_p d_r d_A, d_Wx d_Wy d_Wz, d_Fx d_Fy d_Fz]
x_hat = [ 0       0     0    0     0    0    0   0   0    0      0   0     0     0   0]';

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

P = diag([30 30 30 3 3 3 .1 .1 .1  .1 .1 .1  .1 .1 .1]);
Pdot=P*0;
%% normal test
t(1)=0;dt=0.01;time=1000;
Ve(1)=5;Vn(1)=5;Vu(1)=5;
d_Ve(1)=0.1;d_Vn(1)=0.1;d_Vu(1)=0.1;
r(1)=3;p(1)=2;y(1)=1;
d_r(1)=0.3;d_p(1)=0.2;d_A(1)=0.1;
lat(1)=31;lon(1)=52;h(1)=82;
d_lat(1)=0.001;d_lon(1)=0.0001;d_h(1)=0.00001;
N(1)=0;E(1)=0;U(1)=h(1);
% Fb=[0;0;0];

%% 1st step of Mechanization RAW measurment DATA
Fb=zeros(3,time*100);
beta_Fx=0.1;beta_Fy=0.1;beta_Fz=0.1;
var_Fx=0;var_Fy=0;var_Fz=0;
d_Fx(1)=0;d_Fy(1)=0;d_Fz(1)=0;
% fb(3,:)=9.7758966251266699032385076861829;
% W=[0;0;0];
W=ones(3,time*100)*1e-100;
beta_Wx=0.02;beta_Wy=0.03;beta_Wz=0.001;
var_Wx=0;var_Wy=0;var_Wz=0;
d_Wx(1)=0.1;d_Wy(1)=0.01;d_Wz(1)=0.001;
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
for k=1:length(Fb(1,:))
  %1st Step of INS/GPS Integration
    Fx(k)=Fb(1,k);
    Fy(k)=Fb(2,k);
    Fz(k)=Fb(3,k);

    d_Fx(k+1)=d_Fx(k)+0.5*(-beta_Fx*d_Fx(k)+(sqrt(2*beta_Fx*var_Fx^2))*White)*dt;
    d_Fy(k+1)=d_Fy(k)+0.5*(-beta_Fy*d_Fy(k)+(sqrt(2*beta_Fy*var_Fy^2))*White)*dt;
    d_Fz(k+1)=d_Fz(k)+0.5*(-beta_Fz*d_Fz(k)+(sqrt(2*beta_Fz*var_Fz^2))*White)*dt;
%
    Wx(k)=W(1,k);
    Wy(k)=W(2,k);
    Wz(k)=W(3,k);
  
    d_Wx(k+1)=d_Wx(k)+0.5*(-beta_Wx*d_Wx(k)+(sqrt(2*beta_Wx*var_Wx^2))*White)*dt;
    d_Wy(k+1)=d_Wy(k)+0.5*(-beta_Wy*d_Wy(k)+(sqrt(2*beta_Wy*var_Wy^2))*White)*dt;
    d_Wz(k+1)=d_Wz(k)+0.5*(-beta_Wz*d_Wz(k)+(sqrt(2*beta_Wz*var_Wz^2))*White)*dt;
%%
    R_N = a/((1-e^2*sind(lat(k))^2)^(1/2));
    R_M = (a*(1-e^2))/((1-e^2*sind(lat(k))^2)^(3/2));
%%
    Sb=[0 Wz(k) -Wy(k) Wx(k);
    -Wz(k) 0 Wx(k) Wy(k)
    Wy(k) -Wx(k) 0 Wz(k)
    -Wx(k) -Wy(k) -Wz(k) 0]*dt;

    teta=sqrt((Wx(k)*dt)^2+(Wy(k)*dt)^2+(Wz(k)*dt)^2);

    q(:,k+1)=q(:,k)+(0.5*(2*(cos(teta/2)-1)*eye(4)+(2/teta)*sin(teta/2)*Sb))*q(:,k);
    q1=q(1,k+1);
    q2=q(2,k+1);
    q3=q(3,k+1);
    q4=q(4,k+1);

    R_b_l=[q1^2-q2^2-q3^2+q4^2 2*(q1*q2-q3*q4) 2*(q1*q3+q2*q4)
        2*(q1*q2+q3*q4) -q1^2+q2^2-q3^2+q4^2 2*(q2*q3-q1*q4)
        2*(q1*q3-q2*q4) 2*(q2*q3+q1*q4) -q1^2-q2^2+q3^2+q4^2];
%% 2nd Step of INS/GPS Integration
    %% 
    d_p(k+1)=d_p(k)+0.5*(((d_Vn(k))/(R_M+h(k)))+R_b_l(1,1)*(d_Wx(k)+d_Wx(k+1))...
    +R_b_l(1,2)*(d_Wy(k)+d_Wy(k+1)) +R_b_l(1,3)*(d_Wz(k)+d_Wz(k+1)))*dt;
    
    p(k+1)=round(asind(R_b_l(3,2)),1)-d_p(k+1);

    d_r(k+1)=d_r(k)+0.5*((-(d_Ve(k))/(R_N+h(k)))+R_b_l(2,1)*(d_Wx(k)+d_Wx(k+1))...
    +R_b_l(2,2)*(d_Wy(k)+d_Wy(k+1))+R_b_l(2,3)*(d_Wz(k)+d_Wz(k+1)))*dt;
    
    r(k+1)=round(-atand(R_b_l(3,1)/R_b_l(3,3)),1)-d_r(k+1);

    d_A(k+1)=d_A(k)+0.5*(((-tand(lat(k))*(d_Ve(k)))/(R_N+h(k)))+...
    R_b_l(3,1)*(d_Wx(k)+d_Wx(k+1))+R_b_l(3,2)*(d_Wy(k)+d_Wy(k+1))+...
    R_b_l(3,3)*(d_Wz(k)+d_Wz(k+1)))*dt;
    y(k+1)=round(-atand(R_b_l(1,2)/R_b_l(2,2)),1)-d_A(k+1);
%%
    omelel=[0 (-Ve(k)*tand(lat(k)))/(R_N+h(k)) Ve(k)/(R_N+h(k))
        (Ve(k)*tand(lat(k)))/(R_N+h(k)) 0 Vn(k)/(R_M+h(k))
        -Ve(k)/(R_N+h(k)) -Vn(k)/(R_M+h(k)) 0];
    omeeil=[0 -we*sind(lat(k)) we*cosd(lat(k))
        we*sind(lat(k)) 0 0
        -we*cosd(lat(k)) 0 0];
    
    g=a1*(1+a2*sind(lat(k))^2+a3*sind(lat(k))^4)+((a4+a5*sind(lat(k))^2)*h(k))+a6*h(k)^2;
    gl=[0;0;-g];

    d_Vbl=R_b_l*Fb(:,k)*dt;

    d_Vl(:,k+1)=d_Vbl-(2*omeeil+omelel)*Vl(:,k)*dt+gl*dt;
    Vl(:,k+1)=Vl(:,k)+0.5*(d_Vl(:,k)+d_Vl(:,k+1));
    %%
    Fe(k)=R_b_l(1,1)*Fx(k)+R_b_l(1,2)*Fy(k)+ R_b_l(1,3)*Fz(k);
    Fn(k)=R_b_l(2,1)*Fx(k)+R_b_l(2,2)*Fy(k)+ R_b_l(2,3)*Fz(k);
    Fu(k)=R_b_l(3,1)*Fx(k)+R_b_l(3,2)*Fy(k)+ R_b_l(3,3)*Fz(k);
%%
    d_Ve(k+1)=d_Ve(k)+0.5*(Fu(k)*(d_r(k)+d_r(k+1))-Fn(k)*(d_A(k)+d_A(k+1)))+...
    R_b_l(1,1)*d_Fx(k)+R_b_l(1,2)*d_Fy(k)+R_b_l(1,3)*d_Fz(k)*dt;
    Ve(k+1)=Vl(1,k+1)-d_Ve(k+1);

    d_Vn(k+1)=d_Vn(k)+0.5*(-Fu(k)*(d_p(k)+d_p(k+1))+Fe(k)*(d_A(k)+d_A(k+1))+...
    R_b_l(2,1)*d_Fx(k)+R_b_l(2,2)*d_Fy(k)+R_b_l(2,3)*d_Fz(k))*dt;
    Vn(k+1)=Vl(2,k+1)-d_Vn(k+1);

    d_Vu(k+1)=d_Vu(k)+0.5*(Fn(k)*(d_p(k)+d_p(k+1))-Fe(k)*(d_r(k)+d_r(k+1))+...
    R_b_l(3,1)*d_Fx(k)+R_b_l(3,2)*d_Fy(k)+R_b_l(3,3)*d_Fz(k))*dt;
    Vu(k+1)=Vl(3,k+1)-d_Vu(k+1);

%%
    d_lat(k+1)=d_lat(k)+0.5*((d_Vn(k)+Vn(k+1))/(R_M+h(k)))*dt;
    lat(k+1)=(lat(k)+0.5*((Vn(k)+Vn(k+1))/(R_N+h(k)))*dt)-d_lat(k+1);

    d_lon(k+1)=d_lon(k)+0.5*((d_Ve(k)+d_Ve(k+1))/((R_N+h(k))*cos(d_lat(k))))*dt;
    lon(k+1)=(lon(k)+0.5*((Ve(k)+Ve(k+1))/((R_N+h(k))*cos(lat(k))))*dt)-d_lon(k+1);

    d_h(k+1)=d_h(k)+0.5*(d_Vu(k)+d_Vu(k+1))*dt;
    h(k+1)=(h(k)+0.5*(Vu(k)+Vu(k+1))*dt)-d_h(k+1);



%%
    E(k+1)=E(k)+0.5*(Ve(k)+Ve(k+1))*dt;
    N(k+1)=N(k)+0.5*(Vn(k)+Vn(k+1))*dt;
    U(k+1)=U(k)+0.5*(Vu(k)+Vu(k+1))*dt;
    t(k+1)=t(k)+dt;
    if h(k+1)<=0
        break
    end
%%
    time= (k-1)*dt;
    
    %  Streaming sensor measurements and adjust for bias
    % these come from the file that is loaded in the begining
    %%%% These are still measurements but now bias has been removed
%     x_hat_old=x_hat;
    x_hat=[d_lat,d_lon,d_h, d_Ve,d_Vn,d_Vu, d_p,d_r,d_A, d_Wx,d_Wy,d_Wz, d_Fx,d_Fy,d_Fz]';


    
    d_r_l=[d_lat;d_lon;d_h];
    d_V_l=[d_Ve;d_Vn;d_Vu];
    E_l=[d_p;d_r;d_A];
    d_W_l=[d_Wx;d_Wy;d_Wz];
    d_F_l=[d_Fx;d_Fy;d_Fz];
    
    x_hat=[d_r_l;d_V_l;E_l;d_W_l;d_F_l];
    
    G=[];
    W=[];
    x_hat=[];
%     Fr = [0             (1/R_M+h(k))    0
%         (1/(R_N+h(k))*cosd(lat))    0       0
%            0                  0       1];
% %     d_lat(k) = eye(3)*d_lat(i-1) + (1/R_M+h)*delta_t*d_ve;
%     d_lon(k)
    
%%    
    % Raw sensor measurments for plotting
    p_raw = A(1,k)*pi/180;
    q_raw = A(2,k)*pi/180;
    r_raw = A(3,k)*pi/180;
    fx_raw = A(4,k);
    fy_raw = A(5,k);
    fz_raw = A(6,k);
    
    %% Implement your code here: 
    
    %% Prediction step
    %First write out all the dots, e.g. pxdot, pydot, q1dot etc
     
    %Now integrate Euler Integration for Process Updates and Covariance Updates
    % Euler works fine

    %Remember again the state vector [ px py pz vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
F=zer
    phi_matrix = (eye(15)+F*dt);
    Pdot=phi_matrix*P+P*phi_matrix'+Q;
    P1=Pdot*dt;
    P=P+P1;
    
    %% Correction step
    % Get your measurements, 3 positions and 3 velocities from GPS
    z =[ A(7,k) A(8,k) A(9,k) A(10,k) A(11,k) A(12,k)]'; % x y z vx vy vz
       
    % Assemble H
    %rank(obsv(phi_matrix,H))
    H = [eye(6) zeros(6,9)];
    %Compute Kalman gain
    S=H*P*H'+R;
    K=P*H'*inv(S);
    % Perform xhat correction    xhat = xhat + K*(z - H*xhat);

    x_hat=x_hat+K*(z-H*x_hat);
    
    P=(eye(15,15)-K*H)*P;
    
    % We are predicting state for next time instant. But in data we are
    % plotting at current time instant. Hence the previous state is stored
    % (which would be current) now
%     quatprev=[x_hat_old(7) x_hat_old(8) x_hat_old(9) x_hat_old(10)]';
%     quatprev=quatprev/norm(quatprev);
%      
%     
    %% Now let us do some book-keeping 
    % Get some Euler angles
    [phi(k),theta(k),psi(k)]=quat2euler(quatprev');
    phi(k)=phi(k)*180/pi;
    theta(k)=theta(k)*180/pi;
    psi(k)=psi(k)*180/pi;
    
    quat1 = A(13:16,k);
    [phi_raw(k),theta_raw(k),psi_raw(k)]=quat2euler(quat1');
    phi_raw(k)=phi_raw(k)*180/pi;
    theta_raw(k)=theta_raw(k)*180/pi;
    psi_raw(k)=psi_raw(k)*180/pi;
    xhatR(k,:)= [x_hat_old(1:6)' quatprev(1) quatprev(2) quatprev(3) quatprev(4) x_hat_old(11:16)'];
    P_R(k,:) = diag(P);
    z_R(k,:) = z;
    OMEGA_raw(k,:)=[p_raw,q_raw,r_raw]';
    OMEGA(k,:)=[p,q,r]';
    FX(k,:)=[fx_raw,fy_raw,fz_raw]';
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