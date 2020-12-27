% In the name of GOD 
% 990921_M.h.Aghanoori_982015004

clc,clear,close all
load check
load INS_LLF
%% Earth components
a=6378137;e=0.08181919;we=7.292115*10^-5;% e=0;we=0;
a1=9.7803267714;a2=0.0052790414;a3=0.0000232718;a4=-0.0000030876910891;a5=0.0000000043977311;a6=0.0000000000007211;
%% normal test
k=1;dk=1;time=1000;
%% 1st step of Mechanization RAW measurment DATA
    lat_raw_GPS = 31+randi([-1 1]);
    lon_raw_GPS = 52+randi([-1 1]);
    h_raw_GPS   = 82+randi([-1 1]);
    Ve_raw_GPS  = 5+randi([-1 1]);
    Vn_raw_GPS  = 5+randi([-1 1]);
    Vu_raw_GPS  = 5+randi([-1 1]);

    lat_raw_INS=lat+randi([-5 5]);
    lon_raw_INS=lon-randi([-5 5]);
    h_raw_INS=h+randi([-5 5]);
    Fb=zeros(3,k*100);
    
%%
% initialize x hat
%x_hat=[d_lat d_lon d_h, d_Ve d_Vn d_Vu, d_p d_r d_A, d_Wx d_Wy d_Wz, d_Fx d_Fy d_Fz]
x_hat = [ 0.1  0.1  0.1  0.2  0.2   0.2    0   0   0    0      0   0     0     0   0]';

% noise params process noise 
% [d_lat d_lon d_h, d_Ve d_Vn d_Vu, d_p d_r d_A, d_wx d_wy d_wz,  d_fx    d_fy  d_fz]
Q = diag([.1 .1 .1   .1    .1  .1   .8  .8  .8 0.0001 .0001 .0001 .0001 .0001 .0001]);

% noise params, measurement noise
% measurements are GPS position and Accelerometer
R = diag([9 9 9 3 3 3]);


%% set sample time
dt = .02;
tf=372;
%% 1st step of Mechanization RAW measurment DATA
beta_Fx=0.1;beta_Fy=0.1;beta_Fz=0.1;
var_Fx=0.1;var_Fy=0.02;var_Fz=0.01;
beta_Wx=0.02;beta_Wy=0.03;beta_Wz=0.001;
var_Wx=0.45;var_Wy=0.02;var_Wz=0.5;
White= randi([-1 1]);
%% UPDATING

for k=1:tf
%     time= (k-1)*dt;

    d_r_l=[d_lat(1,k)  ; d_lon(1,k) ; d_h(1,k) ];
    d_V_l=[d_Ve(1,k)  ; d_Vn(1,k)  ; d_Vu(1,k)];
     E_l =[d_p(1,k)   ; d_r(1,k)   ; d_A(1,k) ];
    d_W_l=[d_Wx(1,k)  ; d_Wy(1,k)  ; d_Wz(1,k)];
    d_F_l=[d_Fx(1,k)  ; d_Fy(1,k)  ; d_Fz(1,k)];
    % 1. Project the state ahead
    x_hat_prev=[d_r_l;d_V_l;E_l;d_W_l;d_F_l];
    x_update = x_hat_prev ;
%     x_hat_predict(1) = 0;
 %% Initiallizaion 
 %Now integrate Euler Integration for Process Updates and Covariance Updates Euler works fine
        R_N = a/((1-e^2*sind(lat(1,k))^2)^(1/2));
        R_M = (a*(1-e^2))/((1-e^2*sind(lat(1,k))^2)^(3/2));       
        F_r=[0                           1/(R_M+h(k))         0
            1/((R_N+h(k))*cosd(lat(k)))         0             0
            0                                    0             1];
        
        F_v=[0                                  U(1,k)       -N(k)
            -U(k)                                   0         E(k)
             N(k)                                 -E(k)         0];
        
        F_e=[0                        inv(R_M+h(k))       0
            inv(R_N+h(k))                   0             0
            -tand(lat(k))/(R_M+h(k))        0             0];
        
        F_w=[beta_Wx                0              0
              0                  beta_Wy           0
              0                      0          beta_Wz];
          
        F_f=[beta_Fx                 0             0
              0                  beta_Fy           0
              0                      0          beta_Fz];
    %Remember again the state vector [ px py pz vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
    F=[zeros(3)     F_r     zeros(3)  zeros(3)    zeros(3)
       zeros(3)   zeros(3)    F_v     zeros(3)    zeros(3)
       zeros(3)     F_e     zeros(3)   R_b_l      zeros(3)
       zeros(3)   zeros(3)  zeros(3)    F_w       zeros(3)
       zeros(3)   zeros(3)  zeros(3)  zeros(3)      F_f   ];
    phi_matrix = (eye(15)+F*dk);


    
    G=[0.1 0.1 0.1 0.2 0.2 0.2 0.005 0.005 0.005 0.001 0.001 0.001 0.3 0.3 0.3]';
    Wn=0.5*randi([-5 5]);
      
    %% Prediction step
   
    x_hat_predict=phi_matrix*x_update + G*Wn; % State Vector prediction
    x_buffer(:,k) = x_hat_predict;             % just for graphical result
    % Initialize P, the covariance matrix
    P = diag([3 3 3 3 3 3 .1 .1 .1 .1 .1 .1 .1 .1 .1]);
    
    Pk_predict = phi_matrix.*P.*phi_matrix' + G'*phi_matrix*G; % Covariance prediction 
%     Pdot=P*0;
%     Pdot=phi_matrix*P+P*phi_matrix'+Q;
%     P1=Pdot*dk;
%     P=P+P1;
    %% Correction step
    % Get your measurements, 3 positions and 3 velocities from GPS
    Z = [lat(1,k)-lat_raw_GPS
         lon(1,k)-lon_raw_GPS
         h(1,k)-h_raw_GPS
         Ve(1,k)-Ve_raw_GPS
         Vn(1,k)-Vn_raw_GPS
         Vu(1,k)-Vu_raw_GPS];
    Z_buffer(:,k) = Z;
%     Z =[ p q r fx fy fz]'; % x y z vx vy vz   
    % Assemble H
    H = [eye(6) zeros(6,9)];
%   Measurement Update (a.k.a. Correction or Innovation stage) %%%%%%%
    S= H*Pk_predict*H'+R;  
        %  1. compute the Kalman gain
    K=(Pk_predict*H')/(S);
    
        %  2. update the estimate with measurement Zk
    x_update =x_hat_predict + K*(Z -H*x_hat_predict);
    x_update_buffer(:,k) = x_update;
        %  3. Update the error Covariance
    Pk_update = Pk_predict - K * H * Pk_predict;   % Pk = (I - K*H)Pk_prev
    
    
%     if x_hat_predict < 0.0001
%         break
%         
%     end
    
    
end
%%
 
%%
t = 1:(k);

p_nRew=x_update(1,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot the resultant graph %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graphcial comparison b/w Kalman filter and Noisy Measurement (Position)
% figure(20);
% plot(t, x_hat_prev(1,1), 'b', 'LineWidth', 5);hold on;
% scatter(t, x_update, '+', 'k', 'LineWidth', 1.5);hold on;
plot(t, x_buffer(1,:), 'r', 'LineWidth', 1);hold on;
plot(t, x_buffer(2,:), 'g', 'LineWidth', 1);hold on;
plot(t, x_buffer(3,:), 'm', 'LineWidth', 1);hold on;
plot(t, x_buffer(4,:), 'c', 'LineWidth', 1);hold on;
plot(t, x_buffer(5,:), 'b', 'LineWidth', 1);hold on;
plot(t, x_buffer(6,:), 'k', 'LineWidth', 1);hold on;

title('Position of one-dimentional train tracking example');
xlabel('Time in seconds');
ylabel('Position of the Train (meter in 1D)');
% legend('x-hat-prev','x-update','X1','X2','X3', 'X4','X5','X6');
legend('X1','X2','X3', 'X4','X5','X6');
% legend('x-hat','x-hat_prev','Kalman Filter', 'Moving Average');
hold off;

%Graphical result (Velocity)
figure(2)

plot(t, Z_buffer(1,:),'--k', 'LineWidth', 1.5);hold on; 
plot(t, Z_buffer(2,:), '--b', 'LineWidth', 1);hold on;
plot(t, Z_buffer(3,:), '--c', 'LineWidth', 1);hold on;
plot(t, Z_buffer(4,:), '--y', 'LineWidth', 1);hold on;
plot(t, Z_buffer(5,:), '--g', 'LineWidth', 1);hold on;
plot(t, Z_buffer(6,:), '--m', 'LineWidth', 1);hold on;
title('Velocity of one-dimentional train tracking example');
xlabel('Time in seconds');
% ylabel('Velocity of the Train (m/s)');
legend('Z1-Measurements','Z2-Measurements','Z3-Measurements', 'Z4-Measurements','Z5-Measurements','Z6-Measurements');
hold off;


figure(8)
plot(t,lat,t,lon,t,h,t,lat_raw_INS,'b:',t,lon_raw_INS,'g:',t,h_raw_INS,'r:','LineWidth',2)
title('latitude, longtitude, altitude')
legend('lat','lon','h','lat-raw', 'lon-raw', 'h-raw')
grid on

%%
% % figure(21)
% % subplot(3,4,1)
% % plot(t,lat,'g','linewidth',1)
% % xlabel('Time(s)');ylabel('Latitude');
% % grid
% % %%
% % subplot(3,4,5)
% % plot(t,lon,'g','linewidth',1)
% % xlabel('Time(s)');ylabel('longtitude');
% % grid
% % %%
% % subplot(3,4,9)
% % plot(t,h,'g','linewidth',1)
% % xlabel('Time(s)');ylabel('h');
% % grid
% % %%
% % subplot(3,4,2)
% % plot(t,E,'r','linewidth',1)
% % xlabel('Time(s)');ylabel('E');
% % grid
% % %%
% % subplot(3,4,6)
% % plot(t,N,'r','linewidth',1)
% % xlabel('Time(s)');ylabel('N');
% % grid
% % %%
% % subplot(3,4,10)
% % plot(t,U,'r','linewidth',1)
% % xlabel('Time(s)');ylabel('U');
% % grid
% % %%
% % subplot(3,4,3)
% % plot(t,Ve,'c','linewidth',1)
% % xlabel('Time(s)');ylabel('Ve');
% % grid
% % %%
% % subplot(3,4,7)
% % plot(t,Vn,'c','linewidth',1)
% % xlabel('Time(s)');ylabel('Vn');
% % grid
% % %%
% % subplot(3,4,11)
% % plot(t,Vu,'c','linewidth',1)
% % xlabel('Time(s)');ylabel('Vu');
% % grid
% % %%
% % subplot(3,4,4)
% % plot(t,r,'linewidth',1)
% % xlabel('Time(s)');ylabel('roll');
% % grid
% % %%
% % subplot(3,4,8)
% % plot(t,p,'linewidth',1)
% % xlabel('Time(s)');ylabel('pich');
% % grid
% % %%
% % subplot(3,4,12)
% % plot(t,y,'linewidth',1)
% % xlabel('Time(s)');ylabel('yaw');
% % grid
%%
% % figure(1)
% % plot(t,P_R(:,1:3),'LineWidth',2)
% % title('Covariance of Position')
% % legend('px','py','pz')
% % grid on
% % 
% % figure(2)
% % plot(t,P_R(:,4:6),'LineWidth',2)
% % legend('pxdot','pydot','pzdot')
% % title('Covariance of Velocities')
% % grid on
% % 
% % figure(3)
% % plot(t,P_R(:,7:10),'LineWidth',2)
% % title('Covariance of Quaternions')
% % legend('q0cov','q1cov','q2cov','q3cov')
% % grid on

% figure(4)
% hold on
% plot(t,x_hat_predict(12:15,t),'r',t,A(7:9,t),'b','LineWidth',2)
% %plot(t,z_R(:,1),'r')
% title('Position')
% legend('px','py','pz','pxgps','pygps','pzgps')
% grid on
% % 
% % figure(5)
% % plot(t,x_hat(4:6,t),t,A(10:12,t),':','LineWidth',2)
% % % plot(t,xhatR(:,4:6),t,A(10:12,:),':','LineWidth',2)
% % title('vel x y z')
% % legend('vx','vy','vz','vxgps','vygps','vzgps')
% % grid on
% % 
% % figure(7)
% % plot(t,OMEGA(:,1:3),'LineWidth',2)
% % title('OMEGA without bias')
% % legend('p','q','r')
% % grid on
% % 
% % figure(10)
% % plot(t,OMEGA_raw(:,1),t,OMEGA_raw(:,2),t,OMEGA_raw(:,3),'LineWidth',2)
% % title('OMEGA raw without Bias')
% % legend('p','q','r')
% % grid on
% % 
% % figure(11)
% % plot(t,FX(:,1),t,FX(:,2),t,FX(:,3),'LineWidth',2)
% % title('accelerometer')
% % legend('ax','ay','az')
% % grid on
