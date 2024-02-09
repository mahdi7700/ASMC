%**************************************************************************
%       User documentation of any code that uses this code must cite the
%       Authors in the following articles:
%
%       (1) M. H. Korayem, M. Safarbal and N. Yousefi Lademakhi, 
%       “Adaptive Robust Control with Slipping Parameters Estimation 
%       Based on Intelligent Learning for Wheeled Mobile Robot ,” 
%       ISA transactions, 2024.(Article in Press)
%
%      
%       Permission to modify this file (ONLY) and to distribute modified
%       code is granted, provided the above notices are retained, and a
%       notice that the code was modified is included with the above
%       statements.
%
%       NOTE: This MATLAB code is provided free of charge.
%       You may make copies of the codes, but this NOTICE must appear in
%       all copies.
%
%       ** >> Contact Information: mahdi_safarbali@yahoo.com << **
%**************************************************************************

clc;
clear;
close all;

Tf = 60;    % Final Time
dt = 0.01;  % time step
t = 0:dt:Tf;
N = numel(t);
%% WMR's body parameters (See Table 2 of the article)
b = 0.27/2;
d = 0.05;
r = 0.05;
mb = 2.3;
mw = 0.1;
Ib = 0.062;
Iwz = 0.00028;
Iwy = 0.00028;
%% modified dugoff parameters (Equations 9-14)
Fz = (mb+4*mw)*9.81/4;
Mu = 0.2;
Ca = 8000;
Cb = 2600;
%% backstepping control parametrs (Kinematic control gains Equation (22))
k1 = 15;
k2 = 48;
k3 = 20;
%% ASMC parameters (Dynamic control gains Equations 25 & 33)
beta = 32;  % 𝜓 in article
eps = 0.1;
rho1 = 1;
rho2 = 1.13;

%% Desired Path (Defining desired states)
% In this section, Users can choose four different desired trajectories for
% the WMR to track them. By adjusting the control gains (k1, k2, k3, beta,
% rho1, rho2) and initial conditions, the robot can tracks the trajectory
% with minimum tracking errors
path = 2;  % Users must choose a desired path from 1 to 4
% Eight Shape
if path == 1
    freq = 2*pi/Tf;
    Xr =  sin(2*freq*t);
    Yr =  sin(freq*t);
    dXr = 2*freq*cos(2*freq*t);
    dYr = freq*cos(freq*t);
    ddXr = -4*freq^2*sin(2*freq*t);
    ddYr = -freq^2*sin(freq*t);
    dddXr = -8*freq^3*cos(2*freq*t);
    dddYr = -freq^3*cos(freq*t);
    THETAr = atan2(dYr,dXr);                          % Equation (19)
    Vr = sqrt(dXr.^2 + dYr.^2);                       % Equation (19)
    Wr = (dXr.*ddYr - dYr.*ddXr)./(dXr.^2 + dYr.^2);  % Equation (19)
    Vr_dot = (ddXr+ddYr)./sqrt(dXr.^2 + dYr.^2);      
    Wr_dot = ((ddXr.*ddYr+dddYr.*dXr-ddYr.*ddXr-dYr.*dddXr).*(dXr.^2 + dYr.^2) - 2*(ddXr+ddYr).*(dXr.*ddYr - dYr.*ddXr))./(dXr.^2 + dYr.^2).^2;
    
% Circle Shape
elseif path == 2
    freq = 2*pi/Tf;
    Xr =  sin(freq*t);
    Yr =  cos(freq*t);
    dXr = freq*cos(freq*t);
    dYr = -freq*sin(freq*t);
    ddXr = -freq^2*sin(freq*t);
    ddYr = -freq^2*cos(freq*t);
    dddXr = -freq^3*cos(freq*t);
    dddYr = freq^3*sin(freq*t);
    THETAr = atan2(dYr,dXr);                          % Equation (19)
    Vr = sqrt(dXr.^2 + dYr.^2);                       % Equation (19)
    Wr = (dXr.*ddYr - dYr.*ddXr)./(dXr.^2 + dYr.^2);  % Equation (19)
    Vr_dot = (ddXr+ddYr)./sqrt(dXr.^2 + dYr.^2);
    Wr_dot = ((ddXr.*ddYr+dddYr.*dXr-ddYr.*ddXr-dYr.*dddXr).*(dXr.^2 + dYr.^2) - 2*(ddXr+ddYr).*(dXr.*ddYr - dYr.*ddXr))./(dXr.^2 + dYr.^2).^2;
    
% Flower Shape
elseif path == 3
    Xr=0.2*(20+cos(36*t/50)).*cos(6*t/50);
    Yr=0.2*(20+cos(36*t/50)).*sin(6*t/50);
    dXr=-0.144*sin((18/25)*t).*cos((3/25)*t)-(0.024*(20+cos((18/25)*t))).*sin((3/25)*t);
    dYr=-0.144*sin((18/25)*t).*sin((3/25)*t)+(0.024*(20+cos((18/25)*t))).*cos((3/25)*t);
    ddXr =-0.1036800000*cos((18/25)*t).*cos((3/25)*t)+0.3456000000e-1*sin((18/25)*t).*sin((3/25)*t)-(0.2880000000e-2*(20+cos((18/25)*t))).*cos((3/25)*t);
    ddYr =-0.1036800000*cos((18/25)*t).*sin((3/25)*t)-0.3456000000e-1*sin((18/25)*t).*cos((3/25)*t)-(0.2880000000e-2*(20+cos((18/25)*t))).*sin((3/25)*t);
    dddXr = 0.1036800000*(18/25*sin((18/25)*t).*cos((3/25)*t)+3/25*sin((3/25)*t).*cos((18/25)*t))+0.3456000000e-1*(18/25*cos((18/25)*t).*sin((3/25)*t)+3/25*cos((3/25)*t).*sin((18/25)*t))+0.2880000000e-2*(20*3/25*sin((3/25)*t)+18/25*sin((18/25)*t).*cos((3/25)*t)+3/25*sin((3/25)*t).*cos((18/25)*t));
    dddYr = 0.1036800000*(18/25*sin((18/25)*t).*sin((3/25)*t)-3/25*cos((18/25)*t).*cos((3/25)*t))-0.3456000000e-1*(18/25*cos((18/25)*t).*cos((3/25)*t)-3/25*sin((18/25)*t).*sin((3/25)*t))-0.2880000000e-2*(20*3/25*cos((3/25)*t)-18/25*sin((18/25)*t).*sin((3/25)*t)+3/25*cos((18/25)*t).*cos((3/25)*t));
    THETAr = atan2(dYr,dXr);                          % Equation (19)
    Vr = sqrt(dXr.^2 + dYr.^2);                       % Equation (19)
    Wr = (dXr.*ddYr - dYr.*ddXr)./(dXr.^2 + dYr.^2);  % Equation (19)
    Vr_dot = (ddXr+ddYr)./sqrt(dXr.^2 + dYr.^2);
    Wr_dot = ((ddXr.*ddYr+dddYr.*dXr-ddYr.*ddXr-dYr.*dddXr).*(dXr.^2 + dYr.^2) - 2*(ddXr+ddYr).*(dXr.*ddYr - dYr.*ddXr))./(dXr.^2 + dYr.^2).^2;
    
% Sin Shape
elseif path == 4
    freq = 2*pi/6;
    Xr =  t;
    Yr =  5*sin(freq*t);
    dXr = ones(1,numel(t));
    dYr = 5*freq*cos(freq*t);
    ddXr = zeros(1,numel(t));
    ddYr = -5*freq^2*sin(freq*t);
    dddXr = zeros(1,numel(t));
    dddYr = -5*freq^3*cos(freq*t);
    THETAr = atan2(dYr,dXr);                          % Equation (19)
    Vr = sqrt(dXr.^2 + dYr.^2);                       % Equation (19)
    Wr = (dXr.*ddYr - dYr.*ddXr)./(dXr.^2 + dYr.^2);  % Equation (19)
    Vr_dot = (ddXr+ddYr)./sqrt(dXr.^2 + dYr.^2);
    Wr_dot = ((ddXr.*ddYr+dddYr.*dXr-ddYr.*ddXr-dYr.*dddXr).*(dXr.^2 + dYr.^2) - 2*(ddXr+ddYr).*(dXr.*ddYr - dYr.*ddXr))./(dXr.^2 + dYr.^2).^2;
end

%% Initial Condition
% In this section the initial position of the WMR and its velocities are
% defined. Users should choose the initial conditions properly according to
% the desired trajectory.
X0 = 0;
Y0 = 0.8;
theta0 = pi/6;
phi_r0 = 0;
phi_l0 = 0;
vf0 = 0;   % Initial Linear Velocity
wf0 = 0;   % Initial Angular Velocity
SS10 = 2;  % Initial Sliding Surface
SS20 = -2; % Initial Sliding Surface

X(1) = X0;
Y(1) = Y0;
theta(1) = theta0;
phi_r(1) = phi_r0;
phi_l(1) = phi_l0;
vf(1) = vf0;
wf(1) = wf0;
SS(1,1) = SS10;
SS(2,1) = SS20;
SS_all(:,1) = [SS10;SS20];

q(:,1) = [X(1) Y(1) theta(1) phi_r(1) phi_l(1)]';

dtheta(1) = wf(1);
dphi_r(1) = (vf(1)+b*wf(1))/(r);
dphi_l(1) = (vf(1)-b*wf(1))/(r);

alpha1(1) = 3;
alpha2(1) = 3;
Alpha_all(:,:,1) = [alpha1 0;0 alpha2];

% Initial Tracking Errors (Equation (20))
e1(1) = (Xr(1) - X(1)) * cos(theta(1)) + (Yr(1) - Y(1)) * sin(theta(1));
e2(1) = -( Xr(1) - X(1) ) * sin(theta(1)) + ( Yr(1) - Y(1) ) * cos(theta(1));
e3(1) = wrapToPi(THETAr(1) - theta(1));

e1dot(1) = e2(1)*wf(1) - vf(1) + Vr(1)*cos(e3(1));
e2dot(1) = -e1(1)*wf(1) + Vr(1)*sin(e3(1));
e3dot(1) = Wr(1) - wf(1);

landa(1) = 0;    % Initial Lateral Slip
landa_dot(1) = 0;
gamma_r(1) = 0;  % Initial longitudinal slip angular velocity of right wheel (𝜂𝑟(𝑡))
gamma_l(1) = 0;  % Initial longitudinal slip angular velocity of left wheel (𝜂𝑙(𝑡))
gamma_r_dot(1) = 0;
gamma_l_dot(1) = 0;

zeta_v(1) = 0;
zeta_w(1) = 0;
zeta_v_dot(1) = 0;
zeta_w_dot(1) = 0;
zeta(:,1) = [0;0];
zeta_dot(:,1) = [0;0];

E_all = zeros(2,2,N-1);

z(:,1) = [0;0];
F(:,1) = [0;0;0;0;0];
T(:,1) = [0;0];

%%

for i = 1:N-1
    S = [cos(theta(i)) sin(theta(i)) 0 1/r 1/r;
        0 0 1 b/r -b/r]'; % 𝐒(𝐪) 
    S_all(:,:,i) = S;
    
    S_dot = [-sin(theta(i))*dtheta(i) cos(theta(i))*dtheta(i) 0 0 0;
        0 0 0 0 0]';
    S_dot_all(:,:,i) = S_dot;
    
    M = [mb 0 0 0 0;
        0 mb 0 0 0;
        0 0 Ib+2*Iwz 0 0;
        0 0 0 mw*r^2+Iwy 0;
        0 0 0 0 mw*r^2+Iwy];    % inertia matrix
    
    B = [0 0 0 1 0;0 0 0 0 1]'; % input transformation matrix
    C = zeros(5,5);             % centrifugal and coriolis matrix
    E = S'*M*S;
    E_all(:,:,i) = E;
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    hq = [-landa(i)*sin(theta(i)) landa(i)*cos(theta(i)) 0 gamma_r(i) gamma_l(i)]';
    hq_dot = [-landa_dot(i)*sin(theta(i))-landa(i)*cos(theta(i)) landa_dot(i)*cos(theta(i))-landa(i)*sin(theta(i)) 0 gamma_r_dot(i) gamma_l_dot(i)]';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% kinematic control law (backstepping) Equation (22)
    v_c(i) = Vr(i) * cos(e3(i)) + k1 * e1(i);
    w_c(i) = Wr(i) + k2 * Vr(i) * e2(i) + k3 * Vr(i) * sin(e3(i));
    
    v_c_dot(i) = Vr_dot(i)*cos(e3(i))-e3dot(i)*Vr(i)*sin(e3(i))+k1*e1dot(i);
    w_c_dot(i) = Wr_dot(i)+k2*Vr_dot(i)*e2(i)+k2*Vr(i)*e2dot(i)+k3*Vr_dot(i)*sin(e3(i))+k3*Vr(i)*e3dot(i)*cos(e3(i));
    Vc_dot = [v_c_dot(i);w_c_dot(i)];
    %  auxiliary velocity tracking error (Equation (23))
    e_c1(i) = v_c(i) - vf(i);
    e_c2(i) = w_c(i) - wf(i);
    Ec = [e_c1(i);e_c2(i)];
    
    %% Uncertainty & Disturbances
    % Users can change the domain and the entrance time of Uncertainties
    % and disturbances of the system
    if t(i) > 16 && t(i) < 25
        dis = [1*sin(t(i));1*sin(t(i))];
        Unc1 = 1*(-E\(S'*M*S_dot+S'*C*S));
        Unc2 = 1*(E\(S'*B));
    else
        dis = zeros(2,1);
        Unc1 = zeros(2,2);
        Unc2 = zeros(2,2);
    end
    
    %% Dynamic control (ASMC)
    % Saturation Fun
    if abs(SS(1)) > eps
        satS1(i) = sign(SS(1));
    else
        satS1(i) = SS(1)/eps;
    end
    
    if abs(SS(2)) > eps
        satS2(i) = sign(SS(2));
    else
        satS2(i) = SS(2)/eps;
    end
    
    Sat_S = [satS1(i);satS2(i)];
    % tan hyperbulic Fun
    hyp_S = [tanh(SS(1));tanh(SS(2))];
    % Sign Fun
    sgn_S = [sign(SS(1));sign(SS(2))];
    
%     Adaptive law - Equation (33)
    alpha1_dot = rho1*SS(1)*Sat_S(1)/(rho2+SS(1)^2);
    alpha2_dot = rho1*SS(2)*Sat_S(2)/(rho2+SS(2)^2);
    alpha1 = alpha1+dt*alpha1_dot;  % 𝛬1
    alpha2 = alpha2+dt*alpha2_dot;  % 𝛬2
    Alpha = [alpha1 0;0 alpha2];
    Alpha_all(:,:,i+1) = Alpha;
    
    % ASMC Control Law - Equation (32)
    T = (E\S'*B)\(Vc_dot + E\S'*M*S_dot*(z) + E\S'*C*S*(z) - E\S'*F + beta*Ec + Alpha*Sat_S); 
    
    % Dynamic Model of the WMR in presence of Wheel Slip and Uncertainties - Equation (28)
    z_dot = (S'*M*S)\(-S'*M*S_dot*(z-zeta)+S'*B*T-S'*M*hq_dot-S'*C*(S*(z-zeta)+hq)-dis+S'*F)+zeta_dot+Unc1*z+Unc2*T;
    
    T_all(:,i+1) = T;
 
    z = z+dt*z_dot;
    
    vf(i+1) = z(1);
    wf(i+1) = z(2);
    
    dtheta(i+1) = wf(i+1);
    dphi_r(i+1) = (vf(i+1)+b*wf(i+1))/(r);
    dphi_l(i+1) = (vf(i+1)-b*wf(i+1))/(r);
    
    % Sliding Surface - Equation (26)
    SS_dot = (Vc_dot - z_dot) + beta*Ec;
    SS = SS+dt*SS_dot;
    SS_all(:,i+1) = SS;
    
    % Kinematic Model of the WMR - Equation (4)
    qdot = S*(z - zeta)+hq;
    q = q+dt*qdot;
    
    X(i+1) = q(1);
    Y(i+1) = q(2);
    theta(i+1) = q(3);
    phi_r(i+1) = q(4);
    phi_l(i+1) = q(5);
   
    %% Tracking Errors (Equation (20))
    e1(i+1) = (Xr(i+1) - X(i+1)) * cos(theta(i+1)) + (Yr(i+1) - Y(i+1)) * sin(theta(i+1));
    e2(i+1) = -( Xr(i+1) - X(i+1) ) * sin(theta(i+1)) + ( Yr(i+1) - Y(i+1) ) * cos(theta(i+1));
    e3(i+1) = wrapToPi(THETAr(i+1) - theta(i+1));
    % Equation (21)
    e1dot(i+1) = e2(i+1)*wf(i+1) - vf(i+1) + Vr(i+1)*cos(e3(i+1));
    e2dot(i+1) = -e1(i+1)*wf(i+1) + Vr(i+1)*sin(e3(i+1));
    e3dot(i+1) = Wr(i+1) - wf(i+1);
    
    %% Slipping Parameters
    % Users can change the slipping parameters functions but the amount and
    % domain of them should be chosen reasonable and low
    landa(i+1) = 0.05*exp(-0.5*t(i+1))*abs(sin(t(i+1))); % lateral slip velocity
    landa_dot(i+1) = (landa(i+1)-landa(i))/dt;
    gamma_r(i+1) = 2*exp(-0.5*t(i))*sin(t(i));  %longitudinal slip angular velocity of right wheel
    gamma_l(i+1) = 2*exp(-0.5*t(i))*sin(t(i));  %longitudinal slip angular velocity of left wheel
    gamma_r_dot(i+1) = (gamma_r(i+1)-gamma_r(i))/dt;
    gamma_l_dot(i+1) = (gamma_l(i+1)-gamma_l(i))/dt;
    
    zeta_v(i+1) = r*(gamma_r(i+1)+gamma_l(i+1))/2;
    zeta_w(i+1) = r*(gamma_r(i+1)-gamma_l(i+1))/(2*b);
    zeta_v_dot(i+1) = r*(gamma_r_dot(i+1)+gamma_l_dot(i+1))/2;
    zeta_w_dot(i+1) = r*(gamma_r_dot(i+1)-gamma_l_dot(i+1))/(2*b);
    zeta = [zeta_v(i+1);zeta_w(i+1)];
    zeta_dot = [zeta_v_dot(i+1);zeta_w_dot(i+1)];
    
    %% Modified Dugoff (Equations 9-16)
    drho_r(i+1) = r*(dphi_r(i+1) - gamma_r(i+1));
    drho_l(i+1) = r*(dphi_l(i+1) - gamma_l(i+1));
    sr(i+1) = r*gamma_r(i+1)/max(abs(r*dphi_r(i+1)),abs(drho_r(i+1))); % Equation (15)
    sl(i+1) = r*gamma_l(i+1)/max(abs(r*dphi_l(i+1)),abs(drho_l(i+1))); % Equation (15)
    if r*dphi_r(i+1) == drho_r(i+1)
        sr(i+1) = 0;
    end
    
    if  r*dphi_l(i+1) == drho_l(i+1)
        sl(i+1) = 0;
    end
    
    alpha(i+1) = atan2(landa(i+1),vf(i+1));
    
    if landa(i+1) == vf(i+1)
        alpha(i+1) = 0;
    end
    
    if vf(i+1) == 0
        alpha(i+1) =0;
    end
    
    zr(i+1) = Mu*Fz*(1-sr(i+1))/(2*sqrt((Ca*sr(i+1))^2 + (Cb*tan(alpha(i+1)))^2));
    zl(i+1) = Mu*Fz*(1-sl(i+1))/(2*sqrt((Ca*sl(i+1))^2 + (Cb*tan(alpha(i+1)))^2));
    
    if sr(i+1) == 0 && tan(alpha(i+1)) == 0
        zr(i+1) = 0;
    end
    
    if sl(i+1) == 0 && tan(alpha(i+1)) == 0
        zl(i+1) = 0;
    end
    
    if zr(i+1) < 1
        fz_r(i+1) = zr(i+1)*(2-zr(i+1));
    else
        fz_r(i+1) = 1;
    end
    
    if zl(i+1) < 1
        fz_l(i+1) = zl(i+1)*(2-zl(i+1));
    else
        fz_l(i+1) = 1;
    end
    
    g_long_r(i+1) = (1.15-0.75*Mu)*sr(i+1)^2 - (1.63-0.75*Mu)*sr(i+1) + 1.5;
    g_long_l(i+1) = (1.15-0.75*Mu)*sl(i+1)^2 - (1.63-0.75*Mu)*sl(i+1) + 1.5;
    
    g_lat_r(i+1) = (Mu - 1.6)*tan(alpha(i+1)) + 1.5;
    g_lat_l(i+1) = g_lat_r(i+1);
    
    f_long_r(i+1) = Ca*sr(i+1)/(1-sr(i+1))*fz_r(i+1)*g_long_r(i+1);
    f_long_l(i+1) = Ca*sl(i+1)/(1-sl(i+1))*fz_l(i+1)*g_long_l(i+1);
    f_lat_r(i+1) = Cb*tan(alpha(i+1))/(1-sr(i+1))*fz_r(i+1)*g_lat_r(i+1);
    f_lat_l(i+1) = Cb*tan(alpha(i+1))/(1-sl(i+1))*fz_l(i+1)*g_lat_l(i+1);
    
    if sr(i+1) == 1 && sl(i+1) == 1
        f_long_r(i+1) = 0;
        f_long_l(i+1) = 0;
        f_lat_r(i+1) = 0;
        f_lat_l(i+1) = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F = [0;0;0;-r*f_long_r(i+1);-r*f_long_l(i+1)];
    F_all(:,i+1) = F;
end

% To better comprehend the performance of the proposed controller and
% compare the effect of control gains, two index error are introduced. IAE
% is Integral Absolute Error and RMSE is Root Mean Square Errors. Users
% should reach the minimum value of these two index errors by changing the
% control gains, but the value of control effort must be checked.
IAE1 = trapz(abs(e1))
IAE2 = trapz(abs(e2))
IAE3 = trapz(abs(e3))

RMSE1 = sqrt(sum(e1.^2)/numel(e1))
RMSE2 = sqrt(sum(e2.^2)/numel(e2))
RMSE3 = sqrt(sum(e3.^2)/numel(e3))

% Defined in Section 4-2 (𝑒𝑝(𝑡)) 
path_error = sqrt(Xr.^2+Yr.^2) - sqrt(X.^2+Y.^2);

figure
plot(Xr,Yr,'LineWidth',2,'Color','k')
hold on
plot(X,Y,'--','LineWidth',2,'Color','g')
xlabel('X (m)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
ylabel('Y (m)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
hold on
plot(X(1),Y(1),'o','LineWidth',3,'Color','r')
hold on
plot(X(end),Y(end),'s','LineWidth',3,'Color','b')
grid on
legend('Reference Trajectory','Actual Trajectory','start point','end point','FontSize',10)
title('Trajectory Tracking Result','FontSize',10);
axis equal

figure (5)
plot(t,e1,'-','LineWidth',1.5,'Color','g');
xlabel('Time (s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
grid on
hold on

plot(t,e2,'-','LineWidth',1.5,'Color','b');
xlabel('Time (s)','color','b','FontSize',12,'FontWeight','bold','FontName','Cambria');
ylabel('errors (m)','color','b','FontSize',12,'FontWeight','bold','FontName','Cambria');
grid on
legend('X-error','Y-error','FontSize',12)

figure
plot(t,e3,'-','LineWidth',1.5,'Color','r');
xlabel('Time (s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
ylabel('e_\theta (rad)','color','b','FontSize',12,'FontWeight','bold','FontName','Cambria');
grid on

figure
plot(t,path_error,'-','LineWidth',1.5,'Color',[0.1 0.1 0.8]);
xlabel('Time (s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
ylabel('e_p (m)','color','b','FontSize',12,'FontWeight','bold','FontName','Cambria');
grid on

figure
subplot(2,1,1)
plot(t,vf,'LineWidth',1.5,'Color','k');
xlabel('Time (s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
ylabel('Linear Velocity (m/s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
grid on
subplot(2,1,2)
plot(t,wf,'LineWidth',1.5,'Color','k');
xlabel('Time (s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
ylabel('Angular Velocity (rad/s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
grid on

figure
plot(t,T_all(1,:),'LineWidth',2,'Color','b')
hold on
plot(t,T_all(2,:),'--','LineWidth',2,'Color','r')
xlabel('Time (s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
ylabel('Torque (N.m)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
grid on

figure
plot(t,SS_all(1,:),'LineWidth',2,'Color','c')
hold on
plot(t,SS_all(2,:),'--','LineWidth',2,'Color','m')
xlabel('Time (s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
ylabel('Sliding Surface ','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
legend('S_1','S_2','FontSize',10)
grid on

alpha1_all = Alpha_all(1,1:2:2*N);
alpha2_all = Alpha_all(2,2:2:2*N);
figure
plot(t,alpha1_all,'LineWidth',2,'Color',[1 0.37 0.76])
hold on
plot(t,alpha2_all,'--','LineWidth',2,'Color',[0.37 0 0.6])
xlabel('Time (s)','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
ylabel('Adaptive gains ','color','b','FontSize',10,'FontWeight','bold','FontName','Cambria');
legend('\alpha_1','\alpha_2','FontSize',10)
grid on