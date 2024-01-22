% clc
% clear 
% 
% load("AltCostFcn_trial5.mat")
grid on
%% z-position

plot(t,-pz,LineWidth=1)
%ylim([0.5,1.5])
xlabel("t (s)")
ylabel("p_z (m)")

%% x-position

plot(t,px,LineWidth=1)
%ylim([0.5,1.5])
xlabel("t (s)")
ylabel("p_x (m)")

%% vx-position

plot(t,vx,LineWidth=1)
%ylim([0.5,1.5])
xlabel("t (s)")
ylabel("v_x (m/s)")

%% theta
figure
plot(t,theta*180/pi,LineWidth=1)
%ylim([89.9,90.1])
xlabel("t (s)")
ylabel("\theta (deg)")

%% inputs
figure
plot(t,-msp1,LineWidth=1)
hold on
plot(t,msp2,LineWidth=1)
%ylim([89.9,90.1])
xlabel("t (s)")
ylabel("\omega (rad/s)")
legend("Motor 1", "Motor 2")

figure
plot(t,delta1*180/pi,LineWidth=1)
hold on
plot(t,delta2*180/pi,LineWidth=1)
%ylim([-30,30])
xlabel("t (s)")
ylabel("\delta (deg)")
legend("Elevon 1", "Elevon 2")
