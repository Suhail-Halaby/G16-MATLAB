%% Table Exporter
clc
clear
close all





%% Q(u) - Thrust Function
thrust_curve = readmatrix("Big_Motors.csv");

% Corresponding PWM singnal
PWM = thrust_curve(9:59,2);
% Newtons of thrust force
NTF = 9.81*thrust_curve(9:59,6)/1000;

% Power draw WATTS
POW = thrust_curve(9:59,11);

% Thrust function handle takes in % throttle, returns newtons of the thurst
% force


u_range = 0:1:100;

Q_table = interp1(PWM,NTF,1000+u_range*13);
P_table = 2 * interp1(PWM,POW,1000+u_range*13);


%% T(x) - Tension Model
V_range = linspace(0,1.25,10);
offset = 0.065;

T_table = zeros(size(V_range));
for i = 1:length(V_range)   
    T_table(i) = Caternary_Script(V_range(i),offset);
end

%% Friction

mu = 0.12;
h = 0.2;
l_t = 0.2;
l_c = 0.2;
l_g = 0.4;
m_c = 1;
g = 9.81;

F = @(Q,T,X) ((2*mu)/h)*abs(l_t*Q-m_c*g*l_g-T*l_c);

%% Plotting
subplot(1,2,1)
plot(u_range,Q_table)
title('Thrust Force (N) vs. Control Input (% Thrtl)')
subplot(1,2,2)
plot(V_range,T_table)
title('Vertical Position (m) vs. Vertical Cable Tension')


%% Tables

writematrix("Parameters_and_Tables.csv") 
