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

%% Edge Protection - Terminal Velocity Saturation

E = @(x,h,k) (1+h).*(1 ./ (1+exp(-k*x))) - h ;
h_funct = @(V) 1-2*V;

Max_Terminal_V_factor = 0.25;
Sharpness_Const = 10;

V_range2 = 0:0.01:1.44;
E_table = E(V_range2,h_funct(Max_Terminal_V_factor),Sharpness_Const); 





%% Plotting
subplot(2,2,1)
plot(u_range,Q_table)
title('Thrust Force (N) vs. Control Input (% Thrtl)')
subplot(2,2,2)
plot(V_range,T_table)
title('Vertical Position (m) vs. Vertical Cable Tension')
subplot(2,2,3)
plot(V_range2,E_table)
title('Velocity Saturation Limit (LWR) vs. Position')
subplot(2,2,4)
plot(V_range2,flip(E_table))
title('Velocity Saturation Limit (UPPR) vs. Position')

%% Tables

writematrix("Parameters_and_Tables.csv") 
