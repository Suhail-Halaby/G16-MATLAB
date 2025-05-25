%% Table Exporter
clc
close all

%% PART 1: CONSTANTS

% note: all forces expressed in newtons, massess expressed in kilograms, lengths
% in meters.

g = 9.81;

% Uncertain friction
mu = 0.17; % Varies: 0.11 - 0.17
ST = 1.30; % Vary proportionally? Reseasrch: correlation between stat and dyn coeffs.

% Masses (kg)

m_prop = 0.006;
m_motor = 0.076;
m_esc = 0.042;
m_des = 0.094;
m_cw = 0.400;

    % Total cart mass (cart mass + arm + propellers + motors + ESC)
    m_c = 0.55 + m_des + m_prop + m_motor + m_esc;
    
    % Inertial Mass (cart+counterwieght)
    m_i = m_c + m_cw;

% Rail Constraints
x_bottom = 0;
x_top = 1.44;

% Length dimensions
l_T = 0.040;
l_g = 0.040;
l_c = 0.055;
h = 0.2104;

Const_vars = [g,mu,ST,m_prop,m_motor,m_esc,m_des,m_cw,m_c,m_i,x_bottom,x_top,l_T,l_g,l_c,h];

%% Part 2: Gains

% tuning OLD
P1 = 5;
I1 = 4;
D1 = 0.1;

P2 = 0.5;
I2 = 0.2;
D2 = 0;

clegg = 0.5;

% New gains - Cascade
P1 = 5.9335;
I1 = 4.0314;
D1 = 0.32668;

P2 = 0.17157;
I2 = 0.41039;
D2 = 0;

clegg = 0.12565;

% New gains - MHE
P1_mhe = 2;
I1_mhe = 0.2;
D1_mhe = 0.15;

P2_mhe = 0.17157;
I2_mhe = 0.1;
D2_mhe = 0.1;

clegg_mhe = 0.12565;

%% Part 3: Q(u) - Thrust Function Curve
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


%% Part 4: T(x) - Tension Model
V_range = linspace(0,1.25,10);
offset = 0.065;

T_table = zeros(size(V_range));
for i = 1:length(V_range)   
    T_table(i) = Caternary_Script(V_range(i),offset);
end


%% Part 5: Edge Protection - Terminal Velocity Saturation

E = @(x,h,k) (1+h).*(1 ./ (1+exp(-k*x))) - h ;
h_funct = @(V) 1-2*V;

Max_Terminal_V_factor = 0.25;
Sharpness_Const = 10;

V_range2 = 0:0.01:1.44;
E_table = E(V_range2,h_funct(Max_Terminal_V_factor),Sharpness_Const); 


%% Part 6: MHE Tuning

time_horizon = 6;
cost_1 = 1;
cost_2 = 1;
cost_3 = 1;
var_x = 1;
var_v = 3;



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
