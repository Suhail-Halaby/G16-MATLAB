function X_new = DT_EOM_fcn2(current_vel,current_pos,current_u,T_table,V_range,Q_table,u_range,dt,Const)

X_current = [current_vel;current_pos];
%% System Parameters

% Constants
g = Const(1);
mu = Const(2); % mu ranges from 0.11 to 0.17

% Motor choice
m_prop = Const(4);
m_motor = Const(5);
m_esc = Const(6);
m_des = Const(7);
m_cw = Const(8);

% Total cart mass (cart mass + arm + propellers + motors + ESC)
m_c = 0.55 + m_des + m_prop + m_motor + m_esc;

% Inertial Mass (cart+counterwieght)
m_i = m_c + m_cw;

% Rail Constraints
x_bottom = Const(11);
x_top = Const(12);

% Length dimensions
l_T = Const(13);
l_g = Const(14);
l_c = Const(15);
h = Const(16);

%% Key Functions

Q = interp1(u_range,Q_table,current_u,"nearest",'extrap');
T = interp1(V_range,T_table,current_pos,"nearest",'extrap');

% Friction Force Function  (N)
F = @(Q,T) abs(2*Q*l_g - g*m_c*l_c - 2*T*l_T )*(mu/h) ;
% Non-Fricative Forces (N)
N = @(Q,T) 2.*Q - 2.*T - (m_c-m_cw)*g ;
% Static Friction Force (N)
ST = Const(3);

%% Equations of Motion

% X_current(1) ---> Velocity at n
% X_current(2) ---> Position at n

% X_new(1) ---> Velocity at n+1
% X_new(2) ---> Position at n+1

X_new = zeros(2,1);

% static tolerance

tol = 0.001;
edge_tol = 0.001;

% acceleration at either bound must only be in a specific direction
    % if velocity is greater than 0+tol. , friction acts downwards
    % if velocity is less than 0-tol. friction acts upwards
    % if velocity is within the 'static' range:
        % if non-fricative forces are greater than static force, accel nonzero
        % if non-fricative forces are less than static, accel is zero




if  (X_current(1) >= -tol) && (X_current(1) <= tol)

     if abs(N(Q,T)) < ST
        % If non-fricative force can't overcome static friction, no accel
        X_new(1) = 0;
     elseif N(Q,T) > ST
        % Starting upwards, downwards friction
        X_new(1) = X_current(1) + dt*( N(Q,T) - ST )/m_i;
     elseif N(Q,T) < -ST
        % Starting upwards, downwards friction
        X_new(1) = X_current(1) + dt*( N(Q,T) + ST )/m_i;
     end

elseif X_current(1) > tol
    % Moving Upwards, Downwards Friction
    X_new(1) = X_current(1) + dt*( N(Q,T) - F(Q,T) )/m_i;

elseif X_current(1) < -tol
    % Moving Downwards, Upwards Friction
    X_new(1) = X_current(1) + dt*( N(Q,T) + F(Q,T) )/m_i;
end

% Position State Updates
X_new(2) = X_current(2) + dt * 0.5 * (X_current(1) + X_new(1));

%% Edge Impacts

% if located at the top, velocity must be zero or negative
% if locatede at the bottom, velocity must be zero or positive


if X_current(2) >= x_top-edge_tol
    if X_new(1) >= 0
        X_new(1) = 0;
    end
    if X_new(2) > x_top
        X_new(2) = x_top;
    end
elseif X_current(2) <= x_bottom+edge_tol
    if X_new(1) <= 0
        X_new(1) = 0;
    end
    if X_new(2) < x_bottom
        X_new(2) = x_bottom;
    end
end


if X_new(2) > x_top
        X_new(2) = x_top;
elseif X_new(2) < x_bottom
        X_new(2) = x_bottom;
end





