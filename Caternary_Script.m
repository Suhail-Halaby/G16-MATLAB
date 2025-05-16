%% DuoCopter EoM
clear 
clc
close all
%% Governing Equation Caternary Curve:

x_mount = 0;
y_mount = 0.7;
x_cart = 0.2 ;
y_cart = 0.33 ;


F = @(x,a,c,d) c*cosh((x-a)/c)+d;

% Cable Length
s = 1;

x0 = [mean([x_mount, x_cart]) ; abs(y_mount - y_cart)/2 ; 0];
% First equation: pass through one known point
F1 = @(p) y_mount - F(x_mount, p(1), p(2),p(3));
% Second Equation: pass through second anchor
F2 = @(p) y_cart - F(x_cart, p(1), p(2),p(3));
% Third equation: arc length equals total cable length
F3 = @(p) catenary_arc_length(p(1), p(2), x_mount, x_cart) - s;

% Combine into a system
Fun = @(p) [F1(p); F2(p) ; F3(p)];

% Solve
G = fsolve(Fun, x0);
a_sol = G(1);
c_sol = G(2);
d_sol = G(3);


x_plot = linspace(x_mount,x_cart,100);
y_plot = F(x_plot,a_sol,c_sol,d_sol);
hold on
plot(x_plot,y_plot,'k');
plot(x_cart,y_cart,'.r', MarkerSize=15)
plot(x_mount,y_mount,'.r', MarkerSize=15)

function L = catenary_arc_length(a, c, x1, x2)
    L = c * (sinh((x2 - a)/c) - sinh((x1 - a)/c));
end


rho = 0.190;

T_H = rho*9.81*c_sol
T_V = T_H * sinh((x_cart - a_sol)/c_sol)
T_total = T_H * cosh((x_cart - a_sol)/c_sol)