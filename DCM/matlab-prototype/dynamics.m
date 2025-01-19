t = 1.30; %m
L = 1.55; %m
h = 0.30; %m
g = 9.8; %m/s2
M = 284; %kg

close all;

lon_xfr = @(Ax) M * g * (Ax) * (h / t);
lat_xfr = @(Ay) M * g * (Ay) * (h / L);
fl_n = @(Ax, Ay) (lat_xfr(-Ay) * 0.5) + (lon_xfr(-Ax) * 0.5) + (M * g / 4);
fr_n = @(Ax, Ay) (lat_xfr(-Ay) * 0.5) + (lon_xfr(Ax) * 0.5) + (M * g / 4);
rr_n = @(Ax, Ay) (lat_xfr(Ay) * 0.5) + (lon_xfr(Ax) * 0.5) + (M * g / 4);
rl_n = @(Ax, Ay) (lat_xfr(Ay) * 0.5) + (lon_xfr(-Ax) * 0.5) + (M * g / 4);

figure;
fplot(lon_xfr, [-2 2]);
% hold on;
% fplot(lat_xfr, [-2 2]);

fl_n(0.5,1) + fr_n(0.5,1) + rl_n(0.5,1) + rr_n(0.5,1) - M*g

ClA = 3.42;
CdA = 1.79;

x_rw = 0; % m
x_fw = 1.55; % m 
rho = 1.225; % kg/m^3
CoP_x = 0.35; % percent rear
CoP_z = -0.6; % m

F_lift = @(v) 0.5 * ClA * rho * v^2; % N
F_drag = @(v) 0.5 * CdA * rho * v^2; % N

M_drag = @(v) F_drag(v) * CoP_z;

F_rlift = @(v) CoP_x * F_lift(v); % N, rear axle downforce, should be positive
F_flift = @(v) (1 - CoP_x) * F_lift(v); % N, front axle downforce, should be positive
F_fdrag = @(v) M_drag(v) / (x_fw - x_rw); % N, front axle drag moment contribution, should be negative
F_rdrag = @(v) -F_fdrag(v); % N, should be positive
F_ftotal = @(v) F_flift(v) + F_fdrag(v); % N, total aero load on front axle
F_rtotal = @(v) F_rlift(v) + F_rdrag(v); % N, total aero load on rear axle

figure;
hold on;
fplot(F_ftotal, [0 30]);
fplot(lon_xfr, [0 2]);
