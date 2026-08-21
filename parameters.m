function params = parameters(configChoice)
%PARAMETERS Central parameter definition for the Inverted Pendulum project.
%
%   params = PARAMETERS()           returns parameters using the DEFAULT
%                                    configuration set below ('A').
%   params = PARAMETERS('A')        Configuration A: cart with two wheels,
%                                    driven by motor torque.
%   params = PARAMETERS('B')        Configuration B: sliding cart with
%                                    direct linear force input u.
%
%   Every other file in the project (Models, Controllers, Observers, RBF,
%   Simulations, Utilities, Visualizations) should call this function
%   rather than hard-coding numbers, so the whole project can be
%   re-pointed at Configuration A or B from this single switch.
%
%   Struct layout
%   -------------
%   params.config            'A' or 'B'
%   params.pend.*            Parameters common to both configurations
%                             (pendulum mass, length, gravity, etc.)
%   params.cart.*            Configuration-specific cart/actuator
%                             parameters (fields differ between A and B,
%                             see below)
%   params.rbf.*             RBF network parameters
%   params.test.*            Disturbance / robustness test ranges
%
%   Configuration-specific fields
%   ------------------------------
%   Config A (wheels + motor torque):
%       params.cart.M        Cart mass               [kg]
%       params.cart.r        Wheel radius             [m]
%       params.cart.tau_max  Max motor torque         [N*m]
%       params.cart.b        Rolling resistance       [N*s/m]  (b_w)
%       params.cart.v_max    Max cart velocity        [m/s]
%       params.cart.F_max    Max equivalent force = tau_max / r   [N]
%
%   Config B (sliding cart, direct force input):
%       params.cart.M        Cart mass                [kg]
%       params.cart.u_max    Max input force          [N]
%       params.cart.b        Friction coefficient      [N*s/m]
%       params.cart.a_max    Max acceleration          [m/s^2]
%       params.cart.F_max    Max usable force = u_max  [N]  (alias, for
%                             code that wants a config-agnostic force cap)

% =========================================================================
% >>> EDIT HERE to change the project-wide default configuration <<<
DEFAULT_CONFIG = 'A';
% =========================================================================

if nargin < 1 || isempty(configChoice)
    configChoice = DEFAULT_CONFIG;
end

configChoice = upper(configChoice);
if ~ismember(configChoice, {'A', 'B'})
    error('parameters:badConfig', ...
        'configChoice must be ''A'' or ''B'', got ''%s''.', configChoice);
end

params.config = configChoice;

%% ---- 4. Common Pendulum Parameters (Table 2) --------------------------
params.pend.m       = 0.15;            % pendulum mass [kg]
params.pend.l       = 0.30;            % length to center of mass [m]
params.pend.L       = 0.60;            % total pendulum length [m]
params.pend.g       = 9.81;            % gravity [m/s^2]
params.pend.theta_max_deg = 20;        % operating angle range [deg]
params.pend.theta_max     = deg2rad(params.pend.theta_max_deg); % [rad]

%% ---- 2/3. Configuration-specific cart/actuator parameters -------------
switch params.config
    case 'A'
        % Configuration A: Cart with Two Wheels (Table 0)
        params.cart.M       = 0.5;         % cart mass [kg]
        params.cart.r       = 0.03;        % wheel radius [m]
        params.cart.tau_max = 0.3;         % max motor torque [N*m] (+-0.3)
        params.cart.b       = 0.06;        % rolling resistance [N*s/m]
                                            % (datasheet range 0.02-0.1,
                                            %  midpoint used as nominal)
        params.cart.b_range = [0.02, 0.1]; % full datasheet range, for
                                            % friction-variation tests
        params.cart.v_max   = 1.5;         % max cart velocity [m/s]
        params.cart.no_slip = true;        % no-slip condition assumed

        % Derived: equivalent linear force capability F = tau / r
        params.cart.F_max   = params.cart.tau_max / params.cart.r; % [N]

    case 'B'
        % Configuration B: Sliding Cart, direct force input u (Table 1)
        params.cart.M       = 0.5;         % cart mass [kg]
        params.cart.u_max   = 10;          % max input force [N] (+-10)
        params.cart.b       = 0.05;        % friction coefficient [N*s/m]
        params.cart.a_max   = 6;           % max acceleration [m/s^2]
        params.cart.actuator_model = 'direct_force';
        params.cart.has_wheel_dynamics = false;

        % Alias so downstream code can read F_max regardless of config
        params.cart.F_max   = params.cart.u_max; % [N]
end

%% ---- 6. RBF Parameters (Table 3) --------------------------------------
params.rbf.input_vector   = {'theta', 'theta_dot', 'x', 'x_dot'};
params.rbf.n_inputs       = numel(params.rbf.input_vector);
params.rbf.n_hidden       = 60;            % hidden neurons
params.rbf.center_method  = 'grid';        % grid-based over operating range
params.rbf.sigma_theta    = 0.25;          % [rad]
params.rbf.sigma_thetadot = 2.0;           % [rad/s]
params.rbf.learning_rate_range = [0.01, 0.1];
params.rbf.learning_rate  = 0.05;          % nominal default (mid-range)
params.rbf.weight_bounds  = [-10, 10];     % +-10
params.rbf.leakage        = 0.01;          % e-modification leakage gain,
params.rbf.alpha_innovation = 0.7;  % blend: 0=pure position tracking error,
% 1=pure observer innovation signal.
% 0.7 weights the innovation heavily,
% targeting unmodeled dynamic compensation
% rather than steady-state position offset.
% prevents unbounded weight
% drift during continuous
% unattended online adaptation
%% ---- NEW: Embedded control loop rate (single source of truth) --------
params.embedded.Ts = 0.01;        % 100 Hz — also use this in simulation_main.m
params.embedded.use_single = false; % set true if targeting single-precision MCU

%% ---- NEW: Sensor model (lab-feasible estimates — VERIFY ON HARDWARE) --
params.sensor.x_counts_per_meter   = 4000;   % e.g. quadrature encoder, ~0.25mm/count
params.sensor.x_zero_offset        = 0;      % counts, calibrate at startup
params.sensor.theta_counts_per_rad = 2000;   % e.g. potentiometer/encoder on pivot
params.sensor.theta_zero_offset    = 0;      % counts, calibrate at upright
params.sensor.adc_bits             = 12;
params.sensor.sample_rate_hz       = 100;    % should match params.embedded.Ts

% Quantization noise std (uniform quantization -> resolution/sqrt(12))
params.sensor.x_quant_std     = (1/params.sensor.x_counts_per_meter)/sqrt(12);
params.sensor.theta_quant_std = (1/params.sensor.theta_counts_per_rad)/sqrt(12);

%% ---- NEW: Actuator electrical model (Config A motor — VERIFY) --------
params.actuator.motor_R        = 2.5;    % winding resistance [Ohm]
params.actuator.motor_L        = 0.001;  % winding inductance [H]
params.actuator.motor_Kt       = 0.05;   % torque constant [N*m/A]
params.actuator.motor_Ke       = 0.05;   % back-EMF constant [V/(rad/s)]
params.actuator.supply_voltage = 12;     % [V]
params.actuator.current_limit  = 2.0;    % [A]
params.actuator.pwm_freq_hz     = 20000;
% Electrical time constant - if this is not << Ts, the "instant torque"
% assumption in nonlinear_plant.m / actuator_saturation.m is not valid:
params.actuator.tau_elec = params.actuator.motor_L / params.actuator.motor_R;

%% ---- NEW: Mechanical limits ------------------------------------------
params.mech.track_length   = 0.80;  % total rail length [m] -> x in [-0.4, 0.4]
params.mech.theta_hardstop_deg = 35; % physical stop angle [deg]

%% ---- NEW: Safety thresholds -------------------------------------------
params.safety.theta_trip_deg = 25;   % trip BEFORE the 35 deg hard stop
params.safety.x_trip_margin  = 0.05; % [m] stop short of the rail end
params.safety.max_consecutive_saturation_steps = 50; % ~0.5s at 100Hz
% Grid ranges used by rbf_initialize.m to place centers (derived from
% operating ranges already defined above, kept here for convenience).
params.rbf.theta_range     = [-params.pend.theta_max, params.pend.theta_max];
params.rbf.thetadot_range  = [-2*pi, 2*pi];      % [rad/s], generous default
params.rbf.x_range         = [-1, 1];            % [m], adjust to test rig

% Cart velocity range for RBF centers: Config A has an explicit v_max;
% Config B does not, so fall back to a generous default.
if strcmp(params.config, 'A')
    v_bound = params.cart.v_max;
else
    v_bound = 2.0; % [m/s] generous default for Config B
end
params.rbf.xdot_range      = [-v_bound, v_bound];
clear v_bound

%% ---- 5. Disturbance and Robustness Testing -----------------------------
params.test.force_dist_range_N      = [0.5, 3];        % [N]
params.test.init_angle_range_deg    = [5, 20];          % +- [deg]
params.test.mass_variation_pct      = 0.30;             % +-30%
params.test.friction_variation_pct  = 3.00;             % up to +300%
params.test.length_variation_pct    = [0.10, 0.15];     % +-10-15%

end
