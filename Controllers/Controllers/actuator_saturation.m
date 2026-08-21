function u_sat = actuator_saturation(u, params)
%ACTUATOR_SATURATION Clip a commanded actuator signal to physical limits.
%
%   u_sat = actuator_saturation(u, params)
%
%   Config A: limit is the motor's max torque, tau_max [N*m]
%   Config B: limit is the max input force, u_max [N]
%
%   Kept as a standalone function (rather than inlined in lqi_controller.m)
%   so any controller — LQI, RBF-augmented, or a future one — saturates
%   identically and consistently.

switch params.config
    case 'A'
        limit = params.cart.tau_max;
    case 'B'
        limit = params.cart.u_max;
    otherwise
        error('actuator_saturation:badConfig', 'Unknown config ''%s''.', params.config);
end

u_sat = max(min(u, limit), -limit);

end