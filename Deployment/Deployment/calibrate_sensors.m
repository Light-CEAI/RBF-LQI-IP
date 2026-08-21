function y = sensor_calibration(raw, params)
%SENSOR_CALIBRATION Raw encoder/ADC counts -> physical units [m, rad].
%
%   IMPORTANT: params.sensor.* values are lab-feasibility ESTIMATES from
%   the datasheet, not measured calibration constants. Re-derive these
%   against your actual encoder/potentiometer before trusting this.
%
%   Output is cast to single precision at this boundary, since this is
%   where raw hardware data first enters the control pipeline. Everything
%   downstream (control_step.m, rbf_forward.m, rbf_update.m) expects
%   single-precision inputs on the embedded (ESP32/STM32) path -- both
%   lack a double-precision FPU, so letting double leak in here would
%   either force slow software double-precision emulation at runtime or
%   trip MATLAB Coder's strict type checking when generating code,
%   depending on build settings.

x_counts     = raw(1);
theta_counts = raw(2);

x_meas     = (x_counts - params.sensor.x_zero_offset) / params.sensor.x_counts_per_meter;
theta_meas = (theta_counts - params.sensor.theta_zero_offset) / params.sensor.theta_counts_per_rad;

y = single([x_meas; theta_meas]);

end