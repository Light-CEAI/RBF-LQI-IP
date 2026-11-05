function out = actuator_limit_step(Fprev,Frequest,act,Ts)
% Force saturation and slew-rate command limiter; not a motor model.
if Ts<=0 || act.forceMax<=0 || act.forceRateMax<=0
 error('Sample time and limits must be positive.');
end
Fsat=min(max(Frequest,-act.forceMax),act.forceMax);
dF=act.forceRateMax*Ts;
out.force=min(max(Fsat,Fprev-dF),Fprev+dF);
out.saturatedForce=Fsat; out.trackingError=out.force-Frequest;
out.rateLimited=abs(out.force-Fsat)>10*eps(max(1,abs(Fsat)));
end
