function inputs = experiment_inputs(t, p)
%EXPERIMENT_INPUTS Reproducible force disturbance and two encoder channels.
stream = RandStream('mt19937ar','Seed',p.seed);
inputs.measurementNoise = randn(stream,2,numel(t))' ...
    *diag([p.positionNoiseStd,p.angleNoiseStd]);
forceStream = RandStream('mt19937ar','Seed',p.seed+1);
inputs.processForce = p.forceNoiseStd*randn(forceStream,numel(t),1);
inputs.reference = p.reference*(t>=p.referenceTime);
inputs.disturbance = p.disturbanceForce*(t>=p.disturbanceTime ...
    & t<p.disturbanceTime+p.disturbanceDuration);
end
