function result = simulate_reference(t, inputs, p, design, useEstimator)
%SIMULATE_REFERENCE Sample-and-hold control with nonlinear RK4 integration.
state = p.initialState;
prediction = zeros(4,1);
states = zeros(numel(t),4);
estimates = zeros(numel(t),4);
force = zeros(numel(t),1);
for k = 1:numel(t)
    measurement = design.C*state + inputs.measurementNoise(k,:)';
    estimate = prediction + design.correction*(measurement-design.C*prediction);
    feedback = state;
    if useEstimator, feedback = estimate; end
    command = -design.K*feedback + design.Nbar*inputs.reference(k);
    command = min(p.forceLimit,max(-p.forceLimit,command));
    states(k,:) = state';
    estimates(k,:) = estimate';
    force(k) = command;
    prediction = design.Ad*estimate + design.Bd*command;
    applied = command + inputs.disturbance(k) + inputs.processForce(k);
    h = p.sampleTime;
    a = cartpole_dynamics(state,applied,p);
    b = cartpole_dynamics(state+h*a/2,applied,p);
    c = cartpole_dynamics(state+h*b/2,applied,p);
    d = cartpole_dynamics(state+h*c,applied,p);
    state = state+h*(a+2*b+2*c+d)/6;
    assert(all(isfinite(state)) && abs(state(3))<pi/2, ...
        'Pendulum left the upright operating region.');
end
result = struct('state',states,'estimate',estimates,'force',force);
end
