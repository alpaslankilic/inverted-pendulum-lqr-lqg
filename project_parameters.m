function p = project_parameters()
%PROJECT_PARAMETERS SI units; pendulum angle is measured from upright.
p.cartMass = 0.65;
p.pendulumMass = 0.18;
p.halfLength = 0.28;
p.inertia = p.pendulumMass*(2*p.halfLength)^2/12;
p.cartFriction = 0.08;
p.gravity = 9.81;
p.forceLimit = 8;
p.sampleTime = 0.002;
p.initialState = [0; 0; 0.08; 0];
p.reference = 0.15;
p.referenceTime = 0.35;
p.disturbanceTime = 2.5;
p.disturbanceDuration = 0.12;
p.disturbanceForce = 0.7;
p.positionNoiseStd = 0.0003;
p.angleNoiseStd = 0.0007;
p.forceNoiseStd = 0.02;
p.seed = 73;
p.stopTime = 4.678;
end
