function derivative = cartpole_dynamics(state, force, p)
%CARTPOLE_DYNAMICS Coupled nonlinear cart and rigid-rod equations.
velocity = state(2);
angle = state(3);
angularVelocity = state(4);
coupling = p.pendulumMass*p.halfLength*cos(angle);
mass = [p.cartMass+p.pendulumMass, coupling; ...
    coupling, p.inertia+p.pendulumMass*p.halfLength^2];
rhs = [force-p.cartFriction*velocity ...
    + p.pendulumMass*p.halfLength*angularVelocity^2*sin(angle); ...
    p.pendulumMass*p.gravity*p.halfLength*sin(angle)];
acceleration = mass\rhs;
derivative = [velocity; acceleration(1); angularVelocity; acceleration(2)];
end
