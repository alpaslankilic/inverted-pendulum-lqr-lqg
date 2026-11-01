function design = design_controller(p)
%DESIGN_CONTROLLER Sampled LQR and a steady-state Kalman measurement update.
M = p.cartMass;
m = p.pendulumMass;
l = p.halfLength;
I = p.inertia;
b = p.cartFriction;
g = p.gravity;
den = I*(M+m)+M*m*l^2;
A = [0,1,0,0; 0,-(I+m*l^2)*b/den,-m^2*g*l^2/den,0; ...
    0,0,0,1; 0,m*l*b/den,(M+m)*m*g*l/den,0];
B = [0;(I+m*l^2)/den;0;-m*l/den];
C = [1,0,0,0;0,0,1,0];
discrete = expm([A,B;zeros(1,5)]*p.sampleTime);
Ad = discrete(1:4,1:4);
Bd = discrete(1:4,5);
Q = diag([45,3,180,4]);
R = 0.25;
K = dlqr(Ad,Bd,Q,R);
position = [1,0,0,0];
Nbar = 1/(position*((eye(4)-Ad+Bd*K)\Bd));
% Qd represents unmodeled per-sample state evolution, not sensor variance.
Qd = diag([1e-10,2e-6,1e-10,4e-6]);
Rv = diag([p.positionNoiseStd^2,p.angleNoiseStd^2]);
P = dare(Ad',C',Qd,Rv);
correction = P*C'/(C*P*C'+Rv);
assert(rank(ctrb(A,B))==4,'Linearized plant is not controllable.');
assert(rank(obsv(A,C))==4,'Position/angle sensing is not observable.');
assert(all(abs(eig(Ad-Bd*K))<1),'Unstable sampled LQR.');
assert(all(abs(eig(Ad*(eye(4)-correction*C)))<1),'Unstable Kalman predictor.');
% Verify the analytical upright linearization against the nonlinear equations.
numerical = zeros(4);
step = 1e-6;
for k = 1:4
    offset = zeros(4,1);
    offset(k) = step;
    numerical(:,k) = (cartpole_dynamics(offset,0,p) ...
        -cartpole_dynamics(-offset,0,p))/(2*step);
end
assert(max(abs(numerical-A),[],'all')<1e-7,'Linearization sign or coefficient error.');
design = struct('A',A,'B',B,'C',C,'Ad',Ad,'Bd',Bd,'K',K, ...
    'Nbar',Nbar,'correction',correction,'Q',Q,'R',R,'Qd',Qd,'Rv',Rv);
end
