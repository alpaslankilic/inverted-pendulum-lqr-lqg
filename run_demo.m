function report = run_demo(showFigures)
%RUN_DEMO LQR/LQG on one nonlinear plant with matched disturbance realizations.
if nargin == 0, showFigures = true; end
root = fileparts(mfilename('fullpath'));
previous = pwd;
cleanup = onCleanup(@() cd(previous));
cd(root);
p = project_parameters();
design = design_controller(p);
probe = (0:p.sampleTime:8).';
probeInputs = experiment_inputs(probe,p);
settling = zeros(1,2);
for mode = 0:1
    result = simulate_reference(probe,probeInputs,p,design,mode);
    settling(mode+1) = settling_duration(probe,result.state,p);
end
pulseEnd = p.disturbanceTime+p.disturbanceDuration;
p.stopTime = ceil((pulseEnd+1.2*max(settling))/p.sampleTime)*p.sampleTime;
t = (0:p.sampleTime:p.stopTime).';
inputs = experiment_inputs(t,p);
names = {'LQR','LQG'};
result = cell(1,2);
metrics = struct([]);
if ~isfolder('results'), mkdir('results'); end
for mode = 0:1
    in = Simulink.SimulationInput('cart_pendulum');
    variables = {'p','K','Nbar','C','Ad','Bd','Mkalman','Fkalman', ...
        'useEstimator','Mc','mp','ell','Jp','fric','grav', ...
        'measurementNoise','processForce'};
    values = {p,design.K,design.Nbar,design.C,design.Ad,design.Bd, ...
        design.correction,eye(4)-design.correction*design.C,mode, ...
        p.cartMass,p.pendulumMass,p.halfLength,p.inertia, ...
        p.cartFriction,p.gravity,sampled_signal(inputs.measurementNoise), ...
        sampled_signal(inputs.processForce)};
    for k = 1:numel(variables)
        in = in.setVariable(variables{k},values{k},'Workspace','cart_pendulum');
    end
    in = in.setModelParameter('StopTime',num2str(p.stopTime,12), ...
        'SolverType','Fixed-step','Solver','ode4','FixedStep',num2str(p.sampleTime));
    out = sim(in);
    result{mode+1}.state = interp1(out.states.Time,out.states.Data,t,'linear');
    result{mode+1}.estimate = interp1(out.estimate.Time,out.estimate.Data,t,'linear');
    result{mode+1}.force = interp1(out.command.Time,out.command.Data,t,'previous');
    exact = simulate_reference(t,inputs,p,design,mode);
    error = max(abs(result{mode+1}.state-exact.state),[],'all');
    assert(error<1e-8,'Simulink and sample-by-sample RK4 disagree.');
    assert(max(abs(result{mode+1}.force))<=p.forceLimit+1e-8,'Force bound failed.');
    assert(max(abs(result{mode+1}.state(:,3)))<pi/2,'Pendulum fell.');
    settled = settling_duration(t,result{mode+1}.state,p);
    assert(p.stopTime>pulseEnd+settled,'Figure would truncate disturbance recovery.');
    after = t>=p.referenceTime;
    difference = result{mode+1}.estimate-result{mode+1}.state;
    metrics(mode+1).controller = names{mode+1};
    metrics(mode+1).peak_angle_deg = rad2deg(max(abs(result{mode+1}.state(:,3))));
    metrics(mode+1).peak_actuator_force_N = max(abs(result{mode+1}.force));
    metrics(mode+1).final_position_error_mm = ...
        1e3*abs(result{mode+1}.state(end,1)-p.reference);
    metrics(mode+1).post_pulse_settling_s = settled;
    metrics(mode+1).rk4_max_state_error = error;
    metrics(mode+1).velocity_estimation_rms_m_s = rms(difference(after,2));
    metrics(mode+1).angular_velocity_estimation_rms_rad_s = rms(difference(after,4));
    data = array2table([t,inputs.reference,inputs.disturbance, ...
        inputs.processForce,inputs.measurementNoise,result{mode+1}.state, ...
        result{mode+1}.estimate,result{mode+1}.force], 'VariableNames', ...
        {'time_s','reference_m','disturbance_N','force_noise_N', ...
        'position_noise_m','angle_noise_rad','cart_m','cart_velocity_m_s', ...
        'angle_rad','angular_velocity_rad_s','estimated_cart_m', ...
        'estimated_cart_velocity_m_s','estimated_angle_rad', ...
        'estimated_angular_velocity_rad_s','actuator_N'});
    writetable(data,fullfile('results',[lower(names{mode+1}) '.csv']));
end
report.controllers = metrics;
report.stop_time_s = p.stopTime;
report.sample_time_s = p.sampleTime;
report.position_settling_tolerance_m = 0.02*p.reference;
report.angle_settling_tolerance_rad = 0.02*abs(p.initialState(3));
report.noise_seed = p.seed;
report.verification = 'PASS';
write_json('results/metrics.json',report);
write_json('results/parameters.json',p);
write_json('results/controller_design.json',design);
plot_results(t,inputs,result,p,showFigures);
disp(report);
disp(struct2table(metrics));
end

function signal = sampled_signal(values)
% Empty time uses the block's sample index, avoiding floating-point time lookup.
signal.time = [];
signal.signals = struct('values',values,'dimensions',size(values,2));
end

function duration = settling_duration(t,states,p)
after = find(t>=p.disturbanceTime+p.disturbanceDuration);
outside = abs(states(:,1)-p.reference)>0.02*abs(p.reference) ...
    | abs(states(:,3))>0.02*abs(p.initialState(3));
last = find(outside(after),1,'last');
if isempty(last)
    duration = 0;
else
    assert(last<numel(after),'Response did not settle within the sizing horizon.');
    duration = t(after(last+1))-p.disturbanceTime-p.disturbanceDuration;
end
end

function write_json(path,value)
file = fopen(path,'w');
assert(file~=-1,'Cannot write output.');
cleanup = onCleanup(@() fclose(file));
fprintf(file,'%s\n',jsonencode(value,'PrettyPrint',true));
end
