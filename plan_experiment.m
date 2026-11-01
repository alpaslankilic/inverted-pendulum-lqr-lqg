function [p,timing] = plan_experiment(p,design)
%PLAN_EXPERIMENT Settle both controllers, apply the pulse, then settle again.
probe = (0:p.sampleTime:12).';
unforced = p;
unforced.disturbanceForce = 0;
inputs = experiment_inputs(probe,unforced);
initialSettled = zeros(1,2);
tolerance = zeros(2,4);
for mode = 0:1
    result = simulate_reference(probe,inputs,unforced,design,mode);
    scale = [abs(p.reference),max(abs(result.state(:,2))), ...
        abs(p.initialState(3)),max(abs(result.state(:,4)))];
    tolerance(mode+1,:) = 0.02*scale;
    initialSettled(mode+1) = settled_at(probe,result.state, ...
        p.referenceTime,p.reference,tolerance(mode+1,:));
end
p.disturbanceTime = max(initialSettled)+p.sampleTime;
inputs = experiment_inputs(probe,p);
finalSettled = zeros(1,2);
pulseEnd = p.disturbanceTime+p.disturbanceDuration;
for mode = 0:1
    result = simulate_reference(probe,inputs,p,design,mode);
    finalSettled(mode+1) = settled_at(probe,result.state, ...
        pulseEnd,p.reference,tolerance(mode+1,:));
end
lastSettled = max(finalSettled);
p.stopTime = ceil((lastSettled+0.1*(lastSettled-pulseEnd))/p.sampleTime)*p.sampleTime;
timing.initial_settled_at_s = initialSettled;
timing.force_pulse_start_s = p.disturbanceTime;
timing.force_pulse_end_s = pulseEnd;
timing.final_settled_at_s = finalSettled;
timing.post_pulse_settling_s = finalSettled-pulseEnd;
timing.state_tolerance = tolerance;
timing.state_order = {'position_m','velocity_m_s','angle_rad','angular_velocity_rad_s'};
timing.stop_time_s = p.stopTime;
timing.tail_fraction = 0.1;
timing.angle_reference_rad = 0;
timing.criteria = '2% of position request, initial lean, and each controller''s startup velocity peaks; true physical states';
assert(p.disturbanceTime>max(initialSettled),'Pulse precedes initial settling.');
assert(p.stopTime>lastSettled,'Plot truncates final recovery.');
end

function time = settled_at(t,states,startTime,reference,tolerance)
selected = find(t>=startTime);
deviation = states(selected,:)-[reference,0,0,0];
outside = any(abs(deviation)>tolerance,2);
last = find(outside,1,'last');
if isempty(last)
    time = startTime;
else
    assert(last<numel(selected),'Sizing horizon did not capture settling.');
    time = t(selected(last+1));
end
end
