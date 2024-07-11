%began by running addpath(genpath('path to spatial v2')
% addpath(genpath('Path to spatial_v2 extended ))

% define model. 7 joints for non floating base
model = iiwa14();
fb_model = fb_iiwa14();
% setup floating base model 

display(model)
q = [-0.3369  1.2966 -0.6775 -1.4218 -0.7067 -0.135  -1.1495];
q = transpose(q);
qd = [0.,0.,0.,0.,0.,0.,0.];
qd = transpose(qd);
qdd = qd;

% fixed base torque values from RNEA
fixed_tau = ID(model, q,qd,qdd)

display(fb_model)
% copy paste from compare_rnea_with_spatial file verbatim
q_fb = {[-0.2044414500023999; 0.4871280855408087; 0.42956516185247917; -0.7323822045359349; -0.336899; 1.29662; -0.677475]; -0.296646; 2.13845; 2.00956; 1.55163; 2.2893; 0.0418005; -0.125271};
qd_fb = {[0.43302; -0.421561; -0.645439; -1.86055; -0.0130938; -0.458284]; 0.741174; 1.76642; 0.898011; -1.85675; 1.62223; 0.709379; -0.382885};
qdd_fb = {[0.741788; 1.92844; -0.903882; 0.0333959; 1.17986; -1.94599]; 0.32869; -0.139457; 2.00667; -0.519292; -0.711198; 0.376638; -0.209225};

% floating base torque values from RNEA
floating_tau = ID(fb_model, q_fb, qd_fb, qdd_fb)

% Joint space inertia matrix CRBA algorithm
H = HandC(fb_model, q_fb, qd_fb)

% Minverse floating base test
[Hinv] = Hinverse(fb_model,q_fb)