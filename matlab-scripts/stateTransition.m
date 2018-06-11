function predictParticles = stateTransition(pf, prevParticles, uCmd, noiseLevel)

% Linear state transition
test;
stateTransitionModel = eye(pf.NumStateVariables);

% Sample the Gaussian noise
processNoise = eye(pf.NumStateVariables);
dist = matlabshared.tracking.internal.NormalDistribution(pf.NumStateVariables);
dist.Mean = zeros(1, pf.NumStateVariables);
dist.Covariance = processNoise;
noise = noiseLevel * dist.sample(pf.NumParticles);

% add position change
deltaPosition = ones(pf.NumParticles, 1) * uCmd;

% Evolve the particle state and add Gaussian noise
predictParticles = prevParticles * stateTransitionModel + deltaPosition + noise;

end
