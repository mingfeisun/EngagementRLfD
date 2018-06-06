function likelihood = measurementLikelihood(pf, predictParticles, measurement)

% Expected measurements for each particle (based on predicted state)
predictMeasurement = predictParticles; 

% Calculate error between predicted and actual measurement
measurementError = bsxfun(@minus, predictMeasurement, measurement);

measurementErrorNorm = sum(abs(measurementError), 2);

% Normal-distributed noise of measurement
% Assuming measurements on all three pose components have the same error distribution 
measurementNoise = eye(pf.NumStateVariables);
    
% Convert error norms into likelihood measure. 
% Evaluate the PDF of the multivariate normal distribution 
likelihood = 1/sqrt((2*pi).^3 * det(measurementNoise)) * exp(-2 * measurementErrorNorm);

end
