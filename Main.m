% Start a parallel pool
poolobj = gcp;

% Define or reference your script functions
% Ensure these are actual functions. Convert scripts to functions if necessary.
function1 = @scriptFunction1; % Replace with your first function/script
function2 = @scriptFunction2; % Replace with your second function/script

% Run the functions in parallel
f1 = parfeval(poolobj, function1, nargout1); % Replace nargout1 with the number of outputs from function1
f2 = parfeval(poolobj, function2, nargout2); % Replace nargout2 with the number of outputs from function2

% Fetch results (if needed)
% result1 = fetchOutputs(f1);
% result2 = fetchOutputs(f2);

% Note: fetchOutputs will block until the function completes. If you do not need
% the output, you can omit these lines.
