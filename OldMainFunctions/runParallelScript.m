data = load('sharedData.mat');
data.updatedVar = updatedValue; % Update or add data as needed
save('sharedData.mat', 'data');
=