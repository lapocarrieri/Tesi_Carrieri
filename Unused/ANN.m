data(1)=load('NN1.mat');
data(2)=load('NN2.mat');
data(3)=load('NN3.mat');
data(4)=load('NN5.mat');
data(5)=load('NN7.mat');
X1=data(1).Residual_calculated; % Nx7 matrix of input vectors
Y1= 4*ones(length(X1),1); % Nx1 vector of boolean outputs
X2=data(2).Residual_calculated; % Nx7 matrix of input vectors
Y2= zeros(length(X2),1); 
X3=data(3).Residual_calculated; % Nx7 matrix of input vectors
Y3= 7*ones(length(X3),1); 
X4=data(4).Residual_calculated; % Nx7 matrix of input vectors
Y4= 6*ones(length(X4),1); 
X5=data(5).Residual_calculated; % Nx7 matrix of input vectors
Y5= 5*ones(length(X5),1); 


X=[X1;X2;X3;X4;X5];
Y=[Y1;Y2;Y3;Y4;Y5];
num_classes = 2;

% Load the dataset
% assuming the dataset is stored in a file called "dataset.mat"


% Define the input and output variables
% Define the neural network architecture

net = feedforwardnet([8 16 32 32 32 16 8]); % 2 hidden layers with 16 and 8 nodes, respectively
net.layers{1}.transferFcn = 'poslin';
net.trainFcn = 'trainlm'; % Levenberg-Marquardt backpropagation algorithm
net.divideFcn = 'dividerand'; % Randomly divide data into training, validation and testing sets
net.performFcn = 'mse'; 
net.layers{4}.transferFcn = 'poslin';
net.layers{5}.transferFcn = 'poslin';% set the activation function for the second layer to ReLU
net.layers{6}.transferFcn = 'poslin'; 
%= num_classes;
net.layers{end}.transferFcn = 'softmax'; % set the activation function for the output layer to sigmoid

num_classes = 2;
net.layers{end}.size = num_classes;
% Train the neural network
net = train(net, X', Y'); % transpose X and Y to match the required format

% Evaluate the performance of the neural network on the training data

Y_pred = net(X1(78,:)')

Y_pred = net(X2(88,:)')
Y_pred = net(X3(222,:)')
Y_pred = net(X3(222,:)')
Y_pred = net(X4(222,:)')
Y_pred = net(X4(222,:)')
Y_pred = net(X5(222,:)')

% get the predicted outputs for the input data
accuracy = sum(round(Y_pred) == Y') / length(Y); % calculate the accuracy
fprintf('Accuracy on the training data: %.2f%%\n', accuracy*100);

% output_vector = net(input_vector'); % predict the output for the input vector
% 
% % Convert the output vector to an index between 1 and 7
% [~, output_index] = max(output_vector); % find the index of the maximum value in the output vector
% output = output_index; % convert the index to a scalar value