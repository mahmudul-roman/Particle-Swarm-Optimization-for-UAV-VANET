clc;
clear;
close all;

%% Import Trace File

% Timmer started
tic;
% Import trace file(SUMO)
filename = 'trace.csv';

% Defining the timestept used from the dataset(SUMO)
timestep = 200; % Have to change according to the trace file
% Number of vehicles available in the dataset(SUMO)
nVehicle = 273; % Have to change according to the trace file
% Duration of time frame (seconds)
duration = 10;

% Infrastructure Position
pos_inf = [1400,1000]; % Have to change according to the trace file
infRadius = 500;

% Reading table
T = readtable(filename);
% Number of rows in the table
n_rows = height(T);

% Finding minimum and maximum of Position X and Y
minPosX = min(T{:,4});
minPosY = min(T{:,5});
maxPosX = max(T{:,4});
maxPosY = max(T{:,5});

% Vehicle template
empty_vehicle.id = [];
empty_vehicle.Position = [];
empty_vehicle.angle = [];

% Creating templates for storing Previous connectivity history of vehicles
empty_pre_conection.id = [];
empty_pre_conection.id1 = [];
empty_pre_conection.t1 = 0;
empty_pre_conection.id2 = [];
empty_pre_conection.t2 = 0;
empty_pre_conection.id3 = [];
empty_pre_conection.t3 = 0;
empty_pre_conection.id4 = [];
empty_pre_conection.t4 = 0;
empty_pre_conection.id5 = [];
empty_pre_conection.t5 = 0;

% Create vehicles connections history array
pre_conection = repmat(empty_pre_conection, nVehicle, 1);

% Create vehicles array
object_vehicle = repmat(empty_vehicle, nVehicle, 1);

% Creating vehicles id for 'connection history' data structure
for i=1:nVehicle
    pre_conection(i).id = ['veh' num2str((i-1), '%d')];
end

% Calculating whether the vehicles has connectivity in the previous time
% slot or not
for i=1:n_rows
    if T{i,1} == timestep
        for j=1:nVehicle
            if strcmp(T{i,3},['veh' num2str((j-1),'%d')])
                object_vehicle(j).id = T{i,3};
                object_vehicle(j).angle = T{i,2};
                object_vehicle(j).Position = [T{i,4}, T{i,5}];
            end
        end
    elseif T{i,1} == (timestep - duration)
        for j=1:nVehicle
            if strcmp(T{i,3},['veh' num2str((j-1),'%d')])
                pre_conection(j).id1 = T{i,3};
                dist = sqrt(sum((pos_inf - [T{i,4}, T{i,5}]) .^2));
                if dist <= 500
                    pre_conection(j).t1 = 1;
                end
            end
        end
    elseif T{i,1} == (timestep - (2*duration))
        for j=1:nVehicle
            if strcmp(T{i,3},['veh' num2str((j-1),'%d')])
                pre_conection(j).id2 = T{i,3};
                dist = sqrt(sum((pos_inf - [T{i,4}, T{i,5}]) .^2));
                if dist <= 500
                    pre_conection(j).t2 = 1;
                end
            end
        end
    elseif T{i,1} == (timestep - (3*duration))
        for j=1:nVehicle
            if strcmp(T{i,3},['veh' num2str((j-1),'%d')])
                pre_conection(j).id3 = T{i,3};
                dist = sqrt(sum((pos_inf - [T{i,4}, T{i,5}]) .^2));
                if dist <= 500
                    pre_conection(j).t3 = 1;
                end
            end
        end
    elseif T{i,1} == (timestep - (4*duration))
        for j=1:nVehicle
            if strcmp(T{i,3},['veh' num2str((j-1),'%d')])
                pre_conection(j).id4 = T{i,3};
                dist = sqrt(sum((pos_inf - [T{i,4}, T{i,5}]) .^2));
                if dist <= 500
                    pre_conection(j).t4 = 1;
                end
            end
        end
    elseif T{i,1} == (timestep - (5*duration))
        for j=1:nVehicle
            if strcmp(T{i,3},['veh' num2str((j-1),'%d')])
                pre_conection(j).id5 = T{i,3};
                dist = sqrt(sum((pos_inf - [T{i,4}, T{i,5}]) .^2));
                if dist <= 500
                    pre_conection(j).t5 = 1;
                end
            end
        end
    end
end


% Storing available vehicle's position in this time-slot (For scatter
% ploting only)
counter = 1;
for i=1:nVehicle
    if ~isempty(object_vehicle(i).id)
        x(counter) = object_vehicle(i).Position(1);
        y(counter) = object_vehicle(i).Position(2);
        all_vehicle(counter).id = object_vehicle(i).id;
        all_vehicle(counter).angle = object_vehicle(i).angle;
        all_vehicle(counter).Position = object_vehicle(i).Position;
        all_vehicle(counter).connec_sum = pre_conection(i).t1 + pre_conection(i).t2 + pre_conection(i).t3 ...
            + pre_conection(i).t4 + pre_conection(i).t5;
        counter = counter + 1;
    end
end

% Number of vehicles both covered and uncovered
nall_vehicle = size(all_vehicle, 2);
counter1 = 1;

% Finding all the uncovered vehicles
for i = 1:nall_vehicle
    dist_inf = sqrt(sum((pos_inf - all_vehicle(i).Position) .^2));
    if dist_inf > infRadius
        uncovered_vehicle(counter1).id = all_vehicle(i).id;
        uncovered_vehicle(counter1).angle = all_vehicle(i).angle;
        uncovered_vehicle(counter1).Position = all_vehicle(i).Position;
        uncovered_vehicle(counter1).connectivity = all_vehicle(i).connec_sum;
        counter1 = counter1 + 1;
    end
end

% Number of uncovered vehicles
nuncovered_vehicle = size(uncovered_vehicle, 2);

% Timmer stoped
toc
%% Problem Definition
tic;

nVar = 2;   % Decision Variable

VarSize = [1 nVar]; % Matrix Size of Decision Variables

VarMin = min(minPosX, minPosY);   % Lower Bound of Decision Variables
VarMax = max(maxPosX, maxPosY);    % Upper Bound of Decision Variables

%% Parameters of PSO

MaxIt = 300;    % Maximum Number of Iterations

nPop = 200;  % Population Size

w = 1;      % Intertia Coefficient
wdamp = 0.99; % Damping Ratio of Intertia Coefficient
c1 = 2;     % Personal Acceleration Coefficient
c2 = 2;     % Social Acceleration Coefficient

MaxVelocity = 0.15*(VarMax-VarMin);
MinVelocity = -MaxVelocity;

%% Initialization

% The particle template
empty_particle.Position = [];
empty_particle.Velocity = [];
empty_particle.Cost = [];
empty_particle.Best.Position = [];
empty_particle.Best.Cost = [];

% create population array
particle = repmat(empty_particle, nPop, 1);

% Initialize Global Best
GlobalBest.Cost = -inf;

% Initialize population members
for i=1:nPop
   
    % Generate Random Solution
    particle(i).Position = unifrnd(VarMin, VarMax, VarSize);
    
    % Initialize Velociy
    particle(i).Velocity = zeros(VarSize);
   
    % Evaluation
    evaluation = feval('objfun',particle(i).Position, nuncovered_vehicle, uncovered_vehicle, pos_inf);
    particle(i).Cost = evaluation.Fitness;
    nMi = evaluation.M;
    nNi = evaluation.N;
    
    % Update the Personal Best
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost > GlobalBest.Cost
        GlobalBest = particle(i).Best;
    end
    
end
% Array to Hold Best Cost Value on Each Iteration
BestCosts = zeros(MaxIt, 1);

%%  Main Loop of PSO

for it=1:MaxIt
    
    for i=1:nPop
       
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position) ...
            + c2*rand(VarSize).*(GlobalBest.Position -particle(i).Position);
        
        % Apply Velocity Limits
        particle(i).Velocity = max(particle(i).Velocity, MinVelocity);
        particle(i).Velocity = min(particle(i).Velocity, MaxVelocity);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Apply Lower and Upper Bound Limits
        particle(i).Position = max(particle(i).Position, VarMin);
        particle(i).Position = min(particle(i).Position, VarMax);
        
        % Evaluation
        evaluation = feval('objfun',particle(i).Position, nuncovered_vehicle, uncovered_vehicle, pos_inf);
        particle(i).Cost = evaluation.Fitness;
        
        % Update Personal Best
        if particle(i).Cost > particle(i).Best.Cost
            
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost > GlobalBest.Cost
                GlobalBest = particle(i).Best;
                nM = evaluation.M;
                nN = evaluation.N;
            end

        end
        
    end
    
    % Store the Best Cost Value
    BestCosts(it) = GlobalBest.Cost;
    
    % Display Iteration Information
    disp(['Iteration ' num2str(it) ': Fitness = ' num2str(BestCosts(it)')]);
    
    % Damping Intertia Coefficient
    w = w * wdamp;
    
end

toc

%% Results
close all;

% Displaying Results
disp(['UAV Position: ' num2str(GlobalBest.Position)]);
disp(['Fitness: ' num2str(GlobalBest.Cost)]);
disp(['Total number of uncovered vehicles: ' num2str(evaluation.V)]);
disp(['Total number of vehicles covered by UAV: ' num2str(nM+nN)]);
disp(['Vehicles moving towards center: ' num2str(nM)]);
disp(['Vehicles moving towards edge: ' num2str(nN)]);

% Figure 1: Displaying Covergence (Normal Plot)
figure;
plot(BestCosts, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Fitness');
grid on;

% Figure 2: Displaying Covergence (Semilogy Plot)
figure;
semilogy(BestCosts, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Fitness');
grid on;

% Figure 3: Ploting all vehicles position with UAV
figure;
scatter(x,y,'s');
hold on
scatter(GlobalBest.Position(1),GlobalBest.Position(2),'d','r','filled');
text(GlobalBest.Position(1)-50,GlobalBest.Position(2)-50,'UAV');
hold on
scatter(pos_inf(1),pos_inf(2),'o','k','filled');
text(pos_inf(1)-50,pos_inf(2)-50,'RSU');
xlim([VarMin VarMax])
ylim([VarMin VarMax])
axis square
viscircles(GlobalBest.Position, 500,'Color','r');
viscircles(pos_inf, 500,'Color','k');
grid on;
