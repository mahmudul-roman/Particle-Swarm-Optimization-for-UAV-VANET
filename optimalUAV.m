clc;
clear;
close all;
tic;
%% Initialization

param.filename = 'trace_low_70.csv';
param.filename_obj = 'object_vehicle_table_low_70_100.csv';
param.filename_connec = 'connection_t_low_70_100.csv';

param.nVehicle = 70;
param.niter = 2; % No. of RSUs
param.MaxIt = 150;
param.nPop = 300;

position(1).inf = [2500,1000]; % RSU 1
position(2).inf = [375,2000]; % RSU 2
coverage_thrs = 0.05;

%% Calling PSO
nM = 0;
nN = 0;
counter1 = 2; % No. of RSUs

while 1
    out = PSOSearch(param, position);
    coverage_impr = (out.nM + out.nN)/out.Total_vehicle;
    
    x = out.x;
    y = out.y;
    VarMin = out.VarMin;
    VarMax = out.VarMax;
    
    if coverage_impr < coverage_thrs
        break;
    end
    
    Fitness(counter1) = out.Fitness;
    UAV_Position = out.Position;
    Uncov_veh = out.Uncov_veh;
    covered_veh(counter1) = out.nM + out.nN;
    nM = nM + out.nM;
    nN = nN + out.nN;
    counter1 = counter1 + 1;
    position(counter1).inf = UAV_Position;
    param.niter = param.niter + 1;
    
    BestCosts = out.Best_cost;
end
toc
%% Results

% Figure 1: Displaying Covergence (Normal Plot)
figure;
plot(BestCosts, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Fitness');
grid on;

% Displaying Results
disp(['Total number of vehicles covered by UAV: ' num2str(nM+nN)]);
disp(['Vehicles moving towards center: ' num2str(nM)]);
disp(['Vehicles moving towards edge: ' num2str(nN)]);

% Figure 2: Ploting all vehicles position with UAV
figure;
scatter(position(1).inf(1),position(1).inf(2),'o','k','filled');
text(position(1).inf(1)-50,position(1).inf(2)-50,'RSU1');
hold on
scatter(position(2).inf(1),position(2).inf(2),'o','k','filled');
text(position(2).inf(1)-50,position(2).inf(2)-50,'RSU2');
for i = 3:param.niter
    scatter(position(i).inf(1),position(i).inf(2),'d','r','filled');
    text(position(i).inf(1)-50,position(i).inf(2)-50,['UAV ' num2str(i-2)]);
    disp(['UAV ' num2str(i-2) ' Position: ' num2str(position(i).inf(1)) ',' num2str(position(i).inf(2))]);
end
xlim([0 5000])
ylim([0 5000])
%ylim([VarMin VarMax])
axis square
viscircles(position(1).inf, 500,'Color','g');
viscircles(position(2).inf, 500,'Color','g');
for i = 3:param.niter
    viscircles(position(i).inf, 500,'Color','r');
end
hold on
scatter(x,y,'s','b','filled');
grid on;