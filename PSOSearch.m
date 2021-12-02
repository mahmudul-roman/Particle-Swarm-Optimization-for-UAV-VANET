function out = PSOSearch(param, position)

    % Import trace file(SUMO)
    filename = param.filename;
    filename_obj = param.filename_obj;
    filename_connec = param.filename_connec;

    % Number of vehicles available in the dataset(SUMO)
    nVehicle = param.nVehicle; % Have to change according to the trace file
    niter =param.niter;

    % Infrastructure Position
    pos = position; % Have to change according to the trace file
    infRadius = 500;

    % Reading table
    T = readtable(filename);
    T_obj = readtable(filename_obj);
    T_connec = readtable(filename_connec);
    % Number of rows in the table
    n_rows_obj = height(T_obj);

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
    for i=1:n_rows_obj
        for j=1:nVehicle
            if strcmp(T_obj{i,1},['veh' num2str((j-1),'%d')])
                object_vehicle(j).id = T_obj{i,1};
                object_vehicle(j).angle = T_obj{i,2};
                object_vehicle(j).Position = [T_obj{i,3}, T_obj{i,4}];
            end
        end
    end

    % Creating vehicles id for 'connection history' data structure
    for i=1:nVehicle
        pre_conection(i).id = ['veh' num2str((i-1), '%d')];
        pre_conection(i).id1 = T_connec{i,1};
        pre_conection(i).t1 = T_connec{i,2};
        pre_conection(i).id2 = T_connec{i,3};
        pre_conection(i).t2 = T_connec{i,4};
        pre_conection(i).id3 = T_connec{i,5};
        pre_conection(i).t3 = T_connec{i,6};
        pre_conection(i).id4 = T_connec{i,7};
        pre_conection(i).t4 = T_connec{i,8};
        pre_conection(i).id5 = T_connec{i,9};
        pre_conection(i).t5 = T_connec{i,10};
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
            all_vehicle(counter).x = object_vehicle(i).Position(1);
            all_vehicle(counter).y = object_vehicle(i).Position(2);
            all_vehicle(counter).connec_sum = pre_conection(i).t1 + pre_conection(i).t2 + pre_conection(i).t3 ...
                + pre_conection(i).t4 + pre_conection(i).t5;
            counter = counter + 1;
        end
    end

    % Number of vehicles both covered and uncovered
    uncovered_vehicle = all_vehicle;
    nall_vehicle = size(uncovered_vehicle, 2);
    counter1 = 1;

    % Finding all the uncovered vehicles
    for j = 1:niter
        for i = 1:nall_vehicle
            dist(j).inf = sqrt(sum((pos(j).inf - uncovered_vehicle(i).Position) .^2));
            if dist(j).inf > infRadius
                uncovered_vehicle(counter1).id = uncovered_vehicle(i).id;
                uncovered_vehicle(counter1).angle = uncovered_vehicle(i).angle;
                uncovered_vehicle(counter1).Position = uncovered_vehicle(i).Position;
                uncovered_vehicle(counter1).connec_sum = uncovered_vehicle(i).connec_sum;
                counter1 = counter1 + 1;
            end
        end
        uncovered_vehicle = uncovered_vehicle(1:counter1-1);
        nall_vehicle = size(uncovered_vehicle, 2);
        counter1 = 1;
    end

    % Number of uncovered vehicles
    nuncovered_vehicle = size(uncovered_vehicle, 2);

    % Problem Definition
    nVar = 2;   % Decision Variable
    VarSize = [1 nVar]; % Matrix Size of Decision Variables
    VarMin = min(minPosX, minPosY);   % Lower Bound of Decision Variables
    VarMax = max(maxPosX, maxPosY);    % Upper Bound of Decision Variables

    % Parameters of PSO
    MaxIt = param.MaxIt;    % Maximum Number of Iterations
    nPop = param.nPop;  % Population Size
    w = 1;      % Intertia Coefficient
    wdamp = 0.99; % Damping Ratio of Intertia Coefficient
    c1 = 2;     % Personal Acceleration Coefficient
    c2 = 2;     % Social Acceleration Coefficient
    MaxVelocity = 0.15*(VarMax-VarMin);
    MinVelocity = -MaxVelocity;

    % Initialization
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
    GlobalBest.Position = [0,0];

    % Initialize population members
    for i=1:nPop
        % Generate Random Solution
        particle(i).Position = unifrnd(VarMin, VarMax, VarSize);

        % Initialize Velociy
        particle(i).Velocity = zeros(VarSize);

        % Evaluation
        evaluation = objfuntest(particle(i).Position, nuncovered_vehicle, uncovered_vehicle, pos, niter);
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
    
    nM = 0;
    nN = 0;

    %  Main Loop of PSO
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
            evaluation = objfuntest(particle(i).Position, nuncovered_vehicle, uncovered_vehicle, pos, niter);
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

        % Damping Intertia Coefficient
        w = w * wdamp;
    end
    out.Fitness = GlobalBest.Cost;
    out.Position = GlobalBest.Position;
    out.Uncov_veh = evaluation.V;
    out.nM = nM;
    out.nN = nN;
    out.x = x;
    out.y = y;
    out.VarMin = VarMin;
    out.VarMax = VarMax;
    out.Total_vehicle = size(all_vehicle, 2);
    out.Best_cost = BestCosts;
end
