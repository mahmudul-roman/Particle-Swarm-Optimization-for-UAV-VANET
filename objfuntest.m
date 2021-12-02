function z = objfuntest(x, y, v, u, k)
        
    % Number of Uncovered Vehicles
     nVehicle = y;
    % Initializing Vehicles Position Struct
     vehicle = v;
    % Radio coverage of UAV in meters
    UAVRadius = 500;
    
    % Infrastructure Radius
    infRadius = 500;
    % Communication Range between RSU and UAV or UAV and UAV
    losRange = 1200;
    % Position of infrastructure
    pos = u;
    niter = k;
    
    % Initializing Covered Vehicles Position Struct (Moving Towards Center)
    center_covered_vehicle.Position = [];
    % Initializing Covered Vehicles Position Struct (Moving Towards Edge)
    edge_covered_vehicle.Position = [];
    % Number of Vehicles Covered (Moving Towards Center)
    M = 0;
    % Number of Vehicles Covered (Moving Towards Edge)
    N = 0;
    % For counting both covered and uncovered vehicles
    vehicle_count = 0;
    
    counter2 = 0;
    counter3 = 0;

    for i = 1:niter
        dist(i).inf_UAV = sqrt(sum((x - pos(i).inf) .^2));
        if dist(i).inf_UAV > (infRadius + UAVRadius)
            counter2 = counter2 + 1;
        end
        if dist(i).inf_UAV < losRange
            counter3 = 1;
        end
    end
    
    if counter2 == niter && counter3 == 1
        
        for i=1:nVehicle
            if ~isempty(vehicle(i).id)
                % Counting the total number of vehicles in the timeslot
                vehicle_count = vehicle_count + 1;
                % Calculate distance between particle and vehicle(i)
                Distance = sqrt(sum((x - vehicle(i).Position) .^2));
                if Distance <= UAVRadius
                    % Checking whether vehicle is going towards center or edge of
                    % UAV coverage
                    if vehicle(i).angle >= 45 && vehicle(i).angle < 135
                        newPosition = vehicle(i).Position + [Distance/UAVRadius, 0];
                        newDistance = sqrt(sum((x - newPosition) .^2));
                        if (Distance - newDistance) >= 0
                            % Number of vehicle covered by UAV (Moving Towards Center)
                            M = M + 1;
                            % Position of the Mth vehicle
                            center_covered_vehicle(M).Position = vehicle(i).Position;
                            center_covered_vehicle(M).c_eval = vehicle(i).connec_sum;
                        elseif (Distance - newDistance) < 0
                            % Number of vehicle covered by UAV (Moving Towards Edge)
                            N = N + 1;
                            % Position of the Nth vehicle
                            edge_covered_vehicle(N).Position = vehicle(i).Position;
                            edge_covered_vehicle(N).c_eval = vehicle(i).connec_sum;
                        end
                    elseif vehicle(i).angle >= 225 && vehicle(i).angle < 315
                        newPosition = vehicle(i).Position - [Distance/UAVRadius, 0];
                        newDistance = sqrt(sum((x - newPosition) .^2));
                        if (Distance - newDistance) >= 0
                            % Number of vehicle covered by UAV (Moving Towards Center)
                            M = M + 1;
                            % Position of the Mth vehicle
                            center_covered_vehicle(M).Position = vehicle(i).Position;
                            center_covered_vehicle(M).c_eval = vehicle(i).connec_sum;
                        elseif (Distance - newDistance) < 0
                            % Number of vehicle covered by UAV (Moving Towards Edge)
                            N = N + 1;
                            % Position of the Nth vehicle
                            edge_covered_vehicle(N).Position = vehicle(i).Position;
                            edge_covered_vehicle(N).c_eval = vehicle(i).connec_sum;
                        end
                    elseif vehicle(i).angle >= 315 || vehicle(i).angle < 45
                        newPosition = vehicle(i).Position + [0, Distance/UAVRadius];
                        newDistance = sqrt(sum((x - newPosition) .^2));
                        if (Distance - newDistance) >= 0
                            % Number of vehicle covered by UAV (Moving Towards Center)
                            M = M + 1;
                            % Position of the Mth vehicle
                            center_covered_vehicle(M).Position = vehicle(i).Position;
                            center_covered_vehicle(M).c_eval = vehicle(i).connec_sum;
                        elseif (Distance - newDistance) < 0
                            % Number of vehicle covered by UAV (Moving Towards Edge)
                            N = N + 1;
                            % Position of the Nth vehicle
                            edge_covered_vehicle(N).Position = vehicle(i).Position;
                            edge_covered_vehicle(N).c_eval = vehicle(i).connec_sum;
                        end
                    elseif vehicle(i).angle >= 135 && vehicle(i).angle < 225
                        newPosition = vehicle(i).Position - [0, Distance/UAVRadius];
                        newDistance = sqrt(sum((x - newPosition) .^2));
                        if (Distance - newDistance) >= 0
                            % Number of vehicle covered by UAV (Moving Towards Center)
                            M = M + 1;
                            % Position of the Mth vehicle
                            center_covered_vehicle(M).Position = vehicle(i).Position;
                            center_covered_vehicle(M).c_eval = vehicle(i).connec_sum;
                        elseif (Distance - newDistance) < 0
                            % Number of vehicle covered by UAV (Moving Towards Edge)
                            N = N + 1;
                            % Position of the Nth vehicle
                            edge_covered_vehicle(N).Position = vehicle(i).Position;
                            edge_covered_vehicle(N).c_eval = vehicle(i).connec_sum;
                        end 
                    end
                end
            end
        end
        s1 = 0;
        s2 = 0;
        % Calculating Fitness for Vehicles (Moving Towards Center)
        for j=1:M
            c1 = 1/(1 + exp(-(center_covered_vehicle(j).c_eval)));
            d1 = sqrt(sum((x - center_covered_vehicle(j).Position) .^2));
            e1 = d1/UAVRadius;
            normalize1 = 0.5 * e1 + 0.5; 
            multi1 = normalize1*c1;
            s1 = s1 + multi1;
        end

        % Calculating Fitness for Vehicles (Moving Towards Edge)
        for k=1:N
            c2 = 1/(1 + exp(-(edge_covered_vehicle(k).c_eval)));
            d2 = sqrt(sum((x - edge_covered_vehicle(k).Position) .^2));
            e2 = d2/UAVRadius;
            normalize2 = 1 - (0.5 * e2 + 0.5);
            multi2 = normalize2*c2;
            s2 = s2 + multi2;
        end
        eval_f1 = (s1 + s2)/vehicle_count;
    else
        eval_f1 = 0;
    end
    
    % Return Fitness Value and number of UAV
    if M > 0 || N > 0
        z.Fitness = eval_f1;
        z.M = M;
        z.N = N;
        z.V = vehicle_count;
    else
        z.Fitness = -inf;
        z.M = M;
        z.N = N;
        z.V = vehicle_count;
    end
    
end