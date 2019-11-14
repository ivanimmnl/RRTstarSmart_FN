function [route, init_n, init_cost, final_cost] = RRTStar_SmartFN_3D (numIterate, maxNodes, x_max, y_max, z_max, obstacle, q_start, q_goal, b)
close all

EPS = 20;
rewire_r = 100;
init_n = 0; %to store at which iteration the goal is reached
% b = 5; %Bias sampling near beacon every b samples
beacons_r = 75; %radius of beacon sampling
init_cost = 0;

q_start_cost = 0;
q_start_parent = 0;
n_goal = 0;

InitialPathFound = false;

nodes = {};
nodes{1} = q_start;
nodes_parent = [];
nodes_parent(1) = q_start_parent;
nodes_cost = [];
nodes_cost(1) = q_start_cost;

q_beacons_old = {};
q_beacons_new = {};
directcost_old = 0;
directcost_new = 0;

for i = 1:1:numIterate
    disp(i);
    disp(InitialPathFound);
    %Spawn random point
    if InitialPathFound && mod(i-init_n,b) == 0 %Spawn random point near beacon
        theta = pi*rand(1);
        psi = 2*pi*rand(1);
        radius = beacons_r*rand(1);
        j = randi(length(q_beacons_old)-2)+1;
        q_rand = [radius*sin(theta)*cos(psi) radius*sin(theta)*sin(psi) radius*cos(theta)] + q_beacons_old{j};
        q_rand(1) = floor(q_rand(1));
        q_rand(2) = floor(q_rand(2));
        q_rand(3) = floor(q_rand(3));
        
        %continue iteration if sample is outside workspace
        if q_rand(1) <= 0 || q_rand(1) > x_max || q_rand(2) <= 0 || q_rand(2) > y_max || q_rand(3) <= 0 || q_rand(3) > z_max
            continue
        end
    else
        q_rand = [randi([1 x_max], 1) randi([1 y_max], 1) randi([1 z_max], 1)];
    end
    
    % Pick the closest node to q_rand
    ndist = [];
    for j = 1:1:length(nodes)
        tmp = dist(nodes{j}, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes{idx};
    q_near_cost = nodes_cost(idx);
    
    q_new = steer(q_rand, q_near, val, EPS);
    
    if ObstacleFree(q_new, obstacle)
        
        % Choose best parent
        C_min = Inf;
        r = 60;
        neighbors = [];
        for j = 1:1:length(nodes)
            if dist(nodes{j}, q_new) <= r %to find neighboring nodes
                neighbors = [neighbors; j];
                if noCollision(nodes{j}, q_new, obstacle) && nodes_cost(j) + dist(nodes{j}, q_new) < C_min
                    q_new_parent = j;
                    C_min = nodes_cost(j) + dist(nodes{j}, q_new);
                    q_new_cost = C_min;
                end
            end
        end
        
        if ~isinf(C_min)
            %Append
            nodes{end+1} = q_new;
            nodes_cost(end+1) = q_new_cost;
            nodes_parent(end+1) = q_new_parent;
            
            %Rewire
            for j = 1:1:length(neighbors)-1
                index = neighbors(j);
                if noCollision(nodes{index}, q_new, obstacle) && dist(nodes{index}, q_new) < rewire_r && q_new_cost + dist(nodes{index}, q_new) < nodes_cost(index)
                    nodes_cost(index) =  q_new_cost + dist(nodes{index}, q_new);
                    nodes_parent(index) = length(nodes); %q_new should be the last element in nodes
                end
            end
            
            %Check if goal is reached
            if ~InitialPathFound && dist(q_new, q_goal) <= 5*EPS && noCollision(q_new, q_goal, obstacle) 
                if length(nodes) > maxNodes %forced removal
                    for jj = 1:2 %delete twice for q_new and q_goal
                        childlessNodes_idx = [];
                        for j = 1:1:length(nodes)-1
                            if isempty(find(nodes_parent == j))
                                childlessNodes_idx = [childlessNodes_idx j];
                            end
                        end
                        
                        %Don't delete nodes near goal
                        isneargoal = true;
                        while isneargoal
                            randnum = randi(length(childlessNodes_idx));
                            chosenidx = childlessNodes_idx(randnum);
                            if dist(nodes{chosenidx}, q_goal) <= 150 %don't delete node within 150 radius from q_goal
                                childlessNodes_idx(randnum) = [];
                                continue
                            end
                            isneargoal = false;
                        end
                        
                        if jj == 1
                            nodes{chosenidx} = q_new;
                            nodes_cost(chosenidx) = q_new_cost;
                            nodes_parent(chosenidx) = q_new_parent;
                            
                            nodes_parent(nodes_parent == length(nodes)) = chosenidx;
                            temp = chosenidx;
                        else
                            nodes{chosenidx} = q_goal;
                            nodes_cost(chosenidx) = q_new_cost + dist(q_new, q_goal);
                            nodes_parent(chosenidx) = temp;
                            n_goal = chosenidx;
                        end
                        
                        nodes(end) = [];
                        nodes_cost(end) = [];
                        nodes_parent(end) = [];
                    end
                else %do as per normal
                    nodes{end+1} = q_goal;
                    nodes_cost(end+1) = q_new_cost + dist(q_new, q_goal);
                    nodes_parent(end+1) = length(nodes)-1;
                    n_goal = length(nodes);
                end
                init_n = i;
                init_cost = q_new_cost + dist(q_new, q_goal);
                directcost_old = init_cost;
                InitialPathFound = true;
                q_beacons_old{1} = q_goal;
            end
            
            
            %Path optimization
            if InitialPathFound
                q_beacons_new = {};
                q_beacons_new{1} = nodes{n_goal};
                q_end = nodes{n_goal};
                q_end_parent = nodes_parent(n_goal);
                temp_beacon = nodes{n_goal};
                temp_beacon_parent = nodes_parent(n_goal);
                
                %Directly connect visible nodes
                while q_end_parent ~= 0
                    if noCollision(q_end, nodes{q_end_parent}, obstacle)
                        temp_beacon = nodes{q_end_parent};
                        temp_beacon_parent = nodes_parent(q_end_parent);
                        q_end_parent = nodes_parent(q_end_parent);
                        continue
                    end
                    q_end = temp_beacon;
                    q_end_parent = temp_beacon_parent;
                    q_beacons_new{end+1} = temp_beacon;
                end
                q_beacons_new{end+1} = q_start;
                
                %check for q_beacons_new total cost
                directcost_new = 0;
                for j = 1:1:length(q_beacons_new)-1
                    directcost_new = directcost_new + dist(q_beacons_new{j}, q_beacons_new{j+1});
                end
                                
                if directcost_new < directcost_old
                    q_beacons_old = q_beacons_new;
                    directcost_old = directcost_new;
                end
                
                
            end
            
            %Forced Removal
            if length(nodes) > maxNodes && InitialPathFound
                childlessNodes_idx = [];
                for j = 1:1:length(nodes)-1
                    if(j == n_goal)
                        continue
                    end
                    
                    if isempty(find(nodes_parent == j))
                        childlessNodes_idx = [childlessNodes_idx j];
                    end
                end
                
                %Don't delete nodes near beacon and goal
                isnearbeacon = true;
                while isnearbeacon
                    if length(childlessNodes_idx == 1) %If only one childless node left, delete that one
                        chosenidx = childlessNodes_idx(1);
                        break
                    else
                        count = 0;
                        randnum = randi(length(childlessNodes_idx));
                        chosenidx = childlessNodes_idx(randnum);
                        for j = 1:1:length(q_beacons_old)-1
                            if dist(nodes{chosenidx}, q_beacons_old{j}) <= beacons_r
                                count = 1;
                                childlessNodes_idx(randnum) = [];
                                break
                            end
                        end
                        
                        if count == 0
                            isnearbeacon = false;
                        end
                    end
                end
                
                %Move last node into the position of deleted node
                nodes{chosenidx} = q_new;
                nodes_cost(chosenidx) = q_new_cost;
                nodes_parent(chosenidx) = q_new_parent;
                
                %Update the parent position of nodes with last node as parent
                nodes_parent(nodes_parent == length(nodes)) = chosenidx;
                               
                %Delete last element
                nodes(end) = [];
                nodes_cost(end) = [];
                nodes_parent(end) = [];
                
            elseif length(nodes) > maxNodes && ~InitialPathFound
                childlessNodes_idx = [];
                for j = 1:1:length(nodes)-1
                    if isempty(find(nodes_parent == j))
                        childlessNodes_idx = [childlessNodes_idx j];
                    end
                end
                
                %Don't delete nodes near goal
                isneargoal = true;
                while isneargoal
                    randnum = randi(length(childlessNodes_idx));
                    chosenidx = childlessNodes_idx(randnum);
                    if dist(nodes{chosenidx}, q_goal) <= 150 %don't delete node within 150 radius from q_goal
                        childlessNodes_idx(randnum) = [];
                        continue
                    end
                    isneargoal = false;
                end
                
                nodes{chosenidx} = q_new;
                nodes_cost(chosenidx) = q_new_cost;
                nodes_parent(chosenidx) = q_new_parent;
                
                nodes_parent(nodes_parent == length(nodes)) = chosenidx;
                
                nodes(end) = [];
                nodes_cost(end) = [];
                nodes_parent(end) = [];
            end
            
        end
    end
end


%if goal is not reached after max iterations
if InitialPathFound == false
    route = [];
    init_n = 0;
    init_cost = 0;
    final_cost = 0;
    disp('Path not found');
else
    figure(1)
    axis([0 x_max 0 y_max 0 z_max])
    hold on
    
    plotcube([200 800 800], [300 0 0], 0.8, [0 0 1]);
    plotcube([200 700 700], [600 300 300], 0.8, [0 0 1]);
    
    plot3(q_goal(1), q_goal(2), q_goal(3), 'rs', 'MarkerSize', 20)
    plot3(q_start(1), q_start(2), q_start(3), '*', 'MarkerFaceColor', 'b', 'MarkerSize', 20)
    
    for j = 2:1:length(nodes)
        child = nodes{j};
        parent = nodes{nodes_parent(j)};
        line([child(1), parent(1)], [child(2), parent(2)], [child(3), parent(3)], 'Color', 'g');
    end
    
    route = [q_beacons_old{1}];
    for j = 1:1:length(q_beacons_old)-1
        child = q_beacons_old{j};
        parent = q_beacons_old{j+1};
        route = [route; parent];
        line([child(1), parent(1)], [child(2), parent(2)], [child(3), parent(3)], 'Color', 'r', 'LineWidth', 2);
        hold on
    end
    final_cost = directcost_old;
    disp('Done');
end
end
