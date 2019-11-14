close all
x_max = 1000;
y_max = x_max;
obstacle = zeros(x_max, y_max);

scaling = 2;
numIterate = 5000;
maxNodes = 1000;
b = 5;
% q_start = [100 900];
% q_goal = [750 50];
% 
% obstacle(201:250, 1:250) = 1;
% obstacle(251:500, 201:250) = 1;
% obstacle(601:650, 751:1000) = 1;
% obstacle(651:900, 751:800) = 1;
% obstacle(201:250, 401:1000) = 1;
% obstacle(251:500, 401:450) = 1;
% obstacle(601:650, 1:600) = 1;
% obstacle(651:900, 551:600) = 1;
% obstacle(251:500, 701:750) = 1;
% obstacle(651:900, 301:350) = 1;
% obstacle(451:500, 551:900) = 1;
% obstacle(851:900, 101:450) = 1;

% obstacle = [200, 0, 50, 250;
%             250, 200, 250, 50;
%             600, 750, 50, 250;
%             650, 750, 250, 50;
%             200, 400, 50, 600;
%             250, 400, 250, 50;
%             600, 0, 50, 600;
%             650, 550, 250, 50;
%             250, 700, 250, 50; 
%             650, 300, 250, 50;
%             450, 550, 50, 350;
%             850, 100, 50, 350];

q_start = [600 100];
q_goal = [500 950];

obstacle(301:600, 601:900) = 1;
obstacle(501:800, 201:500) = 1;

[route, init_n, init_cost, final_cost] = Copy_of_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);

% for b = 5:40:205
%     [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
% end

% obstacle = [300, 600, 300, 300;
%             500, 200, 300, 300];
% q_start = [600 100];
% q_goal = [500 950];
% for b = 5:40:205
%     [route, init_n, init_cost, final_cost] = Copy_of_Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
% end

%[route, init_n, init_cost] = Ivan_RRT (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%[route, init_n, init_cost, final_cost] = Ivan_RRTStar (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%[route, init_n, init_cost, final_cost] = Ivan_RRTStarFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal);
%[route, init_n, init_cost, final_cost] = Ivan_RRTStar_Smart (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%[route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
%[route, init_n, init_cost] = Ivan_SkilledRRT (scaling, x_max, y_max, obstacle, q_start, q_goal);
%[route, init_n, init_cost] = Ivan_AStar (x_max, y_max, obstacle, scaling, q_start, q_goal);
%state = DStarLite(100, 8000);


% %first obstacle
% disp('--------------FIRST OBSTACLE--------------');
% disp(' ');
% obstacle = [200, 0, 50, 250;
%             250, 200, 250, 50;
%             600, 750, 50, 250;
%             650, 750, 250, 50;
%             200, 400, 50, 600;
%             250, 400, 250, 50;
%             600, 0, 50, 600;
%             650, 550, 250, 50;
%             250, 700, 250, 50; 
%             650, 300, 250, 50;
%             450, 550, 50, 350;
%             850, 100, 50, 350];
% b = 5;

% disp('---------------- TESTING FOR maxNodes ----------------');        
% numIterate = 8000;
% 
% %max nodes
% for maxNodes = 900:100:900 %maxnodes from 300 to 1200
%     disp(['maxNodes = ' num2str(maxNodes)]);
%     
%     initz_n = [];
%     initz_cost = [];
%     finalz_cost = [];
%     numnodez = [];
%     timez = [];
%     
%     iterate = 20;
%     for m = 1:iterate
%         tic
%         
%         [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
%         
%         if ~isempty(route)
%             timez = [timez toc];
%             initz_n = [initz_n init_n];
%             initz_cost = [initz_cost init_cost];
%             finalz_cost = [finalz_cost final_cost];
%             numnodez = [numnodez length(route)-2];
%         end
%         
%     end
%     
%     disp(['time: avg = ' num2str(mean(timez)) ', std = ' num2str(std(timez))]);
%     disp(['init_n: avg = ' num2str(mean(initz_n)) ', std = ' num2str(std(initz_n))]);
%     disp(['init_cost: avg = ' num2str(mean(initz_cost)) ', std = ' num2str(std(initz_cost))]);
%     disp(['final_cost: avg = ' num2str(mean(finalz_cost)) ', std = ' num2str(std(finalz_cost))]);
%     disp(['numnode: avg = ' num2str(mean(numnodez)) ', std = ' num2str(std(numnodez))]);
%     disp(['success rate = ' num2str(length(timez)/iterate*100) '%']);
%     disp(' ');
% end
% 
% disp('---------------- TESTING FOR numIterate ----------------');
% maxNodes = 600;
% 
% %numIterate
% for numIterate = 5000:500:10000 %numIterate from 5000 to 10000
%     disp(['numIterate = ' num2str(numIterate)]);
%     
%     initz_n = [];
%     initz_cost = [];
%     finalz_cost = [];
%     numnodez = [];
%     timez = [];
%     
%     iterate = 20;
%     for m = 1:iterate
%         tic
%         
%         [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
%         
%         if ~isempty(route)
%             timez = [timez toc];
%             initz_n = [initz_n init_n];
%             initz_cost = [initz_cost init_cost];
%             finalz_cost = [finalz_cost final_cost];
%             numnodez = [numnodez length(route)-2];
%         end
%         
%     end
%     
%     disp(['time: avg = ' num2str(mean(timez)) ', std = ' num2str(std(timez))]);
%     disp(['init_n: avg = ' num2str(mean(initz_n)) ', std = ' num2str(std(initz_n))]);
%     disp(['init_cost: avg = ' num2str(mean(initz_cost)) ', std = ' num2str(std(initz_cost))]);
%     disp(['final_cost: avg = ' num2str(mean(finalz_cost)) ', std = ' num2str(std(finalz_cost))]);
%     disp(['numnode: avg = ' num2str(mean(numnodez)) ', std = ' num2str(std(numnodez))]);
%     disp(['success rate = ' num2str(length(timez)/iterate*100) '%']);
%     disp(' ');
% end
% 
% disp('---------------TESTING FOR COMPARISON--------------------');
% numIterate = 8000; %no need compare success rate
% maxNodes = 800;
% b = 5;
% for mark = 6:6
%     
%     disp(num2str(mark));
%     initz_n = [];
%     initz_cost = [];
%     finalz_cost = [];
%     numnodez = [];
%     timez = [];
%     
%     iterate = 20;
%     for m = 1:iterate
%         switch mark
%             case 1
%                 tic
%                 [route, init_n, init_cost] = Ivan_RRT (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%                 final_cost = init_cost;
%             case 2
%                 tic
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%             case 3
%                 tic
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStarFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal);
%             case 4
%                 tic
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar_Smart (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%             case 5
%                 tic
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
%             case 6
%                 tic
%                 [route, init_n, init_cost] = Ivan_AStar (x_max, y_max, obstacle, scaling, q_start, q_goal);
%                 final_cost = init_cost;
%                 %don't forget that A star number of nodes is not applicable
%             otherwise
%                 disp('None');
%         end
%         
%         if ~isempty(route)
%             timez = [timez toc];
%             initz_n = [initz_n init_n];
%             initz_cost = [initz_cost init_cost];
%             finalz_cost = [finalz_cost final_cost];
%             numnodez = [numnodez length(route)-2];
%         end
%         
%     end
%     
%     disp(['time: avg = ' num2str(mean(timez)) ', std = ' num2str(std(timez))]);
%     disp(['init_n: avg = ' num2str(mean(initz_n)) ', std = ' num2str(std(initz_n))]);
%     disp(['init_cost: avg = ' num2str(mean(initz_cost)) ', std = ' num2str(std(initz_cost))]);
%     disp(['final_cost: avg = ' num2str(mean(finalz_cost)) ', std = ' num2str(std(finalz_cost))]);
%     disp(['numnode: avg = ' num2str(mean(numnodez)) ', std = ' num2str(std(numnodez))]);
%     disp(['success rate = ' num2str(length(timez)/iterate*100) '%']);
%     disp(' ');
% end

% %second obstacle
% disp('--------------SECOND OBSTACLE--------------');
% disp(' ');
% obstacle = [300, 600, 300, 300;
%             500, 200, 300, 300];
% q_start = [600 100];
% q_goal = [500 950];
% b = 5;

% disp('---------------- TESTING FOR maxNodes ----------------');        
% numIterate = 8000;
% 
% %max nodes
% for maxNodes = 400:100:1200 %maxnodes from 300 to 1200
%     disp(['maxNodes = ' num2str(maxNodes)]);
%     
%     initz_n = [];
%     initz_cost = [];
%     finalz_cost = [];
%     numnodez = [];
%     timez = [];
%     
%     iterate = 20;
%     for m = 1:iterate
%         tic
%         
%         [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
%         
%         if ~isempty(route)
%             timez = [timez toc];
%             initz_n = [initz_n init_n];
%             initz_cost = [initz_cost init_cost];
%             finalz_cost = [finalz_cost final_cost];
%             numnodez = [numnodez length(route)-2];
%         end
%         
%     end
%     
%     disp(['time: avg = ' num2str(mean(timez)) ', std = ' num2str(std(timez))]);
%     disp(['init_n: avg = ' num2str(mean(initz_n)) ', std = ' num2str(std(initz_n))]);
%     disp(['init_cost: avg = ' num2str(mean(initz_cost)) ', std = ' num2str(std(initz_cost))]);
%     disp(['final_cost: avg = ' num2str(mean(finalz_cost)) ', std = ' num2str(std(finalz_cost))]);
%     disp(['numnode: avg = ' num2str(mean(numnodez)) ', std = ' num2str(std(numnodez))]);
%     disp(['success rate = ' num2str(length(timez)/iterate*100) '%']);
%     disp(' ');
% end
% 
% disp('---------------- TESTING FOR numIterate ----------------');
% maxNodes = 600;
% 
% %numIterate
% for numIterate = 5000:500:10000 %numIterate from 5000 to 10000
%     disp(['numIterate = ' num2str(numIterate)]);
%     
%     initz_n = [];
%     initz_cost = [];
%     finalz_cost = [];
%     numnodez = [];
%     timez = [];
%     
%     iterate = 20;
%     for m = 1:iterate
%         tic
%         
%         [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
%         
%         if ~isempty(route)
%             timez = [timez toc];
%             initz_n = [initz_n init_n];
%             initz_cost = [initz_cost init_cost];
%             finalz_cost = [finalz_cost final_cost];
%             numnodez = [numnodez length(route)-2];
%         end
%         
%     end
%     
%     disp(['time: avg = ' num2str(mean(timez)) ', std = ' num2str(std(timez))]);
%     disp(['init_n: avg = ' num2str(mean(initz_n)) ', std = ' num2str(std(initz_n))]);
%     disp(['init_cost: avg = ' num2str(mean(initz_cost)) ', std = ' num2str(std(initz_cost))]);
%     disp(['final_cost: avg = ' num2str(mean(finalz_cost)) ', std = ' num2str(std(finalz_cost))]);
%     disp(['numnode: avg = ' num2str(mean(numnodez)) ', std = ' num2str(std(numnodez))]);
%     disp(['success rate = ' num2str(length(timez)/iterate*100) '%']);
%     disp(' ');
% end

% disp('---------------TESTING FOR COMPARISON--------------------');
% numIterate = 8000; %no need compare success rate
% maxNodes = 800;
% b = 5;
% for mark = 6:7
%     
%     disp(num2str(mark));
%     initz_n = [];
%     initz_cost = [];
%     finalz_cost = [];
%     numnodez = [];
%     timez = [];
%     
%     iterate = 20;
%     for m = 1:iterate
%         switch mark
%             case 1
%                 tic
%                 [route, init_n, init_cost] = Ivan_RRT (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%                 final_cost = init_cost;
%             case 2
%                 tic
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%             case 3
%                 tic
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStarFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal);
%             case 4
%                 tic
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar_Smart (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%             case 5
%                 tic
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
%             case 6
%                 tic
%                 [route, init_n, init_cost] = Ivan_AStar (x_max, y_max, obstacle, scaling, q_start, q_goal);
%                 final_cost = init_cost;
%                 %don't forget that A star number of nodes is not applicable
%             case 7
%                 tic
%                 [route, init_n, init_cost] = Ivan_SkilledRRT (scaling, x_max, y_max, obstacle, q_start, q_goal);
%                 final_cost = init_cost;
%             otherwise
%                 disp('None');
%         end
%         
%         if ~isempty(route)
%             timez = [timez toc];
%             initz_n = [initz_n init_n];
%             initz_cost = [initz_cost init_cost];
%             finalz_cost = [finalz_cost final_cost];
%             numnodez = [numnodez length(route)-2];
%         end
%         
%     end
%     
%     disp(['time: avg = ' num2str(mean(timez)) ', std = ' num2str(std(timez))]);
%     disp(['init_n: avg = ' num2str(mean(initz_n)) ', std = ' num2str(std(initz_n))]);
%     disp(['init_cost: avg = ' num2str(mean(initz_cost)) ', std = ' num2str(std(initz_cost))]);
%     disp(['final_cost: avg = ' num2str(mean(finalz_cost)) ', std = ' num2str(std(finalz_cost))]);
%     disp(['numnode: avg = ' num2str(mean(numnodez)) ', std = ' num2str(std(numnodez))]);
%     disp(['success rate = ' num2str(length(timez)/iterate*100) '%']);
%     disp(' ');
% end

% obstacle = [200, 0, 50, 250;
%             250, 200, 250, 50;
%             600, 750, 50, 250;
%             650, 750, 250, 50;
%             200, 400, 50, 600;
%             250, 400, 250, 50;
%             600, 0, 50, 600;
%             650, 550, 250, 50;
%             250, 700, 250, 50; 
%             650, 300, 250, 50;
%             450, 550, 50, 350;
%             850, 100, 50, 350];
% b = 5;
% q_start = [100 900];
% q_goal = [750 50];
% numIterate = 8000;
% maxNodes = 800;
% obstacle = [300, 600, 300, 300;
%             500, 200, 300, 300];
% q_start = [600 100];
% q_goal = [500 950];
% for mark = 6:7
% 
%         switch mark
%             case 1
%                 [route, init_n, init_cost] = Ivan_RRT (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%             case 2
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%             case 3
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStarFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal);
%             case 4
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar_Smart (numIterate, x_max, y_max, obstacle, q_start, q_goal);
%             case 5
%                 [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
%             case 6
%                 [route, init_n, init_cost] = Ivan_AStar (x_max, y_max, obstacle, scaling, q_start, q_goal);
%                 %don't forget that A star number of nodes is not applicable
%             case 7
%                 [route, init_n, init_cost] = Ivan_SkilledRRT (scaling, x_max, y_max, obstacle, q_start, q_goal);
%             otherwise
%                 disp('None');
%         end
% 
% end
% for maxNodes = 400:200:1200
%     [route, init_n, init_cost, final_cost] = Copy_of_Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
% end

% obstacle = [300, 600, 300, 300;
%             500, 200, 300, 300];
% q_start = [600 100];
% q_goal = [500 950];
% for maxNodes = 400:200:400
%     [route, init_n, init_cost, final_cost] = Ivan_RRTStar_SmartFN (numIterate, maxNodes, x_max, y_max, obstacle, q_start, q_goal, b);
% end