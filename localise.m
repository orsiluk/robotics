function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

% UltraScan Returns returns two arrays, one with the distances and one
% with the crossing points of the scan rays.'


%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);


%generate some random particles inside the map
num =300; % number of particles
partPos = zeros(3, num);
dist = zeros(3, num);

particles(num,1) = BotSim; %how to set up a vector of objects
resPart = zeros(num, 3); % position(1,2), angle(3)

pathBot = BotSim(modifiedMap);
pathBot.setScanConfig(generateScanConfig(pathBot,6));

estBot = BotSim(modifiedMap); % estimated bot position
pathBot.setScanConfig(generateScanConfig(estBot,6));

nrGraph = 50;
randCord = zeros(2, nrGraph); %random coordonates on the map to find shortest path
for i = 1:num
    particles(i) = BotSim(modifiedMap);  % each particle should use the same
    % map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(generateScanConfig(particles(i),6));
    partPos(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
    particles(i).drawBot(3);
end

%% Iterate until you find the target

maxNumOfIterations = 60;
var = 10; % variance for Gussian
sqrt2PiVar = sqrt(2*pi*var);
damping = 0.000000001; %damping factor
minrad = 0.523599;
mindist = 2;

botSim.setScanConfig(botSim.generateScanConfig(6));
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(5);
partWeight = zeros(num,1);
notThere = 1;
steps = 0;

while notThere
    %% Localisation code
    
    converged =0; %The filter has not converged yet
    n = 0;
    
    while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
        n = n+1; %increment the current number of iterations
        BScan = botSim.ultraScan(); %get a scan from the real robot.
        
        %% Write code for updating your particles scans
        for i=1:num
            if particles(i).insideMap() == 1
                PScan(:,i) = particles(i).ultraScan();
                %% Write code for scoring your particles
                difference = sqrt(sum((BScan - PScan(:,i)).^2));
                denom = 2*var;
                partWeight(i) = (1 /sqrt2PiVar) * exp(-(difference)^2 /denom)+damping;
            else
                partWeight(i) = 0;
            end
        end
        
        allweights = sum(partWeight);
        for i = 1:num
            partWeight(i) = partWeight(i)/allweights; %normalize weights
        end
        
        %% Write code for resampling your particles
        cumulativeSum = cumsum(partWeight);
        
        for i = 1:num
            partic = find(rand() <= cumulativeSum,1);
            resPart(i,1:2) = particles(partic).getBotPos();
            resPart(i, 3) = particles(partic).getBotAng();
        end
        
        if steps == 0
            for i = 1:num
                particles(i).setBotPos([resPart(i,1), resPart(i,2)]);
                particles(i).setBotAng(resPart(i,3));
                turn = randi(2);
                move = randi(1);
                particles(i).turn(turn);
                particles(i).move(move);
            end
        end
        
        
        %% Write code to check for convergence
        for i = 1:num
            partPos(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
        end
        meanPosX = mean(partPos(1,:));
        meanPosY = mean(partPos(2,:));
        meanAng = mean(partPos(3,:));
        
        for i = 1:num
            dist(:,i) = [abs(partPos(1,i)-meanPosX) abs(partPos(2,i)-meanPosY) abs(partPos(3,i)-meanAng)];
        end
        
        distX = sum(dist(1,:)<mindist); %nr of paricles that are withing distX (2)
        distY = sum(dist(2,:)<mindist); %nr of paricles that are withing distY (2)
        distAng = sum(dist(3,:)<minrad);%nr of paricles that are withing distAng (1) (30 degrees)
        
%         meanPosX = mean(nonzeros(partPos(1,:).*(dist(1,:)<mindist)));
%         meanPosY = mean(nonzeros(partPos(2,:).*(dist(2,:)<mindist)));
%         meanAng = mean(nonzeros(partPos(3,:).*(dist(3,:)<minrad)));
        
        if (distX>(num*0.85) && distY>(num*0.85) && distAng>(num*0.85))
            %     if (distX>(num*0.8) && distY>(num*0.8))
            converged = 1
            
            if steps == 0
                %% Create a graph inside the map which contains the target
                
                maxX = max(map(:,1));
                maxY = max(map(:,2));
                minX = min(map(:,1));
                minY = min(map(:,2));
                
                start = [meanPosX, meanPosY]; 
                randCord(:,1)=[meanPosX,meanPosY]; %First element will be the place we think we are at
                randCord(:,2)=target; %Second element will be the target the rest random points
                
                for i=3:nrGraph
                    randCord(:,i)= [randi([minX,maxX]) randi([minY, maxY])] ;
                    pathBot.setBotPos(randCord(:,i));
                    if pathBot.insideMap() == 0 % checking if point is inside the map
                        % if not, set values to [-1 -1]
                        randCord(:,i) = [-1 -1];
                    end
                end
                % Create graph with edges
                l = 0;
                %         edges = zeros(5,nrGraph);
                mapShape = size(map);
                
                %             For each random point check connections with every other point
                %             if both are inside the map, check if there is an edge
                %             (straight path from A to B) by lookint wether the line
                %             between them intersects with any of the borders. I should
                %             only do this once
                for j = 1:nrGraph
                    for k = j+1: nrGraph
                        if (randCord(:,j) ~= [-1 -1]) & (randCord(:,k) ~= [-1 -1])
                            crashAndBurn = 0;
                            for m = 1:mapShape(1)
                                if m == mapShape(1)
                                    borderX=[randCord(1,j) randCord(1,k) map(m,1) map(1,1)];
                                    borderY=[randCord(2,j) randCord(2,k) map(m,2) map(2,1)];
                                else
                                    borderX=[randCord(1,j) randCord(1,k) map(m,1) map(m+1,1)];
                                    borderY=[randCord(2,j) randCord(2,k) map(m,2) map(m+1,2)];
                                end
                                inter  = intersect(borderX, borderY);
                                if inter == 1
                                    crashAndBurn = 1;
                                    break
                                end
                            end
                            if ~crashAndBurn
                                l = l+1;
                                %edges(:,l)= [randCord(1,j),randCord(2,j),randCord(1,k),randCord(2,k), pdist([randCord(:,j) randCord(:,k)],'euclidean')];
                                edges(:,:,l) = [[randCord(1,j),randCord(2,j)];[randCord(1,k),randCord(2,k)]];
                                
                            end
                        end
                    end
                end
            end
            
            steps = steps+1;
        end
        % 	get the mean of the locations and the turning point and assign it to
        % 	the main bot and suppose it is there if k percent of bots is there, it
        % 	converged
        estBot.setBotPos([meanPosX,meanPosY])
        estBot.setBotAng(meanAng)
        
        %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
        perc = 0.05*num;
        for i=1:perc
            particles(randi(num)).randomPose(0);
        end
        
        
        %% Write code to decide how to move next
        
        % if it is not converged, just move around a bit to find the bot
        if converged == 0
            turn = 0.5;
            move = 1;
            botSim.turn(turn); %turn the real robot.
            botSim.move(move); %move the real robot. These movements are recorded for marking
            estBot.turn(turn); %turn the estimated robot.
            estBot.move(move); %move the estimated robot.
            for i =1:num %for all the particles.
                particles(i).turn(turn); %turn the particle in the same way as the real robot
                particles(i).move(move); %move the particle in the same way as the real robot
            end
        end
        
    end
    
        %% Write path finding algorithm to decide how to move
        
        connected = isconnected(edges,start,target);
        
        if connected == 1
%             slope = (target(2) - start(2)) / (target(1) - start(1));
%             angle = atan(slope);
            start = estBot.getBotPos();
%             estBot.setBotAng(botSim.getBotAng());
            dir_bot = estBot.getBotAng();
%             dir_line = (atan2d(target(2) - start(2), target(1) - start(1))) * pi /180;
            dir_line = (atan2d(start(2) - target(2), start(1) - target(1))) * pi /180;
            turn =pi - dir_bot + dir_line;          
%             turn = estBot.getBotAng()-angle;
            path = [start;target];
%             move = 5;
            move = pdist([target; start],'euclidean');
        else
            % Find shortest path
            message = "NO PATH"
            %       My issue is, if we do A* I shouldn't resample the particles, which might be a good idea
            %       since it takes time to build the database and everything. I would
            %       have the node tho, because that will be the new mean(we have to
            %       update it when we move). But if I redo the random points I will
            %       need simple greedy, not A*. I could just say, which point takes me
            %       closest to target, but that's kinda stupid.. So no re-doing the
            %       graph. Cool. I shall do this tomorrow.
            %             findNextBest;
        end
        
        
        if estBot.getBotPos() == target
            notThere = 1;
            break
        end
        
        %% Drawing
        %only draw if you are in debug mode or it will be slow during marking
%         pred = BotSim(modifiedMap);
%         pred.setScanConfig(generateScanConfig(pred,6));
        
        if botSim.debug()
            hold off; %the drawMap() function will clear the drawing when hold is off
            botSim.drawMap();%drawMap() turns hold back on again -> you can draw the bots
            %         botSim.drawBot(30,'g'); %draw robot with line length 30 and green
            %         for i =1:num
            %             particles(i).drawBot(3); %draw particle with line length 3
            %         end
            if converged == 1
                %                 for i = 1:l
                %                     plot(edges(:,1,i) , edges(:,2,i),'-co')
                %                 end
                %             for i=1:size(path,1)-1
                %                 plot(path(:,i),path(:,i+1),'-ro')
                %             end
                
                for i = 1:size(path,2)-1
                    plot(path(:,i),path(:,i+1),'-ro')
                end
                plot(start(1,1),start(1,2), 'r+')
                plot(target(1,1),target(1,2), 'g+')
                for i = 3:nrGraph
                    if randCord(:,i) ~= [-1 -1]
                        plot(randCord(1,i), randCord(2,i), 'b*')
                    end
                end
            end
            
            botSim.drawBot(30,'g'); %draw robot with line length 30 and green
            estBot.drawBot(5,'c');
%             pred.setBotPos([meanPosX, meanPosY]);
%             pred.setBotAng(meanAng);
%             pred.drawBot(15);
            drawnow;
        end
        
        botSim.turn(turn); %turn the real robot.
%         botSim.drawBot(5,'c');
        drawnow;
        botSim.move(move); %move the real robot. These movements are recorded for marking
        estBot.turn(turn);
        estBot.move(move);
        estBot.drawBot(5,'b');
        botSim.drawBot(5,'m');
        drawnow;
        for i =1:num %for all the particles.
            particles(i).turn(turn); %turn the particle in the same way as the real robot
            particles(i).move(move); %move the particle in the same way as the real robot
            
        end
    break
    end
    
end

%%

    function line = intersect(borderX, borderY)
        %     borderX=[x1 x2 x3 x4];
        %     borderY=[y1 y2 y3 y4];
        deter1=det([1,1,1;borderX(1),borderX(2),borderX(3);borderY(1),borderY(2),borderY(3)])*det([1,1,1;borderX(1),borderX(2),borderX(4);borderY(1),borderY(2),borderY(4)]);
        deter2=det([1,1,1;borderX(1),borderX(3),borderX(4);borderY(1),borderY(3),borderY(4)])*det([1,1,1;borderX(2),borderX(3),borderX(4);borderY(2),borderY(3),borderY(4)]);
        
        if(deter1<=0 & deter2<=0)
            line=1;         %If lines intesect
        else
            line=0;
        end
    end

    function connected = isconnected(edges, node1, node2)
        i=1;
        connected = 0;
        while ( connected == 0 && i<size(edges,3))
            if ismember(edges(:,:,i),[node1;node2])
                connected = 1;
                break
            elseif ismember(edges(:,:,i),[node2;node1])
                connected = 1;
                break
            end
            i=i+1;
        end
        % i = 1;
        % while ( connected == 0 && i<size(edges,3))
        %
        %
        %     if ((edges(1,:,i) == node1) & (edges(2,:,i) == node2))
        %         connected = 1;
        %     elseif ((edges(1,:,i) == node2) & (edges(2,:,i) == node1))
        %         connected = 1;
        %     end
        %     i = i+1;
        % end
    end

% function connected = isconnected(edges,node)
%     j = 0;
% %     connected = WTF?????
%     for i=1:size(edges,2)
%         m = ismember(edges(1:2,:),node);
%         n = ismember(edges(3:4,:),node);
%         if m
%             j = j+1;
%             connected(j)= edges(3:4,i);
%         elseif n
%             j = j+1;
%             connected(j) = edges(1:2,i);
%         end
%     end
% end

% do average only in the area where the density is the highest not the
% whole map
%