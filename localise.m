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

%% Localisation code
maxNumOfIterations = 60;
n = 0;
converged =0; %The filter has not converged yet
var = 10; % variance for Gussian
sqrt2PiVar = sqrt(2*pi*var);
damping = 0.000000001; %damping factor

botSim.setScanConfig(botSim.generateScanConfig(6)); 
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(5);
partWeight = zeros(num,1);
% [botSdist botScross] = botSim.ultraScan() ;
%fprintf(' BotSim distance = %d BotSim crossingPoint = %d\n', botSdist, botScross);  

while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    BScan = botSim.ultraScan() %get a scan from the real robot.

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
%     disp( max(partWeight))
%     disp(min(partWeight))
    
    allweights = sum(partWeight);
    for i = 1:num
        partWeight(i) = partWeight(i)/allweights;
    end

    %% Write code for resampling your particles
    cumulativeSum = cumsum(partWeight);
    
    for i = 1:num
        partic = find(rand() <= cumulativeSum,1);
        resPart(i,1:2) = particles(partic).getBotPos();
        resPart(i, 3) = particles(partic).getBotAng();
    end

    for i = 1:num
        particles(i).setBotPos([resPart(i,1), resPart(i,2)]);
        particles(i).setBotAng(resPart(i,3));
        turn = randi(2);
        % Might be worth
%       looking into how to get direction in +- 0.5 or something (it's a 
%       circle tho, so that's kinda a problem because it doesn't understand 
%       negative)
%         bitmin = resPart(i,3)-0.2;
%         bitmax = resPart(i,3)+0.2;
%         
%         if (bitmin<0)
%             bitmin = 2 + bitmin;
%         end
%         if (bitmax> 2)
%             bitmax = resPart(i,3)+0.2 - 2;
%         end
%         bitmin
%         bitmax
%         if bitmin<bitmax    
%             turn = randi([bitmin bitmax]);
%         else 
%             turn = randi([bitmax bitmin]);
%         end

        move = randi(1);
        particles(i).turn(turn);  
        particles(i).move(move);
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
    
    distX = sum(dist(1,:)<2);
    distY = sum(dist(2,:)<2);
    distAng = sum(dist(1,:)<1);
    
    if (distX>(num*0.85) && distY>(num*0.85) && distAng>(num*0.85))
%     if (distX>(num*0.8) && distY>(num*0.8))
        converged = 1
    end
% 	get the mean of the locations and the turning point and assign it to
% 	the main bot and suppose it is there if k percent of bots is there, it
% 	converged

    %% Write code to take a percentage of your particles and respawn in 
    %  randomised locations (important for robustness)
    perc = 0.10*num;
    for i=1:perc
        particles(randi(num)).randomPose(0);
    end    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    %     do A* and make it move :D
    % I am going to create 100 random particles and exclude the ones that 
    % are outside the mapthan create a fully connected graph and find the 
    % shortest path from start to end
    if converged == 1

        maxX = max(map(:,1));
        maxY = max(map(:,2));
        minX = min(map(:,1));
        minY = min(map(:,2));  

        start = [meanPosX, meanPosY]; % for now we leave the starting pos at this,
                                      % later set it to mean of closest
                                      % particles

        for i=1:nrGraph
            randCord(:,i)= [randi([minX,maxX]) randi([minY, maxY])] ;
            pathBot.setBotPos(randCord(:,i)); 
            if pathBot.insideMap() == 0 % checking if point is inside the map 
                                        % if not, set values to [-1 -1]
                randCord(:,i) = [-1 -1];
            end  
        end
        % Create graph with edges
        l = 0;
        edges = zeros(5,nrGraph);
        mapShape = size(map)
        for j = 1:nrGraph
            for k = j: nrGraph
                if (randCord(:,j) ~= [-1 -1]) 
                    if (randCord(:,k) ~= [-1 -1])
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
                                crashAndBurn = 1
                                break
                            end
                        end
                        
                        if ~crashAndBurn
                            l = l+1;
                            edges(:,l)= [randCord(1,j),randCord(2,j),randCord(1,k),randCord(2,k), pdist([randCord(:,j) randCord(:,k)],'euclidean')] ;             
                        end
                    end
                end
            end
        end

    end 
    turn = 0.5;
    move = 2;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    pred = BotSim(modifiedMap);
    pred.setScanConfig(generateScanConfig(pred,6));
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap();%drawMap() turns hold back on again -> you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3
%         end
        if converged == 1
            
            for i = 1:nrGraph
                if randCord(:,i) ~= [-1 -1]
                    plot(randCord(1,i), randCord(2,i), 'b*')
                end
            end
            
            for i = 1:size(edges)
                plot([edges(1,i) edges(2,i)], [edges(3,i) edges(4,i)])
            end
        end
%         meanPosX = mean(partPos(1,:));
%         meanPosY = mean(partPos(2,:));
%         meanAng = mean(partPos(3,:));
        
        pred.setBotPos([meanPosX, meanPosY]);
        pred.setBotAng(meanAng);
        pred.drawBot(15);
        drawnow;
    end
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

% do average only in the area where the density is the highest not the
% whole map 
% 