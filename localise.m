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
nrGraph = 100;
particles(num,1) = BotSim; %how to set up a vector of objects

estBot = BotSim(modifiedMap); % estimated bot position
estBot.setScanConfig(generateScanConfig(estBot,6));

for i = 1:num
    particles(i) = BotSim(modifiedMap);  % each particle should use the same
    % map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(generateScanConfig(particles(i),6));
    partPos(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
%     particles(i).drawBot(3);
end

%% Iterate until you find the target

botSim.setScanConfig(botSim.generateScanConfig(6));
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(5);
partWeight = zeros(num,1);
notThere = 1;
steps = 0;
path = [];

while notThere == 1
    %% Localisation code
    
    [path, estBot,botSim,randCord,steps] = findLocation(map,steps,botSim,estBot,particles,partWeight,num,target,modifiedMap,nrGraph,path);
    
    
    %% Write path finding algorithm to decide how to move
    for i = 1:size(path)
        distToNode = distance(estBot.getBotPos(),path(i,:));
        if distToNode < 2
            next = path(i+1,:);
        end
    end
    start = estBot.getBotPos();
    dir_bot = estBot.getBotAng();
    dir_line = (atan2d(start(2) - next(2), start(1) - next(1))) * pi /180;
    turn =pi - dir_bot + dir_line;
    %         move = pdist([target; start],'euclidean');
%     move = distance(next, start);
    move = min(5,distance(next, start));
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    %         pred = BotSim(modifiedMap);
    %         pred.setScanConfig(generateScanConfig(pred,6));
    
    if botSim.debug()
%         hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap();%drawMap() turns hold back on again -> you can draw the bots
        
        for i = 1:size(path,2)-1
            plot(path(:,i),path(:,i+1),'-ro')
        end
        plot(start(1,1),start(1,2), 'r+')
        plot(target(1,1),target(1,2), 'g+')
%         for i = 3:nrGraph
%             if randCord(:,i) ~= [-1 -1]
%                 plot(randCord(1,i), randCord(2,i), 'b*')
%             end
%         end
        
%         botSim.drawBot(30,'g'); %draw robot with line length 30 and green
%         estBot.drawBot(5,'c');
%         drawnow;
    end
    
    botSim.turn(turn); %turn the real robot.
    %         botSim.drawBot(5,'c');
    botSim.move(move); %move the real robot. These movements are recorded for marking
    estBot.turn(turn);
    estBot.move(move);
    estBot.drawBot(5,'b');
    botSim.drawBot(5,'g');
    drawnow;
    for i =1:num %for all the particles.
        turnNoise = normrnd(turn,0.6);
        moveNoise = normrnd(move,0.6);
        particles(i).turn(turnNoise); %turn the particle in the same way as the real robot
        particles(i).move(moveNoise); %move the particle in the same way as the real robot
        
    end
    
    distToTarg = distance(estBot.getBotPos(),target);
    
    if distToTarg < 2
        notThere = 0;
    end
end

end

%%

function ang = setMeanAng(botSim,estBot)
ang = 0;
botScan = botSim.ultraScan();
minim = [10000 10000 10000 10000 10000 10000];
%     estScan = zeros(
for i=1:360
%     estBot.turn(deg2rad(1));
    estBot.setBotAng(i*pi/180);
    diff=abs(estBot.ultraScan()-botScan);
    if diff<minim
        minim = diff;
%         ang = deg2rad(i);
        ang = i*pi/180;
    end
end
end

function line = intersect(borderX, borderY)
%     borderX=[x1 x2 x3 x4];
%     borderY=[y1 y2 y3 y4];
deter1=det([1,1,1;borderX(1),borderX(2),borderX(3);borderY(1),borderY(2),borderY(3)])*det([1,1,1;borderX(1),borderX(2),borderX(4);borderY(1),borderY(2),borderY(4)]);
deter2=det([1,1,1;borderX(1),borderX(3),borderX(4);borderY(1),borderY(3),borderY(4)])*det([1,1,1;borderX(2),borderX(3),borderX(4);borderY(2),borderY(3),borderY(4)]);

if(deter1<=0 & deter2<=0)
    line=1;
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

function [realPath,cost] = dijkstra(start,target,randNodes)
i =1;
s =1;
t =2;

for j=1: size(randNodes,2)
    if randNodes(:,j) ~= [-1,-1]
        nodes(:,i) =randNodes(:,j);
        
        if randNodes(:,j) == start
            s = i;
        end
        if randNodes(:,j) == target
            t = i;
        end
        
        i = i+1;
    end
    
end
n=size(nodes,2);

S(1:n) = 0;
dist(i,:) = inf;
dist(s)=0;
prev(1:n) = n+1;

while sum(S)~=n %not all nodes were visited
    candidate=[];
    for i=1:n
        if S(i)==0
            candidate=[candidate, dist(i)];
        else
            candidate=[candidate, inf];
        end
    end
    [place, val]=min(candidate);
    S(val)=1;
    for i=1:n
        % there's no such thing as node J
        if(dist(val)+distance(nodes(:,i),nodes(:,j)))<dist(i)
            dist(i)=dist(val)+distance(nodes(:,i),nodes(:,j));
            prev(i)=val;
        end
        
    end
end

shortestpath = t;

while shortestpath(1) ~= s
%     if prev(shortestpath(1))<=n
        trg = target
        s = shortestpath(1)
        sp = nodes(:,shortestpath(1))
        pr  = prev(:)
        shortestpath=[prev(shortestpath(1)), shortestpath]
        
%     else
%         errorMessage = "--------There is no path ------!"
%     end
end

for i = 1:size(shortestpath)
    realPath(i,:)=nodes(:,shortestpath(i))
end

cost = dist(t);

end



















function [path, estBot,botSim,randCord,steps]= findLocation(map,steps,botSim,estBot,particles,partWeight,num,target,modifiedMap,nrGraph,path)
maxNumOfIterations = 60;
var = 5; % variance for Gussian
sqrt2PiVar = sqrt(2*pi*var);
damping = 0.000000001; %damping factor
% minrad = 0.523599;
mindist = 1;
converged =0; %The filter has not converged yet
n = 0;
randCord = zeros(2, nrGraph); %random coordonates on the map to find shortest path
resPart = zeros(num, 3); % position(1,2), angle(3)
pathBot = BotSim(modifiedMap);
pathBot.setScanConfig(generateScanConfig(pathBot,6));
dist = zeros(2, num);

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
    
%     if steps == 0
        for i = 1:num
            particles(i).setBotPos([resPart(i,1), resPart(i,2)]);
            particles(i).setBotAng(resPart(i,3));
%             turn = randi(2);
%             move = randi(1);
%             particles(i).turn(turn);
%             particles(i).move(move);
        end
%     end
    
    
    %% Write code to check for convergence
    for i = 1:num
        partPos(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
    end
    meanPosX = mean(partPos(1,:));
    meanPosY = mean(partPos(2,:));
    
    for i = 1:num
        dist(:,i) = [abs(partPos(1,i)-meanPosX) abs(partPos(2,i)-meanPosY)];
    end
    
    distX = sum(dist(1,:)<mindist); %nr of paricles that are withing distX (2)
    distY = sum(dist(2,:)<mindist); %nr of paricles that are withing distY (2)
    
    if (distX>(num*0.9) && distY>(num*0.9))
        %     if (distX>(num*0.8) && distY>(num*0.8))
        converged = 1;
        estBot.setBotPos([meanPosX,meanPosY]);
        meanAng = setMeanAng(botSim,estBot);
        estBot.setBotAng(meanAng);
        
        botSim.drawBot(5,'g');
        estBot.drawBot(5,'b');
        drawnow;
        
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
            mapShape = size(map);

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
            
            connected = isconnected(edges,start,target);
            % Find shortest path
            path = [];
            if connected
                path = [start;target];
            else
                [path,len] = dijkstra(start,target,randCord);
            end
            
            botSim.drawBot(30,'g');
            estBot.drawBot(30,'b');
            for i = 1:size(path,2)-1
                line(path(:,i),path(:,i+1));
                drawnow;
            end
            plot(target(1,1),target(1,2), 'r*')

            drawnow;
        end
        
        steps = steps+1;
    end
    
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    perc = 0.15*num;
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
end