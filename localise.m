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

% ms = struct( 'pos',{},'ang',{},'distance',{},'crossingPoint',{}) %% Struct 
% containoing measurements obtained from particles

particles(num,1) = BotSim; %how to set up a vector of objects
resPart = zeros(num, 3); % position(1,2), angle(3)
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
    
    distX = sum(dist(1,:)<2)
    distY = sum(dist(2,:)<2)
    distAng = sum(dist(1,:)<1)
    
    if (distX>(num*0.9) && distY>(num*0.9) && distAng>(num*0.9))
%     if (distX>(num*0.8) && distY>(num*0.8))
        converged = 1
    end
% 	get the mean of the locations and the turning point and assign it to
% 	the main bot and suppose it is there if k percent of bots is there, it
% 	converged

    %% Write code to take a percentage of your particles and respawn in 
    %  randomised locations (important for robustness)
    perc = 0.10*num
    for i=1:perc
        particles(randi(num)).randomPose(0);
    end    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
%     do A* and make it move :D
    
    for i=1:perc
        findpath(i)=randomPose(0);
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
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap();%drawMap() turns hold back on again -> you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3
        end
        drawnow;
    end
end
end
