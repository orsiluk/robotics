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
ms = zeros(3, num); 

% ms = struct( 'pos',{},'ang',{},'distance',{},'crossingPoint',{}) %% Struct 
% containoing measurements obtained from particles

particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  % each particle should use the same 
                                         % map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(generateScanConfig(particles(i),6));
    ms(:,i) = [particles(i).getBotPos() particles(i).getBotAng()];
    particles(i).drawBot(3);
end

% [botSdist botScross] = botSim.ultraScan() ;
% fprintf(' BotSim distance = %d BotSim crossingPoint = %d\n', botSdist, botScross);
% 
% for i = 1:num
%     [distance crossingPoint]  = particles(i).ultraScan();
%     ms(:,i) = [particles(i).getBotPos() particles(i).getBotAng() 
% distance crossingPoint];
% %     
% %     fprintf('%d --- pos = %d ang = %d\n',i, ms(1,i), ms(2,i));
%     particles(i).drawBot(3); %draw particle with line length 3 and default color
% end
%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged =0; %The filter has not converged yet
var = 15; % variance for Gussian
sqrt2PiVar = sqrt(2*pi*var);

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
            partWeight(i) = (1 /sqrt2PiVar) * exp(-(difference)^2 /denom)
        else
            partWeight(i) = 0.000001;
        end
    end
%     disp( max(partWeight))
%     disp(min(partWeight))
    
    allweights = sum(partWeight);
    for i = 1:num
        partWeight(i) = partWeight(i)/allweights;
    end

    %% Write code for resampling your particles
    cumulativeSum = cumsum(partWeight)
    
    for i = 1:num
        if (rand )
    end
    %% Write code to check for convergence   
	

    %% Write code to take a percentage of your particles and respawn in 
    %  randomised locations (important for robustness)	
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
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
