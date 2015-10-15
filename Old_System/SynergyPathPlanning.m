%% Information
%
% Created by Thomas Colestock
% Florida Atlantic University - Robotics Lab
% 08/19/2015
% ver 1.3
% Contact: tcolesto@fau.edu
% Synergy Path Planning Matlab script
%
% Purpose:
% Use recorded joint angles from positioning robots in specific poses.
% Import the joint values as a structure to run through joint angle planning using
% acceleration constraints. Use normalized time values between 0-1 for each step between states.
%
% Pseudo code:
% Run [a,h] = AHStructBuild(fileName)
% Run path planning algorithms
% -- ask for time between states (0-1)
% -- display states that are being path planned for
% 3D matrix holding planned joint angles:
% -- rows are each joint hand=24; arm=6
% -- columns for each time slice along planned path
% ---- column 1 = start time; last column = final time for state
% -- pages correspond to the path between each state
% ---- For 4 states (Home, move1, move2, move3) there will be 3 pages
%       pages = #states - 1;
% ---- should ask the user if a return to state1 (typically 'Home') is
%       desired. will add one extra page to matrix. pages = #states
% Create a polyfit over the data in the 3D matrix. One for each row in each
%   page.
% Display and print results in some logical manner.
% Create a python file that can be executed to move the arm/hand.
%
% Instructions:
% Run the script and either add to path or change to folder
% When prompted to specify input file, type in the name of the file created when recording
%     the joint angles. Should have been done by running "sr_right_print_joints_position_TC.py".
% When prompted to specify output file, write the name you wish the python code that will
%     execute the movements to be called. ex: BottleGrab1
% The outputed python file will be located in the "Output" folder within the
%     "Synergy Path Planning" folder. Make sure there is no other file with the same name
%     in the Output folder or it will be appended to. Or, remove the files when the script is complete.
% When inputting the time, remember that it is on a normalized scale from 0-1.
%     For complex movements, each path segment may only need a fraction of 'time'.
%     ex: for a 4 step path to avoid an obstacle, specify 0.25 for each movemnt.
%     The script will prevent you from exceeding a max normalized time of 1.
%
% =========================================================================

%% Clear the workspace and collect manually overridden user variables
clear all; clc; close all;

% User editable variables:::
% ==========================
polyOrder = 3; % Order of the polynomial to fit to the data (recommended to keep at 3)

slices = 100; % # of time slices to use in the linspace of the time array (recommended at 100)
% ==========================


%% Handle directory and paths

HomePath = pwd; % save current working directory to return to it after script and printing finish
cd(fileparts(which('SynergyPathPlanning.m'))); % Move to proper folder if not already there
addpath('./Output'); % Make sure the Output, Input, Templates folders on on the path
addpath('./Input');
addpath('./Templates');


%% Collect interactive user input for file information

% Request Input and Output file information
inputFile = input('Specify name of input file without suffix(.txt); Remember "_MAT": \n >>> ','s');%'DrETest_MAT.txt';
inputFile = ['./Input/',inputFile,'.txt']; % Make the inputFile point to the proper folder with the proper suffix

% Check to see if input file already exists, request new file if false
existIn = which(inputFile);
while isempty(existIn)
    fprintf(2,'The file %s does not exist! \n',inputFile);
    inputFile = input('Please specify the correct input file, without suffix (.txt); Remember "_MAT": \n >>> ','s');
    inputFile = ['./Input/',inputFile,'.txt']; % Make the outputFile point to the proper folder with the proper suffix
    existIn = which(inputFile);
end


outputFile = input('Specify name of python file to input results to, without suffix (.py): \n >>> ','s');
outputFile = ['./Output/',outputFile,'.py']; % Make the outputFile point to the proper folder with the proper suffix

% Check to see if output file already exists, request overwrite if true
exist = which(outputFile);
while ~isempty(exist)
    fprintf(2,'The file %s already exists! \n', outputFile);
    overwrite = input('Would you like to overwrite it? [y/n] \n >>> : ','s');
    if overwrite == 'n'
        outputFile = input('Specify name of python file to output results to, without suffix (.py): \n >>> ','s');
        outputFile = ['./Output/',outputFile,'.py']; % Make the outputFile point to the proper folder with the proper suffix
        exist = which(outputFile);
    elseif overwrite == 'y'
        break;
    else
        fprintf(2,'You did not enter an appropriate response!\n');
    end
end

% Request if it is desired to return to the "Home" position
to_repeat = input('Would you like the Synergy to return to the initial position \nafter the last state has completed? [y/n]\n >>> ','s');
while not (to_repeat == 'y') | (to_repeat == 'n')
    fprintf(2,'You did not enter a proper response!\n');
    to_repeat = input('Would you like the Synergy to return to the initial position \nafter the last state has completed? [y/n]\n >>> ','s');
end


%% Build Structure and supporting variables

% Build the arm(a) and hand(h) structures based on the specified inputFile
[a, h] = AHStructBuild(inputFile);
fieldNames = fieldnames(a); % Find the field names of the structure (these are the states)

% Determine how many paths will be executed and create the required pages
if to_repeat == 'y'
    pages = length(fieldNames)-1;
    fieldNames(length(fieldNames)+1) = fieldNames(2); % fieldNames(1) is 'Joints'
else
    pages = length(fieldNames) - 2;
end

% Create matrices to preallocate memory for speed
qa = zeros(6,slices,pages); % 3D joint position matrix for the arm
qh = zeros(24,slices,pages); % 3D joint position matrix for the hand

t_fVect = zeros(pages,2); % 2 collumn vector to hold the [start time, end time] of the paths
t = zeros(pages,slices); % time matrix, rows == paths, collumns == slices


%% Solve for the joint positions along path

% Iterate through the number of paths
for i = 1:pages
    % Request time to execute path from user
    fprintf('To transition from the "%s" state to the "%s" state \n', fieldNames{i+1}, fieldNames{i+2});
    if i ~= pages % if it is not the last path
        t_f = input('How much time should it take? [0-1] \n >>> '); % user can enter any appropriate numerical value for this path
    else
        fprintf('Is the last move, a normalized time will be entered to finish on 1\n'); %Ensure to finish on 1 if it is the last path
    end
    
    
    % Make sure there wasn't an empty response
    while (isempty(t_f)) || (~isnumeric(t_f)) || (numel(t_f)>1)
        %     while isempty(t_f)
        if isempty(t_f)
            fprintf(2,'You must enter a numerical value! \nEmpty ("Enter") responses are not allowed!\n');
            fprintf('To transition from the "%s" state to the "%s" state \n', fieldNames{i+1}, fieldNames{i+2});
            t_f = input('How much time should it take? [0-1] \n >>> '); % user can enter any appropriate numerical value for this path
        end
        
        % Make sure the response was numeric
        %     while ~isnumeric(t_f)
        if ~isnumeric(t_f)
            fprintf(2,'You must enter a numerical value! [0-1]. \nNon-numeric responses are not allowed!\n');
            fprintf('To transition from the "%s" state to the "%s" state \n', fieldNames{i+1}, fieldNames{i+2});
            t_f = input('How much time should it take? [0-1] \n >>> '); % user can enter any appropriate numerical value for this path
        end
        
        if numel(t_f) > 1
            fprintf(2,'You must enter a single numerical value! [0-1]. \nMultiple, or Array responses are not allowed!\n');
            fprintf('To transition from the "%s" state to the "%s" state \n', fieldNames{i+1}, fieldNames{i+2});
            t_f = input('How much time should it take? [0-1] \n >>> '); % user can enter any appropriate numerical value for this path
        end
    end
    
    
    %     Structure t_fVect properly to turn current path durration time into
    %     total time for the path group
    if i == 1                           % if it is the first path
        t_fVect(i,2) = t_f;               % the duration is already relative to 0.0
    elseif i == pages                   % if this is the last path
        if t_fVect (i-1,2) ~= 1           % if the previous path did not end on a 1
            t_fVect(i,1) = t_fVect(i-1,2);  % the start of this path picks up where the previous path left off
            t_fVect(i,2) = 1;               % and because it is the last path, it must finish on 1
        else                              % if the previous path did end on a 1
            t_fVect(i,1) = 0;               % this path starts at 0
            t_fVect(i,2) = 1;               % again, we are dealing witht he last path, so must end on 1
        end
        t_f = 1-t_fVect(i,1);             % because the last path does not request an input, assign value to t_f
    else                                % if this is not the first or last path
        if t_fVect(i-1,2) ~= 1            % previous path did not end on 1
            t_fVect(i,1) = t_fVect(i-1,2);  % so pick up where it left off
            t_fVect(i,2) = t_fVect(i,1)+t_f;% and end on this path's durration + where last path finished
        else                              % if last path did end on 1
            t_fVect(i,1) = 0;               % start this path at 0
            t_fVect(i,2) = t_f;             % end this path at its duration value
        end
    end                                 % end t_fVect structuring
    
    % If the normalized end time of the current path is greater than 1
    while t_fVect(i,2) > 1
        fprintf(2,'The normalized time of the recent group of movements is greater that 1!\n');
        fprintf(2,'The maximum time the current action can take is %f seconds.\n',(1-t_fVect(i,1)));
        t_f = input('How much time should it take? [0-1] \n >>> ');    % request for a new time durration
        t_fVect(i,2) = t_fVect(i,1)+t_f;            % assign value based on new durration
    end
    
    
    % Assign theta values based on imported joint data for this path
    a_thetaOld = [a(:).(fieldNames{i+1})]; % a(all the values)'in(.)'(fieldName{cell #i+1})
    a_thetaNew = [a(:).(fieldNames{i+2})];
    h_thetaOld = [h(:).(fieldNames{i+1})];
    h_thetaNew = [h(:).(fieldNames{i+2})];
    
    % need to find minimum acceleration to avoid complex instances
    a_qdd_min = 4*abs(a_thetaNew-a_thetaOld)/t_f^2;
    h_qdd_min = 4*abs(h_thetaNew-h_thetaOld)/t_f^2;
    
    % Find the initial sign of the acceleration
    a_Sqddc = sign(a_thetaNew - a_thetaOld).*a_qdd_min; % 'Signed q double dot joint 1'
    h_Sqddc = sign(h_thetaNew - h_thetaOld).*h_qdd_min;
    
    % Create time vector in the proper row of the time matrix (row==path#)
    t(i,:) = linspace(0,t_f,slices); % when solving for q, always start time at 0 for eqns to work
    Lt = length(t);                 % record the number of collumns in the t matrix (should = slices)
    
    % Solve for q twice using the before cruise time eqns, and after cruise
    % time eqns.
    % Due to this script operating at the minimum acceleration there will
    % be no cruise section of the joint results.
    % Arm:
    qa_seg1 = a_thetaOld'*ones(1,Lt) + (1/2)*a_Sqddc'*t(i,:).^2; % solve for position w/ eqn for before cruise time
    qa_seg2 = a_thetaNew'*ones(1,Lt) - (1/2)*a_Sqddc'*(t_f-t(i,:)).^2; % for after cruise time
    % Hand:
    qh_seg1 = h_thetaOld'*ones(1,Lt) + (1/2)*h_Sqddc'*t(i,:).^2;
    qh_seg2 = h_thetaNew'*ones(1,Lt) - (1/2)*h_Sqddc'*(t_f-t(i,:)).^2;
    
    % Now combine the two q segments into one final q matrix
    qa_mid = ceil(Lt/2); % the transition point will be the middle of the length of the collumns
    % Arm:
    qa(:,1:qa_mid,i)=qa_seg1(:,1:qa_mid);
    qa(:,qa_mid+1:end,i) = qa_seg2(:,qa_mid+1:end);
    % Hand:
    qh(:,1:qa_mid,i)=qh_seg1(:,1:qa_mid);
    qh(:,qa_mid+1:end,i) = qh_seg2(:,qa_mid+1:end);
    % qa and qh are the final 3D matrices that hold the joint position
    % values for every joint for each path.
    % Of size (#joints, #slices, #paths)
    
end % end the iterating for loop

%% Perform the Polyfit

% Pre-allocate for speed
Pa = zeros(size(qa,1),polyOrder+1,pages); % Polynomial coef matrix for the arm
Ph = zeros(size(qh,1),polyOrder+1, pages); % Polynomial coef matrix for the hand

% Iterate through the path pages and solve the polyfit for each row
for i = 1: pages
    t(i,:) = linspace(t_fVect(i,1),t_fVect(i,2),slices); % this time matrix does not start each row at 0, needed for proper polyfit coef's
    for j = 1:size(qa,1)
        Pa(j,:,i) = polyfit(t(i,:),qa(j,:,i),polyOrder);
    end
    for j = 1:size(qh,1)
        Ph(j,:,i) = polyfit(t(i,:),qh(j,:,i),polyOrder);
    end
end

%% Print to the Command Window and Output Python File

to_display = input('\nWould you like to display results in this terminal? [y/n] \n >>> ','s');

% Copy Template 1 to the Output file
copyfile('./Templates/TemplatePot1.py', outputFile);
fOut = fopen(outputFile,'a');

if to_display == 'y'
    f = {fOut,1};
else
    f = {fOut};
end

% Names of all the hand joints, in logical order that matches python output
hand_joint_names = {'rh_FFJ1','rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4',...
    'rh_MFJ1','rh_MFJ2','rh_MFJ3','rh_MFJ4',...
    'rh_RFJ1','rh_RFJ2','rh_RFJ3','rh_RFJ4',...
    'rh_LFJ1','rh_LFJ2','rh_LFJ3','rh_LFJ4','rh_LFJ5',...
    'rh_THJ1','rh_THJ2','rh_THJ3','rh_THJ4','rh_THJ5',...
    'rh_WRJ1','rh_WRJ2'};

% Names of all the arm joints, in logical order that matches python output
arm_joint_names = {'ra_shoulder_pan_joint', 'ra_shoulder_lift_joint',...
    'ra_elbow_joint','ra_wrist_1_joint','ra_wrist_2_joint','ra_wrist_3_joint'};

% Cycle through printing all of the necissary information:
for j = 1:length(f)
    if j == 1
        fprintf('Writing results to: %s. \n',outputFile);
        pause(1);
    end
    %-----------------------------------------------------------
    
    % Print the arm and hand polynomial dictionaries
    fprintf(f{j},'# Dictionaries containing joint name keys, and poly-coef value vectors \n');
    for i = 1:pages
        PaTran = Pa(:,:,i)';
        c = [arm_joint_names; num2cell(PaTran)];
        fprintf(f{j},'%s_to_%s_Arm = {',fieldNames{i+1},fieldNames{i+2});
        fprintf(f{j},'"%s" : [ %f, %f, %f, %f ], ',c{:});
        fprintf(f{j},'};\n\n');
        
        PhTran = Ph(:,:,i)';
        cc = [hand_joint_names; num2cell(PhTran)];
        fprintf(f{j},'%s_to_%s_Hand = {',fieldNames{i+1},fieldNames{i+2});
        fprintf(f{j},'"%s" : [ %f, %f, %f, %f ], ',cc{:});
        fprintf(f{j},'};\n\n');
                
    end
    %--------------------------------------------------------------
    
    % Print list of arm, hand, and general movement names
    fprintf(f{j},'# List of movement names for Arm/Hand \n');
    % Arm
    fprintf(f{j},'movements_Arm = [');
    for i = 1:pages
        fprintf(f{j},'%s_to_%s_Arm,',fieldNames{i+1},fieldNames{i+2});
    end
    fprintf(f{j},'];\n');
    % Hand
    fprintf(f{j},'movements_Hand = [');
    for i = 1:pages
        fprintf(f{j},'%s_to_%s_Hand,',fieldNames{i+1},fieldNames{i+2});
    end
    fprintf(f{j},'];\n');
    % General
    fprintf(f{j},'movements = [');
    for i = 1:pages
        fprintf(f{j},'"%s_to_%s",',fieldNames{i+1},fieldNames{i+2});
    end
    fprintf(f{j},'];\n\n');
    %---------------------------------------------------------------
    
    % Print list of start-end time of movements
    fprintf(f{j},'# A time list to possibly reference the time values by iteration\n');
    fprintf(f{j},'times = [');
    for i = 1:length(t_fVect)
        fprintf(f{j},'[%f,%f],',t_fVect(i,1),t_fVect(i,2));
    end
    fprintf(f{j},']\n\n');
    %---------------------------------------------------------------
    
    % Print the starting positions of the arm and hand 
    fprintf(f{j},'# Start positions for the Arm and Hand\n');
    % Hand
    fprintf(f{j},'hand_start = {');
    for i = 1:24
        fprintf(f{j},'"%s":%f, ',h(i).Joints, h(i).Home);
    end
    fprintf(f{j},'}\n');
    %Arm
    fprintf(f{j},'arm_start = {');
    for i = 1:6
        fprintf(f{j},'"%s":%f, ',a(i).Joints, a(i).Home);
    end
    fprintf(f{j},'}\n\n');
    %---------------------------------------------------------------
    
    if j == 1
        % Read in Template 2 and write its contents to the Output file
        str_buf = fileread('./Templates/TemplatePot2.py');
        fwrite(fOut,str_buf);
        fclose(fOut); % close the output file
        fprintf('Writting complete!\n');
        pause(1)
        if to_display == 'y'
            fprintf('Printing results to the terminal: \n\n');
            pause(2)
        end
    end
        
end % end for j = 1:length(f) --file iteration


%% Clean up changes to the path and directory

rmpath('./Output');
rmpath('./Input');
rmpath('./Templates');
cd(HomePath);



