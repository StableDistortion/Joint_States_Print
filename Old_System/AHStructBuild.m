function [armStruct, handStruct] = AHStructBuild(FileID)

%==========================================================
% Add instructions here.....
% Created by Thomas Colestock
% Florida Atlantic University - Robotics Lab
% 08/18/2015
% ver 1.0
% Contact: tcolesto@fau.edu
% Function to build two structures [armStruct, handStruct] based on inputs
% from a file. File input is the joint printouts of the Shadow Hand and
% UR10 robots, using the sr_right_print_joints_position_TCtest2.py program.
%
%==========================================================


% Import and read the file to a string buffer
str_buf = fileread(FileID);

% Find the states specified by the user, will become the structure fields
% The lists will be referenced to place the cursor to read the state/field
% names
fields_list = strfind(str_buf, '<'); % Field names start with a '<'
end_fields_list = strfind(str_buf, '>'); % End with a '<'

% Determine how many blocks of values there are
n_blocks = numel(fields_list);

% Create an array that holds the names of the states/fields
for i = 1: n_blocks
    field_line = fields_list(i); % location of the ith occurance of '<', also ith field/state
    end_field_line = end_fields_list(i); 
    
    field_buf = str_buf(field_line +1 : end_field_line -1); % assign characters between '<' and '>'
    
    field_cell(i) = textscan(field_buf, '%s'); % store in a cell array
    
    DesiredFields(i) = field_cell{i}; % take from cell array and input into an array of strings
    
end

% Names of all the hand joints, in logical order that matches python output
hand_joint_names = {'rh_FFJ1','rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4',...
    'rh_MFJ1','rh_MFJ2','rh_MFJ3','rh_MFJ4',...
    'rh_RFJ1','rh_RFJ2','rh_RFJ3','rh_RFJ4',...
    'rh_LFJ1','rh_LFJ2','rh_LFJ3','rh_LFJ4','rh_LFJ5'...
    'rh_THJ1','rh_THJ2','rh_THJ3','rh_THJ4','rh_THJ5'...
    'rh_WRJ1','rh_WRJ2'};

% Names of all the arm joints, in logical order that matches python output
arm_joint_names = {'ra_shoulder_pan', 'ra_shoulder_lift',...
    'ra_elbow','ra_wrist_1','ra_wrist_2','ra_wrist_3'};

% Create structure with initial 'Joints' field and assign joint names as
% cell values
handStruct = struct('Joints', cell(hand_joint_names));
armStruct = struct('Joints', cell(arm_joint_names));

% Create structure fields from joint states with empty cells
for i = 1:numel(DesiredFields)
    handStruct(1).(DesiredFields{i})=[];
    armStruct(1).(DesiredFields{i})=[];
end

% Search for the locations in the file where these strings are found
hand_list = strfind(str_buf, 'Hand joints position'); % beginning of hand joint values block
heob_list = strfind(str_buf, 'HEOB'); % end of hand joint values block
arm_list = strfind(str_buf, 'Arm joints position'); % beginning of arm joint values block
aeob_list = strfind(str_buf, 'AEOB'); % end of arm joing values block

% Find, import, and assign joint values for arm and hand for every field
for i = 1 : n_blocks
    hand_line = hand_list(i); % locate beginning of ith block of hand joints
    heob_line = heob_list(i); % locate end of ith block of hand joints
    arm_line = arm_list(i);   % locate beginning of ith block of arm joints
    aeob_line = aeob_list(i); % locate end of ith block of arm joints
    
    hand_buf = str_buf(hand_line : heob_line -1); % hold buffer for whats between start and end of hand block
    arm_buf = str_buf(arm_line : aeob_line -1); % hold buffer for whats between start and end of arm block
    
    hand_cell = textscan(hand_buf, '%f', 'HeaderLines',1); % create cell array importing values a floats, and ingnoring headerline 
    arm_cell = textscan(arm_buf, '%f', 'HeaderLines',1);
    
    hand_vect = cell2mat(hand_cell); % convert cell array to a matrix
    arm_vect = cell2mat(arm_cell);
    
    % Assign the joint values to the proper cells in the structures
    for j = 1:length(hand_vect)
        handStruct(j).(DesiredFields{i}) = hand_vect(j);
    end
    
    for j = 1:length(arm_vect)
        armStruct(j).(DesiredFields{i}) = arm_vect(j);
    end
end









