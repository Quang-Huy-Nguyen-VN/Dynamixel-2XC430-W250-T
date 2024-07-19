clc;
clear;

% Bibliothèque Dynamixel
lib_name = '';
if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Port settings
DEVICENAME = 'COM4';
BAUDRATE = 57600; 

% Addresses de Control table 
ADDR_PRO_TORQUE_ENABLE = 64;
ADDR_PRO_GOAL_POSITION = 116;
ADDR_PRO_PRESENT_POSITION = 132;
ADDR_OPERATING_MODE = 11;  % Extended Position Control Mode addresse

% Version de Protocol
PROTOCOL_VERSION = 2.0;

% Réglage par défaut
DXL_ID_1 = 1;
DXL_ID_2 = 2;
TORQUE_ENABLE = 1;
TORQUE_DISABLE = 0;
EXTENDED_POSITION_CONTROL_MODE = 4;  % Valeur de Extended Position Control Mode

% Initialiser PortHandler 
port_num = portHandler(DEVICENAME);
packetHandler();

% Ouvrir le port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    error('Failed to open the port!\n');
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    error('Failed to change the baudrate!\n');
end

% Set mode Extended Position Control pour les moteurs
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_OPERATING_MODE, EXTENDED_POSITION_CONTROL_MODE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= 0
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end
if dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

%{
% Moteur ID 2 si besoin
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_OPERATING_MODE, EXTENDED_POSITION_CONTROL_MODE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= 0
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end
if dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end
%}

% Enable Dynamixel Torque pour les moteurs ID 1
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
% Enable Dynamixel Torque pour les moteurs ID 2
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE); 

% Fonction pour convertir 'signed int' en 'unsigned int'
signed2unsigned = @(x) mod(x, 2^32);

% Définir la position actuelle
present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_PRESENT_POSITION);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= 0
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end
if dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

fprintf('Current Position of Motor: %d\n', present_position);

% Utiliser le moteur ID1
% Define positions based on the current position
goal_position_pos = present_position + 1860; % Move 2048 steps clockwise (180 degrees)
goal_position_neg = present_position - 1860; % Move 2048 steps counterclockwise (180 degrees)

% Get start time
start_time = tic;

% Loop for 1 minute
while toc(start_time) < 60
    % Move motor to goal_position_pos
    for i = present_position:96:goal_position_pos
        unsigned_position_1 = signed2unsigned(i);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position_1);
        pause(0.01);
    end

    % Move motor to goal_position_neg
    for i = goal_position_pos:-96:goal_position_neg
        unsigned_position_1 = signed2unsigned(i);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position_1);
        pause(0.01);
    end

    % Move motor back to present_position
    for i = goal_position_neg:96:present_position
        unsigned_position_1 = signed2unsigned(i);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position_1);
        pause(0.01);
    end
end

%{
% Utiliser le moteur ID2
% Define positions based on the current position
goal_position_pos = present_position + 2048; % Move 2048 steps clockwise (180 degrees)
goal_position_neg = present_position - 2048; % Move 2048 steps counterclockwise (180 degrees)

% Get start time
start_time = tic;

% Loop for 1 minute
while toc(start_time) < 60
    % Move motor to goal_position_pos
    for j = present_position:96:goal_position_pos
        unsigned_position_2 = signed2unsigned(j);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, unsigned_position_2);
        pause(0.01);
    end

    % Move motor to goal_position_neg
    for j = goal_position_pos:-96:goal_position_neg
        unsigned_position_2 = signed2unsigned(j);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, unsigned_position_2);
        pause(0.01);
    end

    % Move motor back to present_position
    for j = goal_position_neg:96:present_position
        unsigned_position_2 = signed2unsigned(j);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, unsigned_position_2);
        pause(0.01);
    end
end
%}

% Afficher la position actuelle des moteurs
present_position_1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_PRESENT_POSITION);
%present_position_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_PRESENT_POSITION);
fprintf('Present Position of Motor: %d\n', present_position);

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

% Fermer le port
closePort(port_num);
unloadlibrary(lib_name);