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

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_OPERATING_MODE, EXTENDED_POSITION_CONTROL_MODE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= 0
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
end
if dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Enable Dynamixel Torque pour les moteurs ID 1 and 2
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

% Fonction pour convertir 'signed int' en 'unsigned int'
signed2unsigned = @(x) mod(x, 2^32);

% Initialisation les positions
initial_position = 0;
goal_position_pos = 1860;
goal_position_neg = -1860;

%_________________________________________________________________________%
% 1ère mouvement 
% Mouvement du moteur ID1 de 0 à 'goal position' pas à pas 24
for i = initial_position:24:goal_position_pos
    unsigned_position = signed2unsigned(i);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position);

    pause(0.1);
end

%_________________________________________________________________________%
% 2ème mouvement : dans Part I
% Moteur ID1 
for i = goal_position_pos:-24:0
    unsigned_position_1 = signed2unsigned(i);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position_1);


    % Calcul la correspondant position de moteur ID2
    j = goal_position_pos - i;
    unsigned_position_2 = signed2unsigned(j);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, unsigned_position_2);


    pause(0.1);
end

%_________________________________________________________________________%
% 3e mouvement : dans Part II
% Moteur ID1 
for i = 0:-24:goal_position_neg
    unsigned_position_1 = signed2unsigned(i);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position_1);

    % Calcul la correspondant position de moteur ID2
    j = goal_position_pos + i;
    unsigned_position_2 = signed2unsigned(j);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, unsigned_position_2);

    pause(0.1);
end

%_________________________________________________________________________%
% 4e mouvement : dansPart III 
% Moteur ID2
for j = 0:-24:goal_position_neg
    unsigned_position_1 = signed2unsigned(j);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, unsigned_position_1);

    % Calcul la correspondant position de moteur ID1
    i = goal_position_neg - j;
    unsigned_position_2 = signed2unsigned(i);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position_2);

    pause(0.1);
end

%_________________________________________________________________________%
% 5e mouvement : dans Part IV
% Moteur ID2
for j = goal_position_neg:24:0
    unsigned_position_1 = signed2unsigned(j);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, unsigned_position_1);

    % Calcul la correspondant position de moteur ID1
    i = goal_position_pos + j;
    unsigned_position_2 = signed2unsigned(i);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position_2);

    pause(0.1);
end

%_________________________________________________________________________%
% Finalement  mouvement : Retourner l'initialisation
% Mouvement du moteur ID1 de 'goal position' à 0 pas à pas 24
for i = goal_position_pos:-24:0
    unsigned_position = signed2unsigned(i);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, unsigned_position);

    pause(0.1);
end
%_________________________________________________________________________%

% Afficher la position actuelle des moteurs
present_position_1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_PRESENT_POSITION);
present_position_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_PRESENT_POSITION);

fprintf('Present Position of Motor 1: %d\n', present_position_1);
fprintf('Present Position of Motor 2: %d\n', present_position_2);

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

% Fermer le port
closePort(port_num);
unloadlibrary(lib_name);
