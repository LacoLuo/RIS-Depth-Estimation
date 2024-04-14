%%%% DeepSense Synth radar parameters set %%%%
% A detailed description of the parameters is available on ... 

%Ray-tracing scenario
radar_params.scenario = 'one_wall';          % The adopted ray tracing scenario [check the available scenarios at https://deepmimo.net/scenarios/]

%Dynamic Scenario Scenes [only for dynamic (multiple-scene) scenarios]
radar_params.scene_first = 1; %551;
radar_params.scene_last = 1; %551;

% Active radar base stations
radar_params.active_BS = 1; %[1 2 4];             % The numbers of the active BSs (check the scenario description at https://deepmimo.net/scenarios/ for the BS numbers) 

% Antenna array dimensions
radar_params.num_ant_TX = [1, 1, 1];      % Number of antenna elements for the radar TX arrays at the BS in the x,y,z-axes
% By defauly, all BSs will have the same array sizes
% To define different array sizes for the selected active TX BSs, you can add multiple rows. 
% Example: For two active BSs with a 8x4 y-z UPA in the first BS and 4x4
% x-z UPA for the second BS, you write  
% radar_params.num_ant_BS = [[1, 8, 4]; [1, 4, 4]];

radar_params.num_ant_RX = [1, 1, 1];      % Number of antenna elements for the radar RX arrays at the BS in the x,y,z-axes
% To define different array sizes for the selected active TX BSs, you can add multiple rows. 
% Example: For two active BSs with a 8x4 y-z UPA in the first BS and 4x4
% x-z UPA for the second BS, you write  
% radar_params.num_ant_BS = [[1, 8, 4]; [1, 4, 4]];

% Antenna array orientations
radar_params.activate_array_rotation = 0; % 0 -> no array rotation - 1 -> apply the array rotation defined in radar_params.array_rotation_TX and radar_params.array_rotation_RX
radar_params.array_rotation_TX = [5, 10, 20];         
% 3D rotation angles in degrees around the x,y,z axes respectively
% The origin of these rotations is the position of the first BS antenna element
% The rotation sequence applied: (a) rotate around z-axis, then (b) rotate around y-axis, then (c) rotate around x-axis. 
% To define different orientations for the active TX BSs, add multiple rows..
% Example: For two active BSs with different transmit array orientations, you can define
% radar_params.array_rotation_TX = [[10, 30, 45]; [0, 30, 0]];

radar_params.array_rotation_RX = [5, 10, 20]; 
% Radar RX antenna array orientation settings
% To define different orientations for the active RX BSs, add multiple rows..
% Example: For two active BSs with different transmit array orientations, you can define
% radar_params.array_rotation_RX = [[10, 30, 45]; [0, 30, 0]];

% Antenna array spacing
radar_params.ant_spacing_TX = .5;           % ratio of the wavelength; for half wavelength enter .5
radar_params.ant_spacing_RX = .5;           % ratio of the wavelength; for half wavelength enter .5

% Antenna element radiation pattern
radar_params.radiation_pattern = 0;         % 0: Isotropic and 
                                            % 1: Half-wave dipole
radar_params.radar_channel_taps = 2000; % Radar channel tap length to generate a time domain radar channel impulse response for one "chirp burst"
% radar_params.scene_frequency = 30; %Hz %%%%%%%%%%%%%%%% DEBUG!!! %%%%%%%%%%%% 

%Chirp configuration parameters
radar_params.S = 300e12; %MAX AWR2243 S is 300MHz/us --> DO not increase more than that %%%%%%%%%%% CHANGE %%%%%%%%%%%%%%%%%%%%%%%%
radar_params.Fs = 38e6; % Triple Fs from 15e6 to 45e6 to reduce BW by 3 (reduce min and max range by 3 too) %%%%%%%%%%% CHANGE %%%%%%%%%%%%%%%%%%%%%%%%
%MAX AWR2243 sampling frequency is 25MSps --> DO not increase more than 45MSps
radar_params.N_ADC = 512; % reducing this number also makes the Depth Map generation faster %%%%%%%%%%% CHANGE %%%%%%%%%%%%%%%%%%%%%%%%
radar_params.N_loop = 1;  % Number of chirp bursts (minimum of 1) 
radar_params.T_idle = 0;
radar_params.T_start = 0;
radar_params.T_excess = 0;
radar_params.duty_cycle = 1;
radar_params.F0 = 60e9 - radar_params.S*radar_params.T_start;

%Derived radar frame parameters
radar_params.T_active = radar_params.N_ADC/radar_params.Fs;
radar_params.T_ramp = radar_params.T_start + radar_params.T_active + radar_params.T_excess;
radar_params.T_chirp = radar_params.T_idle + radar_params.T_ramp;
% radar_params.T_gap = radar_params.T_chirp - radar_params.T_active;
radar_params.T_gap = radar_params.T_idle + radar_params.T_start + radar_params.T_excess;
radar_params.BW_active = radar_params.S*radar_params.T_active;
radar_params.BW_total = radar_params.S*radar_params.T_chirp;
radar_params.fc = radar_params.F0 + radar_params.S*((radar_params.T_active/2)+radar_params.T_start);
radar_params.f_step = radar_params.S/radar_params.Fs;
% radar_params.T_PRI = radar_params.radar_channel_taps/radar_params.Fs;  % Pulse repetition interval in seconds, for Doppler processing in FMCW radar                                       
radar_params.T_PRI = radar_params.T_chirp;  %%%%%%%%%%% CHANGE %%%%%%%%%%%%%%%%%%%%%%%%

% % % 1/((40*40*4*4) * radar_params.T_PRI)
% % % 1/((20*20*4*4) * radar_params.T_PRI)

% Channel parameter
radar_params.num_paths = 100e3;                 % Maximum number of paths to be considered (starting from 1), e.g., choose 1 if you are only interested in the strongest path

% Computation parameter
radar_params.comp_speed = 5;                 % control the compromise between computational speed and memory requirement 
                                             % (defined between 1 and 5), e.g., choose 5 if you are only interested in the fastest computation with the largest memory requirement  

radar_params.saveDataset = 0;               % 0: Will return the dataset without saving it (highly recommended!) 