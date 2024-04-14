clearvars
clc

addpath("utils/")
set(0,'DefaultFigureWindowStyle','docked')
% tic
fprintf('This message is posted at date and time: %s\n', datetime)
disp('  ')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Indoor room
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% Approximation
% 1) RIS codebook is designed based on a far field reflected channel at one freq only! (60GHz from WI)
% and a near-field incident channel at one freq only! (f_0_active = 60 GHz ---> wavelength_0 variable in line 136)
% 2) Must update N_range below if N_sample variable has changed!!!

%% Calculation of Alpha0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RISsize_xyz = [40 1 40];
RIS_codebook_OSF = [4 1 4];
RIS_element_spacing = 0.5;
sensor_width = 32*(1e-3); %Width of the Blender camera sensor
FOV_azimuth = 100; % Azimuth field of view in degrees (0-180)
Aspect_ratio = 4/3; %16/9;
Folder_name = 'SNR34_CLEANLL_NRadar_4GHz_RIS40_ND';

Current_power_WI = -30; %Current power in WI in dBW (Now it is set to 0 dBm (-30 dBW) )
Target_power = -15; %Target power in the simulation in dBW (Now it is set to 15 dBm (-15 dBW) )
Pt_dB =  Target_power - Current_power_WI; %TX added power in the simulation in dBW
BW = 12e9;
Gt=25;             % dBi --> Horn antenna at 60GHz with 3cm circular diameter and 0.8 effective apperture factor
Gr=25;             % dBi --> Horn antenna at 60GHz with 3cm circular diameter and 0.8 effective apperture factor
NF=10;             % Noise figure at the radar transceiver
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

filename_RDMest = strcat('RIS',num2str(RISsize_xyz(1)),'x',num2str(RISsize_xyz(3)),'_OSF',num2str(RIS_codebook_OSF(1)),'x',num2str(RIS_codebook_OSF(3)),'_RDMest.mat');
filename_data = strcat('RIS',num2str(RISsize_xyz(1)),'x',num2str(RISsize_xyz(3)),'_OSF',num2str(RIS_codebook_OSF(1)),'x',num2str(RIS_codebook_OSF(3)),'_data.mat');
Results_folder = strcat('.\Radar_dataset\',Folder_name);
if ~exist(Results_folder, 'dir')
    mkdir(Results_folder)
end

noise_power_dBW=-204+10*log10(BW)+NF; % Noise power in dBW
Pn=(10^(.1*(noise_power_dBW)))./(10.^(.1*(Gt+Gr+Pt_dB))); %Noise power in Watts
Pn_device = (10^(.1*(noise_power_dBW)));
Pn_dB = 10*log10(Pn); %Noise power in dBW
Current_power_WI_Watt = (10^(.1*(Current_power_WI)));

RISsize = prod(RISsize_xyz);
ang_conv=pi/180;
c = physconst('LightSpeed');

%---- RIS to RIS channel information
load('./Raytracing_scenarios/RIStoRIS_mat/scene_0_TX5.mat')
RISRIS_params.num_paths = numel(channels{1}.paths.phase); %2048;
RISRIS_params.DoD_theta = channels{1}.paths.DoD_theta;
RISRIS_params.DoD_phi = channels{1}.paths.DoD_phi;
RISRIS_params.DoA_theta = channels{1}.paths.DoA_theta;
RISRIS_params.DoA_phi = channels{1}.paths.DoA_phi;
%%% Remove the 1e-3 --> we just input path gain in dB
RISRIS_params.power = (10.^(0.1*( double(channels{1}.paths.power) - (Current_power_WI + 30)  ))); % Subtract the transmit power to get accurate path gain (Path_gain_dB = P_rec_dBm - P_transmit_dBm)
RISRIS_params.phase = channels{1}.paths.phase;
RISRIS_params.ToA = channels{1}.paths.ToA;
clear channels

%---- Feeder to RIS channel information
load('./Raytracing_scenarios/FeedertoRIS_mat/scene_0_TX1.mat')
load('./Raytracing_scenarios/FeedertoRIS_mat/scene_0_TX1_Part2.mat')

% distances; RIS_locations; Feeder_location;
FeedertoRIS_params = cell(RISsize,1);
FeedertoRIS_power=zeros(RISsize,1); FeedertoRIS_phase=FeedertoRIS_power; FeedertoRIS_ToA=FeedertoRIS_power;

RISindices = reshape(reshape((1:1:RISsize).',40,[]).',[],1);
for rr=1:1:numel(channels)
    %%% Remove the 1e-3 --> we just input path gain in dB
    FeedertoRIS_power(rr) = (10.^(0.1*( double(channels{RISindices(rr)}.paths.power) - (Current_power_WI + 30) ))); % Subtract the transmit power to get accurate path gain (Path_gain_dB = P_rec_dBm - P_transmit_dBm)
    FeedertoRIS_phase(rr) = channels{RISindices(rr)}.paths.phase;
    FeedertoRIS_ToA(rr) = channels{RISindices(rr)}.paths.ToA;
end
clear channels

%-------------- Feeder to RIS nearfield channel difference calculation (difference between reference RIS element and all the RIS elements)
g_channel_diff_angle = double(FeedertoRIS_phase - FeedertoRIS_phase(1)); %%%%%%%%%%%%%%%%%%%% Approximation (designed based on one single carrier freq = f0_active = 60GHz)
g_channel_diff_abs = sqrt(FeedertoRIS_power./FeedertoRIS_power(1)); 
g_channel_diff = g_channel_diff_abs.*exp(sqrt(-1).*g_channel_diff_angle.*ang_conv); %No third dimension %Already has the negative j there in the WI phase shift

%-------------- RIS far-field array response vector calculation --------------%
% RX antenna parameters for a UPA RIS structure
M_RIS_ind = double(antenna_channel_map(RISsize_xyz(1), RISsize_xyz(2), RISsize_xyz(3), 0));
kd_RIS=2*pi*RIS_element_spacing;

% Far field Array Response from RIS to target (Forward direction)
gamma_aRIS_F=sqrt(-1)*kd_RIS*[sind(RISRIS_params.DoA_theta).*cosd(RISRIS_params.DoA_phi); sind(RISRIS_params.DoA_theta).*sind(RISRIS_params.DoA_phi); cosd(RISRIS_params.DoA_theta)];
array_response_RIS_F = exp(M_RIS_ind*gamma_aRIS_F);
a_RIS_Forward = reshape(array_response_RIS_F,RISsize,1,RISRIS_params.num_paths); %third dimension is the path index
clear array_response_RIS_F gamma_aRIS_F

% Far field Array Response from target to RIS (backward direction)
gamma_aRIS_B=sqrt(-1)*kd_RIS*[sind(RISRIS_params.DoD_theta).*cosd(RISRIS_params.DoD_phi); sind(RISRIS_params.DoD_theta).*sind(RISRIS_params.DoD_phi); cosd(RISRIS_params.DoD_theta)];
array_response_RIS_B = exp(M_RIS_ind*gamma_aRIS_B);
a_RIS_Backward = reshape(array_response_RIS_B,RISsize,1,RISRIS_params.num_paths); %third dimension is the path index
clear array_response_RIS_B gamma_aRIS_B

%-------------- Final channel magnitude and phase and ToA calculations (with respect to reference elements)
Zeta = (2*FeedertoRIS_ToA(1)) + RISRIS_params.ToA; % row vector of all three TOAs added together (Feeder to RIS reference, RIS reference to RIS reference, RIS reference to RIS)
Gamma_mag = sqrt( (FeedertoRIS_power(1).^2).*RISRIS_params.power );
Gamma_phase = (2*FeedertoRIS_phase(1)) + RISRIS_params.phase;  % (Feeder to RIS reference, RIS reference to RIS reference, RIS reference to RIS)
Alpha_phase = Gamma_phase;

%-------------- RIS reflected angle rectangular grid codebook generation
F_RIS =RIS_rectangular_codebook_generator(RISsize_xyz(1),1,RISsize_xyz(3),...
    RIS_codebook_OSF(1),1,RIS_codebook_OSF(3),RIS_element_spacing,...
    FOV_azimuth,sensor_width,Aspect_ratio);

%-------------- RIS interaction vector calculation --------------%
% Near-field incident phase shifts
Wavelength_0 = c/(60e9);  %%%%%%%%%%%%%%%%%%%% Approximation
kk = ((2*pi)/Wavelength_0);
Incident = kk.*(distances-distances(1));


%% Calculation of IF radar signal

radar_params = read_params('radar_params_SingleChirp_BW4GHz_40by40RIS_3Hz.m');

channel_params.num_paths = RISRIS_params.num_paths;
channel_params.DoD_theta = zeros(1,RISRIS_params.num_paths);
channel_params.DoD_phi = zeros(1,RISRIS_params.num_paths);
channel_params.DoA_theta = zeros(1,RISRIS_params.num_paths);
channel_params.DoA_phi = zeros(1,RISRIS_params.num_paths);
channel_params.power = zeros(1,RISRIS_params.num_paths);
channel_params.phase = Alpha_phase;
channel_params.ToA = Zeta;
channel_params.Doppler_acc = zeros(1,RISRIS_params.num_paths);
channel_params.Doppler_vel = zeros(1,RISRIS_params.num_paths);

Inputs.F_RIS = F_RIS;
Inputs.Incident = Incident;
Inputs.a_RIS_Forward = a_RIS_Forward;
Inputs.a_RIS_Backward = a_RIS_Backward;
Inputs.g_channel_diff = g_channel_diff;
Inputs.Gamma_mag = Gamma_mag;
Inputs.Pn = Pn;
Inputs.radar_params = radar_params;
Inputs.channel_params = channel_params;
Inputs.Current_power_WI_Watt = Current_power_WI_Watt;
Inputs.BW = BW;

rng(0)
[range_est,Ps_dB_avg_per_beam,Pn_dB_avg_per_beam,radar_KPI] = RIS_parforWaitbar(size(F_RIS,2),Inputs);

RIS_radar_KPI = radar_KPI{1}; clear radar_KPI
RIS_radar_KPI.DM_sensing_rate = RIS_radar_KPI.Radar_frame_rate / (prod(RISsize_xyz)*prod(RIS_codebook_OSF));
Ps_dB_avg = 10*log10( mean( 10.^(0.1.*Ps_dB_avg_per_beam) ));  %in dBW
Pn_dB_avg = 10*log10( mean( 10.^(0.1.*Pn_dB_avg_per_beam) ));  %in dBW
SNR_dB = Ps_dB_avg - Pn_dB_avg; %in dB
disp(' ')
disp(['SNR = ' num2str(SNR_dB) ' dB'])
RIS_radar_KPI

% Range Map Construction
RDM_est=reshape(range_est,(RIS_codebook_OSF(1)*RISsize_xyz(1)),(RIS_codebook_OSF(3)*RISsize_xyz(3))).';
RDM_est2 = fliplr(RDM_est);
figure; imagesc(RDM_est2)

%debugging
Ps_power_map=reshape(Ps_dB_avg_per_beam,(RIS_codebook_OSF(1)*RISsize_xyz(1)),(RIS_codebook_OSF(3)*RISsize_xyz(3))).';
Pn_power_map=reshape(Pn_dB_avg_per_beam,(RIS_codebook_OSF(1)*RISsize_xyz(1)),(RIS_codebook_OSF(3)*RISsize_xyz(3))).';
SNR_map = Ps_power_map-Pn_power_map;
figure; imagesc(fliplr(SNR_map))


%% Save data to compare against the ground truth maps
save(strcat(Results_folder,filesep,filename_RDMest),'RDM_est2')
clear F_RIS a_RIS_Backward a_RIS_Forward Gamma_mag Gamma_phase Alpha_phase H_mag Alpha_mag
save(strcat(Results_folder,filesep,filename_data),'-v7.3')
% Run the next code: Test_est_error_....m
disp('  ')
disp('DONE!')
disp('  ')
fprintf('This message is posted at date and time: %s\n', datetime)

%% External function 1 (construct radar IF signal)

function [IF_signal_Final,radar_KPI,SNR_components]=construct_radar_IF_signal(tx_ant_size, tx_rotation, tx_ant_spacing, ~, rx_ant_size, rx_rotation, rx_ant_spacing, ~, channel_params, params, Pn, ~, Power_vec)

    channel_params.power = Power_vec;
    
    fc = params.fc;
    Fs = params.Fs;
    Wavelength = physconst('LightSpeed')/fc;
    ang_conv=pi/180;
    Ts=1/Fs;
    T_PRI = params.T_PRI;
    N_loop = params.N_loop;
    N_ADC = params.N_ADC;
    num_paths = channel_params.num_paths;
    F0_active = params.F0 + params.S*params.T_start;
    
    % Radar KPI
    radar_KPI.range_resolution = physconst('LightSpeed')/(2*params.BW_active);
    radar_KPI.max_detectable_range = radar_KPI.range_resolution*(N_ADC-1);
    radar_KPI.velocity_resolution = Wavelength/(2*params.T_PRI*params.N_loop);
    radar_KPI.max_detectable_velocity = [-1 ((params.N_loop-2)/params.N_loop)]*(Wavelength/(4*params.T_PRI));
    radar_KPI.BW_active = params.BW_active;
    radar_KPI.BW_total = params.BW_total;
    T_radar_frame = (params.N_loop*params.T_PRI)/params.duty_cycle;
    radar_KPI.Radar_frame_rate = 1/T_radar_frame;
    
    % TX antenna parameters for a UPA structure
    M_TX_ind = antenna_channel_map(tx_ant_size(1), tx_ant_size(2), tx_ant_size(3), 0);
    M_TX=prod(tx_ant_size);
    kd_TX=2*pi*tx_ant_spacing;
    
    % RX antenna parameters for a UPA structure
    M_RX_ind = antenna_channel_map(rx_ant_size(1), rx_ant_size(2), rx_ant_size(3), 0);
    M_RX=prod(rx_ant_size);
    kd_RX=2*pi*rx_ant_spacing;
    
    if num_paths == 0
        IF_signal_Final = complex(zeros(M_RX, M_TX, N_ADC, N_loop));
        return;
    end
    
    % Change the DoD and DoA angles based on the panel orientations
    if params.activate_array_rotation
        [DoD_theta, DoD_phi, DoA_theta, DoA_phi] = axes_rotation(tx_rotation, channel_params.DoD_theta, channel_params.DoD_phi, rx_rotation, channel_params.DoA_theta, channel_params.DoA_phi);
    else
        DoD_theta = channel_params.DoD_theta;
        DoD_phi = channel_params.DoD_phi;
        DoA_theta = channel_params.DoA_theta;
        DoA_phi = channel_params.DoA_phi;
    end
    
    % Apply the radiation pattern of choice
    if params.radiation_pattern % Half-wave dipole radiation pattern
        power = channel_params.power.* antenna_pattern_halfwavedipole(DoD_theta, DoD_phi) .* antenna_pattern_halfwavedipole(DoA_theta, DoA_phi);
    else % Isotropic radiation pattern
        power = channel_params.power;
    end
    
    % TX Array Response - BS
    gamma_TX=sqrt(-1)*kd_TX*[sind(DoD_theta).*cosd(DoD_phi);
        sind(DoD_theta).*sind(DoD_phi);
        cosd(DoD_theta)];
    array_response_TX = exp(M_TX_ind*gamma_TX);
    
    % RX Array Response - BS
    gamma_RX=sqrt(-1)*kd_RX*[sind(DoA_theta).*cosd(DoA_phi);
        sind(DoA_theta).*sind(DoA_phi);
        cosd(DoA_theta)];
    array_response_RX = exp(M_RX_ind*gamma_RX);
    
    % Account only for the channel within the user-specific channel tap length
    delay_normalized=channel_params.ToA/Ts;
    power(delay_normalized>(N_ADC-1)) = 0;
    delay_normalized(delay_normalized>(N_ADC-1)) = (N_ADC-1);
    
    % received IF signal calculation
    % tic
    IF_sampling_mat = zeros(N_ADC,num_paths);
    for ll=1:1:num_paths
        IF_sampling_mat((ceil(double(delay_normalized(ll)))+1):1:N_ADC,ll) = 1;
    end
    time_fast = Ts*(0:1:(N_ADC-1)).';
    Noise_phase_shift = exp( sqrt(-1)*2*pi*( (F0_active.*(time_fast)) + (0.5.*params.S.*(time_fast.^2)) ) );
    Rx_noise2 = reshape(repmat(reshape(Noise_phase_shift,1,1,N_ADC,1),M_RX,M_TX,1,N_loop), M_RX, M_TX, N_ADC, N_loop);
    
    switch params.comp_speed
        case 5 %----- faster calculation with higher memory requirement (5D matrix cacluation and 4D matrix output)
            time_slow = time_fast + reshape(((0:1:(N_loop-1))*T_PRI),1,1,[]);
            Tau3_rt = ((double(channel_params.Doppler_acc).*(time_slow.^2))./(2*physconst('LightSpeed')));
            Tau2_rt = ((double(channel_params.Doppler_vel).*time_slow)./physconst('LightSpeed'));
            Tau_rt = (double(delay_normalized).*Ts) + Tau2_rt + Tau3_rt;
            %----- (a) additional traveling distance and (b) Doppler frequency affecting the phase terms
            Extra_phase = exp(sqrt(-1)*double(channel_params.phase).*ang_conv);
            Phase_terms = exp(sqrt(-1)*2*pi*( (F0_active.*(Tau_rt)) -(0.5.*params.S.*(Tau_rt.^2)) +(params.S.*time_fast.*Tau_rt)));
            IF_mat = sqrt(double(power)).*conj(Extra_phase).*Phase_terms.*IF_sampling_mat; %%%% conjugate is based on the new derivation we have reached for +sin(.) Quadrature carrier signal.
    
            IF_signal = sum(reshape(array_response_RX, M_RX, 1, 1, num_paths, 1) .* reshape(array_response_TX, 1, M_TX, 1, num_paths, 1) .* reshape(IF_mat, 1, 1, N_ADC, num_paths, N_loop), 4);
            IF_signal = reshape(IF_signal, M_RX, M_TX, N_ADC, N_loop);
    
            %%------------- Received Signal and Noise Calculation
            Ps = abs(IF_signal).^2;
            Rx_noise = sqrt(Pn/2).* ( randn(size(IF_signal)) + sqrt(-1).*randn(size(IF_signal)) );
            %Rx_noise = sqrt(Pn/(2*BW)).* ( randn(size(IF_signal)) + sqrt(-1).*randn(size(IF_signal)) );
            Rx_noise_final = Rx_noise.*Rx_noise2;
            Pnoise_final = abs(Rx_noise_final).^2;
            SNR_components.Ps_dB_avg_per_beam=10*log10(mean( Ps(:) ));
            SNR_components.Pn_dB_avg_per_beam=10*log10(mean( Pnoise_final(:) ));
    
            IF_signal_Final = IF_signal + Rx_noise_final;
        otherwise
            disp('The parameter params.comp_speed is defined from 1 to 5. please change its value accordingly');
    end
    % toc
end

%% External function 2 (RIS reflection rectangular grid codebook)

function [F_CB]=RIS_rectangular_codebook_generator(Mx,~,Mz,over_sampling_x,~,over_sampling_z,ant_spacing,FOV_azimuth,sensor_width,Aspect_ratio)

    codebook_size_x=over_sampling_x*Mx;
    codebook_size_z=over_sampling_z*Mz;
    
    FOV_az = FOV_azimuth*(pi/180);
    Focal_length = ((sensor_width/2)/tan(FOV_az/2));
    sensor_height = sensor_width/Aspect_ratio;
    sensor_height_spacing = sensor_height/codebook_size_z;
    sensor_width_spacing = sensor_width/codebook_size_x;
    
    x_camera_plane = ((+sensor_width-sensor_width_spacing)/2):-sensor_width_spacing:((-sensor_width+sensor_width_spacing)/2);
    z_camera_plane = ((+sensor_height-sensor_height_spacing)/2):-sensor_height_spacing:((-sensor_height+sensor_height_spacing)/2);
    [X_camera_plane,Z_camera_plane] = meshgrid(x_camera_plane,z_camera_plane);
    Y_camera_plane = (+Focal_length*ones(codebook_size_z,codebook_size_x));
    theta_qz_coordinate = (pi/2)- atan2(Z_camera_plane,sqrt((X_camera_plane.^2) + (Y_camera_plane.^2) ));
    theta_qz =reshape(theta_qz_coordinate.',1,[]);
    
    temp = x_camera_plane;
    x_camera_plane = -z_camera_plane;
    z_camera_plane = temp.';
    clear temp
    [X_camera_plane,Z_camera_plane] = meshgrid(x_camera_plane,z_camera_plane);
    Y_camera_plane = (+Focal_length*ones(codebook_size_x,codebook_size_z));
    theta_qx_coordinate = rot90( (pi/2)- atan2(Z_camera_plane,sqrt((X_camera_plane.^2) + (Y_camera_plane.^2) )));
    theta_qx =reshape(theta_qx_coordinate.',1,[]);
    
    kd=2*pi*ant_spacing;
    antx_index=0:1:Mx-1;
    antz_index=0:1:Mz-1;
    
    F_CBx=zeros(Mx,length(theta_qx));
    for i=1:1:length(theta_qx)
        F_CBx(:,i)=exp(+1j*kd*antx_index.'*cos(theta_qx(i)));
    end
    
    F_CBz=zeros(Mz,length(theta_qz));
    for i=1:1:length(theta_qz)
        F_CBz(:,i)= exp(+1j*kd*antz_index.'*cos(theta_qz(i)));
    end
    
    F_CB=krb(F_CBz,F_CBx);
end

%% External function 3 (My parfor function)

function [Ps_dB_avg_per_beam_val,Pn_dB_avg_per_beam_val,range_est_val,radar_KPI] = My_parfor_function(ff,Inputs)

    F_RIS = Inputs.F_RIS;
    Incident = Inputs.Incident;
    a_RIS_Forward = Inputs.a_RIS_Forward;
    a_RIS_Backward = Inputs.a_RIS_Backward;
    g_channel_diff = Inputs.g_channel_diff;
    Gamma_mag = Inputs.Gamma_mag;
    Pn = Inputs.Pn;
    radar_params = Inputs.radar_params;
    channel_params = Inputs.channel_params;
    Current_power_WI_Watt = Inputs.Current_power_WI_Watt;
    BW = Inputs.BW;
    
    %-------------- RIS interaction vector calculation
    Reflected_phase = angle(F_RIS(:,ff));
    RIS_phases =  - Reflected_phase  - (-Incident); %Outer negative signs are for conjugate beamforming design. The inner negative sign of "Incident" is for constructing the same exact phase as the channel g with its correct sign
    RIS_int_vec = exp(sqrt(-1)*RIS_phases); %No third dimension
    
    %-------------- Final channel magnitude and phase and ToA calculations (with respect to reference elements)
    H_mag = reshape(sum(g_channel_diff.*RIS_int_vec.*a_RIS_Forward,1),1,[]) .* reshape(sum(g_channel_diff.*RIS_int_vec.*a_RIS_Backward,1),1,[]);
    Alpha_mag = double(sqrt(Current_power_WI_Watt).*Gamma_mag .* H_mag);
    
    %%------------------------------- Calculation of IF radar signal
    [IF_signal_Final,radar_KPI,SNR_comps]=...
        construct_radar_IF_signal([1 1 1], [0 0 0], 0.5, [1 2 1 270 90], ...
        [1 1 1], [0 0 0], 0.5, [1 2 1 270 90], channel_params, radar_params, Pn, BW, Alpha_mag.^2);
    
    Max_r = radar_KPI.max_detectable_range;
    Delta_r = radar_KPI.range_resolution;
    N_range = 512; 
    
    RX_signal2= permute(squeeze(IF_signal_Final),[1 2]);
    Ps_dB_avg_per_beam_val = SNR_comps.Ps_dB_avg_per_beam;  %in dBW
    Pn_dB_avg_per_beam_val = SNR_comps.Pn_dB_avg_per_beam;  %in dBW
    
    %%---- Range and Doppler Map
    Range_profile = fft(RX_signal2,N_range,1);
    Range_profile2 = abs(Range_profile(:,1)); %%% if N_Doppler = 1
    range_indices = 0:1:(Max_r/Delta_r);
    range_discrete_values = (range_indices)*Delta_r;
    [~,Idx_max] = max(Range_profile2);
    range_est_val = range_discrete_values(Idx_max);
end

%% External function 4 (parfor progress bar display)
function [range_est,Ps_dB_avg_per_beam,Pn_dB_avg_per_beam,rKPI] = RIS_parforWaitbar(N_size,Inputs)

    D = parallel.pool.DataQueue;
    h = waitbar(0, 'Please wait ...');
    afterEach(D, @nUpdateWaitbar);
    
    p_iter = 1;
    range_est = zeros(N_size,1); Ps_dB_avg_per_beam = range_est; Pn_dB_avg_per_beam = range_est; rKPI=cell(N_size,1);
    parfor ff= 1:N_size
        [Ps_dB_avg_per_beam_val,Pn_dB_avg_per_beam_val,range_est_val,radar_KPI] = My_parfor_function(ff,Inputs)
        Ps_dB_avg_per_beam(ff) = Ps_dB_avg_per_beam_val;
        Pn_dB_avg_per_beam(ff) = Pn_dB_avg_per_beam_val;
        range_est(ff) = range_est_val;
        rKPI{ff} = radar_KPI;
        send(D, ff);
    end

    function nUpdateWaitbar(~)
        waitbar(p_iter/N_size, h);
        p_iter = p_iter + 1;
    end
end
