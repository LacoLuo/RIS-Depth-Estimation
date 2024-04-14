clearvars
clc

set(0,'DefaultFigureWindowStyle','docked')

ss = 1;
scenario_name = 'scene1_SNR34_CLEANLL_NRadar_4GHz_RIS40_ND'; 
Read_folder_name = 'SNR34_CLEANLL_NRadar_4GHz_RIS40_ND'; 
scene_number = 'scene1'; 
scene_number2 = 'Scene 1'; 
Save_Depthest1_folder ='./Results/'; 
Im_resolution_x = 640; 
Im_resolution_z = 480; 

load(strcat('./GT/',scene_number,'.mat')) 
Depth_GT = im(:,:,1); clear im
Radialdist_GT = Depth_GT; 
load(strcat('./Radar_dataset/',Read_folder_name,'/RIS40x40_OSF4x4_RDMest.mat')) 
load(strcat('./Radar_dataset/',Read_folder_name,'/RIS40x40_OSF4x4_data.mat'))

%%%% All other parameters
SNR_dB; %average SNR in dB
Pt_dB; %tx power difference in dB (difference between target transmit power (dBW or dBm) and WI assigned transmit power (dBW or dBm) )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RDM_est_final = RDM_est2 -0.0608; % subtract the "single" distance from Feeder to RIS reference
clear RDM_est2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RIS_codebook_size = RIS_codebook_OSF.*RISsize_xyz;
RDMGT1 = imresize(Radialdist_GT,[RIS_codebook_size(1) RIS_codebook_size(3)], "nearest");
RDMGT3 = imresize(Radialdist_GT,[RIS_codebook_size(1) RIS_codebook_size(3)], "bicubic");
Depth_GT_size = size(Depth_GT);
GT_RIS = figure; imagesc(Depth_GT); colorbar

%% File names
Results_folder = strcat('.\Figures\',scenario_name);
if ~exist(Results_folder, 'dir')
    mkdir(Results_folder)
end
Results_folder2 = strcat('.\Radar_dataset\',Read_folder_name);

RDM_Base_name =strcat(Results_folder,filesep,'BW_12GHz_',num2str(RISsize_xyz(1)),'by',num2str(RISsize_xyz(3)),'_RDM_Pt',num2str(round(Pt_dB)),'dB_diff_SNR_',num2str(round(SNR_dB)),'dB_OSF',num2str(RIS_codebook_OSF(1)),'_');
RDM_Fig_name = strcat(RDM_Base_name,'.fig');  %original one
RDM1_Fig_name= strcat(RDM_Base_name,'1.fig'); %constant interpolation
RDM2_Fig_name= strcat(RDM_Base_name,'2.fig'); %lanczos3 interpolation
RDM3_Fig_name= strcat(RDM_Base_name,'3.fig'); %bicubic interpolation

DM_Base_name = strcat(Results_folder,filesep,'BW_12GHz_',num2str(RISsize_xyz(1)),'by',num2str(RISsize_xyz(3)),'_DM_Pt',num2str(round(Pt_dB)),'dB_diff_SNR_',num2str(round(SNR_dB)),'dB_OSF',num2str(RIS_codebook_OSF(1)),'_');
DMRIS_Base_name = strcat(Results_folder2,filesep,'BW_12GHz_',num2str(RISsize_xyz(1)),'by',num2str(RISsize_xyz(3)),'_DM_Pt',num2str(round(Pt_dB)),'dB_diff_SNR_',num2str(round(SNR_dB)),'dB_OSF',num2str(RIS_codebook_OSF(1)),'_');
ErrorMap_Base_name = strcat(Results_folder2,filesep,'BW_12GHz_',num2str(RISsize_xyz(1)),'by',num2str(RISsize_xyz(3)),'_ErrorMap_Pt',num2str(round(Pt_dB)),'dB_diff_SNR_',num2str(round(SNR_dB)),'dB_OSF',num2str(RIS_codebook_OSF(1)),'_');
DMGT_Base_name = strcat(Results_folder2,filesep,'BW_12GHz_',num2str(RISsize_xyz(1)),'by',num2str(RISsize_xyz(3)),'_DMGT_Pt',num2str(round(Pt_dB)),'dB_diff_SNR_',num2str(round(SNR_dB)),'dB_OSF',num2str(RIS_codebook_OSF(1)),'_');
DM1_Fig_name = strcat(DM_Base_name,'1.fig'); %constant interpolation
DM2_Fig_name = strcat(DM_Base_name,'2.fig'); %lanczos3 interpolation
DM3_Fig_name = strcat(DM_Base_name,'3.fig'); %bicubic interpolation
Final_Result_filename =  strcat(Results_folder,filesep,'BW_12GHz_',num2str(RISsize_xyz(1)),'by',num2str(RISsize_xyz(3)),'_DMresults_Pt',num2str(round(Pt_dB)),'dB_diff_SNR_',num2str(round(SNR_dB)),'dB_OSF',num2str(RIS_codebook_OSF(1)),'.mat');


%% Reshape, upsample and interpolate the estimated depth map

RDM_est = RDM_est_final;
% Upsample the estimated radial distance map based on 3 interpolation methods
RDM_est1 = single(imresize(RDM_est,Depth_GT_size,'nearest'));
RDM_est2 = single(imresize(RDM_est,Depth_GT_size,'lanczos3'));
RDM_est3 = single(imresize(RDM_est,Depth_GT_size,'bicubic'));

% Calculate depth maps from radial distance maps
[xz_grid,Focal_length,~]=RectangularGrid_XZ(Im_resolution_x,1,Im_resolution_z,1,1,1,FOV_azimuth,sensor_width);
theta_coordinate = (pi/2)- atan2(xz_grid(:,:,2),sqrt((xz_grid(:,:,1).^2) + ((Focal_length*ones(Im_resolution_z,Im_resolution_x)).^2) ));
phi_coordinate = atan2((+Focal_length*ones(Im_resolution_z,Im_resolution_x)),xz_grid(:,:,1)); %CHANGE THE phi calculation since boresight direction is +ve Y axis instead of -ve Y axis
Depth_est1 = single(+RDM_est1.*sin(phi_coordinate).*sin(theta_coordinate));
RIS_DM = figure; imagesc(Depth_est1); colorbar
Abs_error_map = abs(Depth_est1-Depth_GT);
Error_map_RIS = figure; imagesc(Abs_error_map); colorbar
Depth_est2 = single(+RDM_est2.*sin(phi_coordinate).*sin(theta_coordinate));
Depth_est3 = single(+RDM_est3.*sin(phi_coordinate).*sin(theta_coordinate));

%% Calculate RMSE and save all maps
% Calculate, display and save the RMSE values
RMSE1(ss) = sqrt(mean((Depth_est1(:) - Depth_GT(:)).^2));
MAE1(ss) = mean(abs(Depth_est1(:) - Depth_GT(:)));
disp(['The upsampled depth RMSE1 using a ' num2str(RISsize_xyz(1)) ' by ' num2str(RISsize_xyz(3)) ' RIS with OSF of ' num2str(RIS_codebook_OSF(1)) ' is ' num2str(100*RMSE1) ' cm.'])
disp(['The upsampled depth MAE1 using a ' num2str(RISsize_xyz(1)) ' by ' num2str(RISsize_xyz(3)) ' RIS with OSF of ' num2str(RIS_codebook_OSF(1)) ' is ' num2str(100*MAE1) ' cm.'])
RMSE2(ss) = sqrt(mean((Depth_est2(:) - Depth_GT(:)).^2));
MAE2(ss) = mean(abs(Depth_est2(:) - Depth_GT(:)));
disp(['The upsampled depth RMSE2 using a ' num2str(RISsize_xyz(1)) ' by ' num2str(RISsize_xyz(3)) ' RIS with OSF of ' num2str(RIS_codebook_OSF(1)) ' is ' num2str(100*RMSE2) ' cm.'])
disp(['The upsampled depth MAE2 using a ' num2str(RISsize_xyz(1)) ' by ' num2str(RISsize_xyz(3)) ' RIS with OSF of ' num2str(RIS_codebook_OSF(1)) ' is ' num2str(100*MAE2) ' cm.'])
RMSE3(ss) = sqrt(mean((Depth_est3(:) - Depth_GT(:)).^2));
MAE3(ss) = mean(abs(Depth_est3(:) - Depth_GT(:)));
disp(['The upsampled depth RMSE3 using a ' num2str(RISsize_xyz(1)) ' by ' num2str(RISsize_xyz(3)) ' RIS with OSF of ' num2str(RIS_codebook_OSF(1)) ' is ' num2str(100*RMSE3) ' cm.'])
disp(['The upsampled depth MAE3 using a ' num2str(RISsize_xyz(1)) ' by ' num2str(RISsize_xyz(3)) ' RIS with OSF of ' num2str(RIS_codebook_OSF(1)) ' is ' num2str(100*MAE3) ' cm.'])

save(Final_Result_filename, 'MAE1','MAE2','MAE3', 'Depth_GT','Depth_est1','RMSE1','Depth_est2','RMSE2','Depth_est3','RMSE3','scenario_name','SNR_dB', 'Pt_dB', 'Read_folder_name', 'scene_number', 'scene_number2');

%% Format figures

save(strcat(Save_Depthest1_folder,filesep,scene_number,'RIS_DMest1.mat'),'Depth_est1')

MAE_RIS = MAE1(1); Depth_est1_RIS = Depth_est1;
min_vector = zeros(2,1); max_vector = zeros(2,1);
min_vector(1)=min(Depth_GT(:));max_vector(1)=max(Depth_GT(:));
min_vector(2)=min(Depth_est1_RIS(:)); max_vector(2)=max(Depth_est1_RIS(:));
Marker_size = 9; Curve_width = 2.5;
Font_size = 16;
labels_fontsize = 16;
Legend_fontsize = 16;
Title_fontsize = 16;
Marker_style = 's<o>^v*x+dph';
Line_style = cell(4,1);
Line_style{1}='-';Line_style{2}='-.';Line_style{3}=':';Line_style{4}='--';
Color = [
    '#0072BD'; %blue
    '#A2142F'; %red
    '#007F00'; %green
    '#7E2F8E'; %purple
    '#D95319'; %orange
    '#4DBEEE'; %cyan
    '#EDB120'; %yellow
    '#888888'; %grey
    '#000000'; %black
    ];


if ishandle(RIS_DM)
    set(0, 'CurrentFigure', RIS_DM)
    set(gca,'FontSize',Font_size)
    title(strcat('\textbf{',scene_number2,' estimated RIS-based depth map (MAE $= ',num2str(round(100*MAE_RIS,1)),'$ cm) }'),'fontsize',Title_fontsize,'interpreter','latex')
    xlabel('\textbf{Horizontal pixel number}','fontsize',labels_fontsize,'interpreter','latex')
    ylabel('\textbf{Vertical pixel number}','fontsize',labels_fontsize,'interpreter','latex')
    hold on;
    %grid on;
    box on;
    clim([min(min_vector([1 2])) max(max_vector([1 2]))])
    set(gca,'XMinorTick','on','YMinorTick','on')
    hold off
end
savefig(RIS_DM,strcat(DMRIS_Base_name,'1.fig'));
saveas(RIS_DM,strcat(DMRIS_Base_name,'1.png'),'png');

if ishandle(Error_map_RIS)
    set(0, 'CurrentFigure', Error_map_RIS)
    set(gca,'FontSize',Font_size)
    title(strcat('\textbf{',scene_number2,' RIS-based absolute est. error map (MAE $= ',num2str(round(100*MAE_RIS,1)),'$ cm) }'),'fontsize',Title_fontsize,'interpreter','latex')
    xlabel('\textbf{Horizontal pixel number}','fontsize',labels_fontsize,'interpreter','latex')
    ylabel('\textbf{Vertical pixel number}','fontsize',labels_fontsize,'interpreter','latex')
    hold on;
    box on;
    set(gca,'XMinorTick','on','YMinorTick','on')
    hold off
end
savefig(Error_map_RIS,strcat(ErrorMap_Base_name,'1.fig'));
saveas(Error_map_RIS,strcat(ErrorMap_Base_name,'1.png'),'png');

if ishandle(GT_RIS)
    set(0, 'CurrentFigure', GT_RIS)
    set(gca,'FontSize',Font_size)
    title(strcat('\textbf{',scene_number2,' ground truth depth map','}'),'fontsize',Title_fontsize,'interpreter','latex')
    xlabel('\textbf{Horizontal pixel number}','fontsize',labels_fontsize,'interpreter','latex')
    ylabel('\textbf{Vertical pixel number}','fontsize',labels_fontsize,'interpreter','latex')
    hold on;
    box on;
    clim([min(min_vector([1 2])) max(max_vector([1 2]))])
    set(gca,'XMinorTick','on','YMinorTick','on')
    hold off
end
savefig(GT_RIS,strcat(DMGT_Base_name,'1.fig'));
saveas(GT_RIS,strcat(DMGT_Base_name,'1.png'),'png');


%% External functions

function [xz_grid,Focal_length,FOV_elevation]=RectangularGrid_XZ(Mx,~,Mz,over_sampling_x,~,over_sampling_z,FOV_azimuth,sensor_width)

    codebook_size_x=over_sampling_x*Mx;
    codebook_size_z=over_sampling_z*Mz;
    Aspect_ratio = codebook_size_x/codebook_size_z;
    
    FOV_az = FOV_azimuth*(pi/180);
    Focal_length = ((sensor_width/2)/tan(FOV_az/2));
    sensor_height = sensor_width/Aspect_ratio;
    sensor_height_spacing = sensor_height/Mz;
    sensor_width_spacing = sensor_width/Mx;
    x = ((+sensor_width-sensor_width_spacing)/2):-sensor_width_spacing:((-sensor_width+sensor_width_spacing)/2);
    z = ((+sensor_height-sensor_height_spacing)/2):-sensor_height_spacing:((-sensor_height+sensor_height_spacing)/2);
    
    [X,Z] = meshgrid(x,z);
    xz_grid=cat(3,X,Z);
    
    FOV_el = 2*atan(sensor_height/(2*Focal_length));
    FOV_elevation = (FOV_el)/(pi/180);

end
