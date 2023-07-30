clc;
clear;
close all;
%% VG-TechCenter

CP=load('matlab.mat');
 [c,~]=size(CP.cameraParams.PatternExtrinsics);
 camera_params.IntrinsicMatrix=CP.cameraParams.K;  
 in= camera_params.IntrinsicMatrix;  %内参矩阵
CMOSsize=CP.cameraParams.ImageSize;%传感器尺寸
image_points_norm_all=cell(1,c);
for s=1:c

camera_params.R=CP.cameraParams.PatternExtrinsics(s, 1).R;
camera_params.t=CP.cameraParams.PatternExtrinsics(s, 1).Translation;

world_point=CP.cameraParams.WorldPoints;
world_point(:,3)=0;
    
    % 相机外参
    R = camera_params.R;    % 旋转矩阵
    t = camera_params.t;    % 平移向量
    
    % 将世界坐标点转换到相机坐标系
    camera_points = R * world_point' + t';
    
    % 透视投影
    
    image_points = in*camera_points;
    image_points=image_points';
    [m,~]=size(image_points);
    image_points_norm=zeros(m,3);
    for i=1:m
        image_points_norm(i,:)=image_points(i,:)/image_points(i,3);
    end
image_points_norm_all{s}=image_points_norm;


end

    %像面成像绘图
    l=ceil(sqrt(s));

    % 设置图形窗口的大小和位置
    width = 2000;   % 宽度（以像素为单位）
    height = 1600;  % 高度（以像素为单位）
    xPos = 0;    % 水平位置（以像素为单位）
    yPos = 0;    % 垂直位置（以像素为单位）
    set(figure, 'Position', [xPos, yPos, width, height]);
    for s=1:c
        subplot(6,6,s);
        plot(image_points_norm_all{s}(:,1),image_points_norm_all{s}(:,2),'b.');axis equal;%hold on;
        xlim([0, CMOSsize(1)]); % 设置 x 轴的范围
        ylim([0, CMOSsize(2)]);  % 设置 y 轴的范围
        xlabel('X');
        ylabel('Y');
        title(['Photo-',num2str(s),'  Imgpoints']);
    end







