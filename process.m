% clear and close everything
% clear all; close all; dbstop error; clc;
% disp('======= Synchronisation Demo =======');

% loadSLAMCameraTrajectory
% Timestamps_SLAM=loadTimestamps('C:\Users\LQH\Documents\shareUbuntu\imu\drive_0018_sync\image');
% Timestamps_IMU=loadTimestamps('C:\Users\LQH\Documents\shareUbuntu\imu\drive_0018_sync\oxts');
% timeIMU = datevec(Timestamps_IMU);
% timeSLAM = datevec(Timestamps_SLAM);
% loadSLAMCameraTrajectory;
%  poseSLAM1;
% oxts = loadOxtsliteData('C:\Users\LQH\Documents\shareUbuntu\imu\drive_0018_sync');
% poseIMU = convertOxtsToPose(oxts);
%%%Euler angle synchronisation%%%
%提取SLAM的欧拉角
% R_SLAM_IMU=[0 -1 0;0 0 -1;1 0 0];
% EulerSLAM=zeros(2762,3);
% for i=1:2762
%     [a,b,c]=dcm2angle(poseSLAM(1:3,1:3,i), 'ZYX');  
% %      EulerSLAM(i,:)=[a,b,c]./pi.*180;
%      EulerSLAM(i,:)=transpose(R_SLAM_IMU*transpose([a,b,c]))./pi.*180;
% end
%%提取从经纬度得来得欧拉角
% EulerIMU1=zeros(2762,3);
% for i=1:2762
%     [a,b,c]=dcm2angle(poseIMU{i}(1:3,1:3), 'ZYX');    
%     EulerIMU1(i,:)=[a,b,c]./pi.*180;
% end

%%转换IMU的角度范围从【-180，180】到【-90，90】
%%EulerIMU1从经纬度得来
% EulerIMU2=zeros(2762,3);
% for i=1:2762
%     if EulerIMU1(i,1)>90
%         EulerIMU2(i,1)=180-EulerIMU1(i,1);
%     elseif EulerIMU1(i,1)<-90
%         EulerIMU2(i,1)=-180-EulerIMU1(i,1);
%     else
%         EulerIMU2(i,1)=EulerIMU1(i,1);
%     end
% end
%%转换IMU的角度范围从【-180，180】到【-90，90】
%%EulerIMU是角速度,EulerIMU3是其积分
% %%转换积分符号
% EulerIMU3=zeros(2762,3);
% EulerIMU4=zeros(2762,3);
% EulerIMU3(:,1) = -cumtrapz(t_IMU,transpose(EulerIMU(:,3)))./pi.*180;
% for i=1:2762
%     if EulerIMU3(i,1)>=90
%         EulerIMU3(i,1)=180-EulerIMU3(i,1);
%         EulerIMU4(i,3)=-EulerIMU(i,3)./pi.*180;
%     elseif EulerIMU3(i,1)<=-90
%         EulerIMU3(i,1)=-180-EulerIMU3(i,1);
%         EulerIMU4(i,3)=-EulerIMU(i,3)./pi.*180;
%     else
%         EulerIMU3(i,1)=EulerIMU3(i,1);
%         EulerIMU4(i,3)=EulerIMU(i,3)./pi.*180;
%     end
% end
% for j=1:2762
%     if EulerIMU3(j,1)>=90
%         EulerIMU3(j,1)=180-EulerIMU3(j,1);
%         EulerIMU4(j,3)=-EulerIMU4(j,3);
%     elseif EulerIMU3(j,1)<=-89
%         EulerIMU3(j,1)=-180-EulerIMU3(j,1);
%         EulerIMU4(j,3)=-EulerIMU4(j,3);
%     else
%         EulerIMU3(j,1)=EulerIMU3(j,1);
%     end
% end
% figure
%%角速度
% plot(t_IMU,-EulerIMU4(:,3),t_SLAM(2:2762),diff(EulerSLAM(:,1))./(t_SLAM(2:2762)-t_SLAM(1:2761))),title('yaw rate'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');
% figure
%角度
% plot(t_IMU,EulerIMU3(:,1),t_SLAM,EulerSLAM(:,1)),title('yaw'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');


% %IMU的角速度数据提取 18，19，20
% EulerIMU=zeros(2762,3);
% R_SLAM_IMU=[0 0 1;-1 0 0;0 -1 0];
% %angVinSLAM=zeros(2762,3);%%IMU到SLAM的坐标变化
% for i=1:2762
%     EulerIMU(i,1)=oxts{i}(18);
%     EulerIMU(i,2)=oxts{i}(19);
%     EulerIMU(i,3)=oxts{i}(20);
%     EulerIMUinSLAM(i,:)=transpose((R_IMU_SLAM)*transpose(EulerIMU(i,:)));
% end

%%时间转换为min
% t_IMU=timeIMU(:,5)*60+timeIMU(:,6)-54*60-39;
% t_SLAM=timeSLAM(:,5)*60+timeSLAM(:,6)-54*60-39;


% figure
% subplot(2,2,1),plot(t_IMU,-EulerIMU(:,1)./pi.*180,t_SLAM(2:2762),diff(EulerSLAM(:,3))./(t_SLAM(2:2762)-t_SLAM(1:2761))),title('roll rate'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');
% subplot(2,2,2),plot(t_IMU,-EulerIMU(:,2)./pi.*180,t_SLAM(2:2762),diff(EulerSLAM(:,2))./(t_SLAM(2:2762)-t_SLAM(1:2761))),title('pitch rate'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');
% subplot(2,2,3),plot(t_IMU,-EulerIMU4(:,3),t_SLAM(2:2762),diff(EulerSLAM(:,1))./(t_SLAM(2:2762)-t_SLAM(1:2761))),title('yaw rate'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');
% figure
% subplot(2,2,1),plot(t_IMU,EulerIMU1(:,3),t_SLAM,EulerSLAM(:,3)),title('roll'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');
% subplot(2,2,2),plot(t_IMU,EulerIMU1(:,2),t_SLAM,EulerSLAM(:,2)),title('pitch'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');
% %对比GPS得来得欧拉角转换角度范围前后效果
% subplot(1,2,1),plot(t_IMU,EulerIMU1(:,1),t_SLAM,EulerSLAM(:,1)),title('yaw'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');
% subplot(1,2,2),plot(t_IMU,EulerIMU2(:,1),t_SLAM,EulerSLAM(:,1)),title('yaw'),legend('IMU','SLAM'),ylabel('in degre'),xlabel('in seconde');

run_demoVelodyne ('C:\Users\LQH\Documents\shareUbuntu\imu\drive_0018_sync','C:\Users\LQH\Documents\shareUbuntu\imu\drive_0018_sync\calib')




