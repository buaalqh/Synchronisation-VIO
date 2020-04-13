%%初始经纬度：49.04951961077 8.3965961639946 113.02558135986
% lat:   latitude of the oxts-unit (deg)
% lon:   longitude of the oxts-unit (deg)
% alt:   altitude of the oxts-unit (m)
% mstruct=defaultm('mercator');
% mstruct.origin=[49.049519610770000,8.396596163994600,99.85]; %高度决定mercator的投影方向！！！
% mstruct.geoid=[6378137.0,0.01745329251994328];
% mstruct=defaultm(mstruct);
%%地理坐标投影到直角坐标:
% [x,y] =projfwd(mstruct,49.04951961077,8.3965961639946);
% %直角坐标投影到地理坐标:该函数转换太慢
% [lat,lon]=projinv(mstruct,x,y);
foward=zeros(1,2762);
horizon=zeros(1,2762);
% latlonIMU=zeros(2762,2);
% for i=1:2762
%     [latlonIMU(i,1),latlonIMU(i,2)]=projfwd(mstruct,oxts{i}(1),oxts{i}(2));
% end
% [x,y]=projfwd(mstruct,49.04951961077,8.3965961639946);

%%SLAM轨迹mercator方法投影到google earth上
%%poseSLAM见process.m
% foward=poseSLAM(3,4,:);
% horizon=poseSLAM(1,4,:);
% % 问题发现：SLAM的数据只是前进方向的里程累积值，需要转换后再投影，（待日后解决）
sens=0;
for i=2:1000
    j=i+48;
    if j<=1000&&abs(EulerSLAM(j,1)-EulerSLAM(i,1))>30
        sens=sens+EulerSLAM(j,1)-EulerSLAM(i,1);
    end
    foward(i)=foward(i-1)+(poseSLAM(3,4,i)-poseSLAM(3,4,i-1))*cos(sens);
    horizon(i)=horizon(i-1)+(poseSLAM(3,4,i)-poseSLAM(3,4,i-1))*sin(sens);
end
figure,plot(horizon,foward);
% ProjectionSLAM=zeros(2762,3);
% for i=1:2762
%     [ProjectionSLAM(i,2),ProjectionSLAM(i,1)]=minvtran(mstruct,foward(i),horizon(i));
%      ProjectionSLAM(i,3)=113.02558135986;
% end

% fid = fopen('C:\Users\LQH\Documents\shareUbuntu\imu\devkit\matlab\dataSLAM.txt','w');
% for k = 1:2762
% for j = 1:3
% fprintf(fid,'%12.20f\t',ProjectionSLAM(k,j));
% % fprintf(fid,'%s\t','一行finish');
% end
% fprintf(fid,'%s\n',''------------------------------------------'');
% end 
% fclose(fid);

%GPS数据输出
% % ProjectionIMU1=zeros(2762,3);
% % for i=1:2762
% %     ProjectionIMU1(i,2)=oxts{i}(1);
% %     ProjectionIMU1(i,1)=oxts{i}(2);
% %     ProjectionIMU1(i,3)=oxts{i}(3);
% % end

%%IMU欧拉角mercator投影到google earth上
% for i=1:2762
% foward(i)=poseIMU{i}(1,4);
% horizon(i)=poseIMU{i}(2,4);
% end
% ProjectionIMU=zeros(2762,3);
% for i=1:2762
%     [ProjectionIMU(i,2),ProjectionIMU(i,1)] = minvtran(mstruct,foward(i),horizon(i));
%      ProjectionIMU(i,3)=113;
% end
% dlmwrite('dataIMU.txt', ProjectionIMU);

%%IMU数据按KML经纬度格式输出
% fid = fopen('C:\Users\LQH\Documents\shareUbuntu\imu\devkit\matlab\dataIMU.txt','w');
% for k = 1:2762
% for j = 1:2
% fprintf(fid,'%1.6f',ProjectionIMU(k,j));
% fprintf(fid,'%s',',');
% % fprintf(fid,'%s\t','一行finish');
% end
% fprintf(fid,'%1.6f\n',ProjectionIMU(k,3));
% % fprintf(fid,'%s\n',''------------------------------------------'');
% end 
% fclose(fid);