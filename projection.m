%%初始经纬度：49.04951961077 8.3965961639946 113.02558135986
% lat:   latitude of the oxts-unit (deg)
% lon:   longitude of the oxts-unit (deg)
% alt:   altitude of the oxts-unit (m)
% mstruct=defaultm('lambert');
% mstruct.origin=[49.04951961077,8.3965961639946,113]; 
% mstruct.geoid=[6378137,0.0818191908426215];
% mstruct.mapparallels=[49.049549356313,8.3965879935924,113];
% mstruct=defaultm(mstruct);
%%地理坐标投影到直角坐标:
% [x,y] =projfwd(mstruct,49.04951961077,8.3965961639946);
% %直角坐标投影到地理坐标:
% [lat,lon]=projinv(mstruct,x,y);
foward=zeros(1,2762);
horizon=zeros(1,2762);
% latlonIMU=zeros(2762,2);
% for i=1:2762
%     [latlonIMU(i,1),latlonIMU(i,2)]=projfwd(mstruct,oxts{i}(1),oxts{i}(2));
% end
% [x,y]=projfwd(mstruct,49.04951961077,8.3965961639946);
% foward=poseSLAM(3,4,:);
% horizon=poseSLAM(1,4,:);
% % 问题发现：SLAM的数据只是前进方向的里程累积值
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

ProjectionIMU=zeros(2762,3);
for i=1:2762
    ProjectionIMU(i,2)=oxts{i}(1);
    ProjectionIMU(i,1)=oxts{i}(2);
    ProjectionIMU(i,3)=oxts{i}(3);
end
% for i=1:2762
% foward(i)=poseIMU{i}(1,4);
% horizon(i)=poseIMU{i}(2,4);
% end
% ProjectionIMU=zeros(2762,3);
% for i=1:2762
%     [ProjectionIMU(i,2),ProjectionIMU(i,1)]=minvtran(mstruct,foward(i),horizon(i));
%      ProjectionIMU(i,3)=113.02558135986;
% end
% dlmwrite('dataIMU.txt', ProjectionIMU);
fid = fopen('C:\Users\LQH\Documents\shareUbuntu\imu\devkit\matlab\dataIMU.txt','w');
for k = 1:2762
for j = 1:2
fprintf(fid,'%1.6f',ProjectionIMU(k,j));
fprintf(fid,'%s',',');
% fprintf(fid,'%s\t','一行finish');
end
fprintf(fid,'%1.6f\n',ProjectionIMU(k,3));
% fprintf(fid,'%s\n',''------------------------------------------'');
end 
fclose(fid);