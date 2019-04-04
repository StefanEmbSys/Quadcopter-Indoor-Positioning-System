
% variables for the storage of the camera view (written to workspace from
% c++ code
az = -37.5;
el = 30;
% variables for 3D-Space written to workspace from c++ code
% x, y, z = 0;

% store camera view
[az,el] = view;

% plot 3D chart
stem3(x,y,z,':bo','filled');

% restore the previous stored view, as stem3 will make default view
view(az,el);

% name axis and graph
xlabel('x','Fontsize',10); 
ylabel('y','Fontsize',10); 
zlabel('z','Fontsize',10); 
title('IPS','Fontsize',15); 

% set fixed graph size for the IPS
axis([-1 1 -1 1 0 2]); 
axis square
set(gca,'Fontsize',5);

%pause(0.2);

