function [ ] = PlotCarLocation( )
% Demonstartion of Localization and Mapping
% using Minimal and Inexpensive Components
%
% Ayush Sinha
% ayush7.sinha@gmail.com
% Date: 15 April 2018
% Summary:
% Plots Robot car's location and creates a map
% as the car explores the world.
% Left map plots desired (or reference) car location and creates
% the map acoordingly, Center corresponds to observed location 
%(through IR readings) and Right map uses location through Kalman Filter
%
%                  World
%          ---------------------
%          | 16 | 15 | 14 | 13 |
%          ---------------------
%          |  9 | 10 | 11 | 12 |
%          ---------------------
%          |  8 |  7 |  6 |  5 |
%          ---------------------
%          | 1  |  2 |  3 |  4 |
%          ---------------------
%% Make Grids
[X,Y]=meshgrid(1:5);
figure; hold on;
set(gcf, 'Position', get(0, 'Screensize'));
subplot(1,3,1);
plot(X,Y,'k'); hold on
plot(Y,X,'k');  axis square; axis off
title('Trajectory through Velocity Model');
subplot(1,3,2);
plot(X,Y,'k'); hold on
plot(Y,X,'k');  axis square; axis off
title('Trajectory through IR Measurements');
subplot(1,3,3);
plot(X,Y,'k'); hold on
plot(Y,X,'k');  axis square; axis off
title('Trajectory through Kalman Filter');
pause (1e-9); % to see plot while code's running
%% Assign Cell numbers on Grid (World)
cell_loc = zeros(16,2);
% row   = cell number in MATLAB
% col 1 = x_loc
% col 2 = y_loc
for i = 1:4
   cell_loc(i,:) = [i + 0.5, 1.5];  
end
for i = 5:8
   cell_loc(i,:) = [9.5 - i, 2.5]; 
end
for i = 9:12
   cell_loc(i,:) = [i - 7.5, 3.5];  
end
for i = 13:16
   cell_loc(i,:) = [17.5 - i, 4.5]; 
end
%% Plotting
%start serial
s = serial('COM3', 'BaudRate', 9600);
fopen(s);

% initialise old_loc to zero-th cell
old_ref_loc = 1;
old_obs_loc = 1;
old_fil_loc = 1;
subplot(1,3,1);
loc = plot(cell_loc(old_ref_loc,1),cell_loc(old_ref_loc,2),'-bs','MarkerSize',25,'MarkerFaceColor','b');
subplot(1,3,2);
oloc = plot(cell_loc(old_obs_loc,1),cell_loc(old_obs_loc,2),'-rs','MarkerSize',25,'MarkerFaceColor','r');
subplot(1,3,3);
floc = plot(cell_loc(old_fil_loc,1),cell_loc(old_fil_loc,2),'-gs','MarkerSize',25,'MarkerFaceColor','g');
pause (1e-9);

driving = 1; % flags when car is driving

% plot new locations
while driving
    % get new location from serial
    C=fscanf(s);
    c_str = regexp(C, '?', 'split');
    
    new_ref_loc = str2double(c_str(1));
    new_obs_loc = str2double(c_str(2));
    new_fil_loc = str2double(c_str(3));
    obstacle_found = round(str2double(c_str(3)));
    
    % MATLAB array indices start from 1, hence +1
    new_ref_loc = round(new_ref_loc) + 1;
    new_obs_loc = round(new_obs_loc) + 1;
    new_fil_loc = round(new_fil_loc) + 1;
    
    % plot robot reference traj
    subplot(1,3,1);
    if (obstacle_found == 1) % blacken cell if obstacle found
       plot(cell_loc(new_ref_loc,1),cell_loc(new_ref_loc,2),'-ks','MarkerSize',70,'MarkerFaceColor','k'); 
    end
    plot([cell_loc(old_ref_loc,1) cell_loc(new_ref_loc,1)],[cell_loc(old_ref_loc,2) cell_loc(new_ref_loc,2)],'-b','LineWidth', 7);
    delete(loc)
    loc = plot(cell_loc(new_ref_loc,1),cell_loc(new_ref_loc,2),'-bs','MarkerSize',25,'MarkerFaceColor','b');
    
    % plot robot observed traj
    subplot(1,3,2);
    if (obstacle_found == 1) % blacken cell if obstacle found
        plot(cell_loc(new_obs_loc,1),cell_loc(new_obs_loc,2),'-ks','MarkerSize',70,'MarkerFaceColor','k'); 
    end
    plot([cell_loc(old_obs_loc,1) cell_loc(new_obs_loc,1)],[cell_loc(old_obs_loc,2) cell_loc(new_obs_loc,2)],'-r','LineWidth', 7);
    delete(oloc)
    oloc = plot(cell_loc(new_obs_loc,1),cell_loc(new_obs_loc,2),'-rs','MarkerSize',25,'MarkerFaceColor','r');
    
    % plot robot kalman-fltered traj
    subplot(1,3,3);
    if (obstacle_found == 1) % blacken cell if obstacle found
        plot(cell_loc(new_fil_loc,1),cell_loc(new_fil_loc,2),'-ks','MarkerSize',70,'MarkerFaceColor','k'); 
    end
    plot([cell_loc(old_fil_loc,1) cell_loc(new_fil_loc,1)],[cell_loc(old_fil_loc,2) cell_loc(new_fil_loc,2)],'-g','LineWidth', 7);
    delete(floc)
    floc = plot(cell_loc(old_fil_loc,1),cell_loc(old_fil_loc,2),'-gs','MarkerSize',25,'MarkerFaceColor','g');
    pause (1e-9); % to see figure while plotting
    
    % update old_loc
    old_ref_loc = new_ref_loc;
    old_obs_loc = new_obs_loc;
    old_fil_loc = new_fil_loc;
    
    % end plotting when traj finished
    if new_ref_loc == 16
        driving = 0;
    end
end
fclose(s);
hold off
end

