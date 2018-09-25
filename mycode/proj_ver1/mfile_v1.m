function [ ] = PlotCarLocation( )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% make grid
[X,Y]=meshgrid(1:5);
figure; hold on;
plot(X,Y,'k');
plot(Y,X,'k'); axis off; axis square
pause (1e-9); % to see plot while code's running

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

%start serial
s = serial('COM3', 'BaudRate', 9600);
fopen(s);
%pause(0.5);

% initialise old_loc to zero-th cell
old_ref_loc = 1;
old_obs_loc = 1;
loc = plot(cell_loc(old_ref_loc,1),cell_loc(old_ref_loc,2),'-bs','MarkerSize',25,'MarkerFaceColor','b');
oloc = plot(cell_loc(old_obs_loc,1)+0.2,cell_loc(old_obs_loc,2)+0.2,'-rs','MarkerSize',25,'MarkerFaceColor','r');
pause (1e-9);

% plot new locations
driving = 1;

while driving
    % get new location from serial
%     new_ref_loc = str2double(fscanf(s)); 
%     pause(1e-3);
%     new_obs_loc = str2double(fscanf(s));

    C=fscanf(s);
    c_str = regexp(C, '?', 'split');
    
    new_ref_loc = str2double(c_str(1));
    new_obs_loc = str2double(c_str(2));
    
    new_ref_loc = round(new_ref_loc) + 1;
    new_obs_loc = round(new_obs_loc) + 1;
    
    % plot robot traj
    plot([cell_loc(old_ref_loc,1) cell_loc(new_ref_loc,1)],[cell_loc(old_ref_loc,2) cell_loc(new_ref_loc,2)],'-b','LineWidth', 7);
    delete(loc)
    loc = plot(cell_loc(new_ref_loc,1),cell_loc(new_ref_loc,2),'-bs','MarkerSize',25,'MarkerFaceColor','b');
    
    plot([cell_loc(old_obs_loc,1)+0.2 cell_loc(new_obs_loc,1)+0.2],[cell_loc(old_obs_loc,2)+0.2 cell_loc(new_obs_loc,2)+0.2],'-r','LineWidth', 7);
    delete(oloc)
    oloc = plot(cell_loc(new_obs_loc,1)+0.2,cell_loc(new_obs_loc,2)+0.2,'-rs','MarkerSize',25,'MarkerFaceColor','r');
    pause (1e-9);
    
    % update old_loc
    old_ref_loc = new_ref_loc;
    old_obs_loc = new_obs_loc;
    
    % end plotting when traj finished
    if new_ref_loc == 16
        driving = 0;
    end
end
fclose(s);
hold off
end

