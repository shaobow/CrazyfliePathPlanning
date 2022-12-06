close all;
clear all;
clc;
addpath(genpath('./'));
addpath(genpath('../bin/'))

map_id = 1;
path = cell(1);

switch map_id
    case 1
        % Plan path 1
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map1.txt', grid_size, grid_size, 0.0);
        start = {[0.0  -4.9 0.2]};
        stop  = {[6.0  17.0 5.0]};
        tic
          disp('Planning ...');
          path{1} = cfPlanning(1, grid_size, margin_size);
        toc
        
        plot_path(map, path{1});
    case 3
        % Plan path 3
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map3.txt', grid_size, grid_size, 0.0);
        start = {[0.0, 5.0, 5.0]};
        stop  = {[20.0, 5.0, 5.0]};
        nquad = length(start);
        tic
            disp('Planning ...');
            path{1} = cfPlanning(3, grid_size, margin_size);
        toc
        plot_path(map, path{1});

end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(map, path);

