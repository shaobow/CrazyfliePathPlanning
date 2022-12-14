close all;
clear all;
clc;
addpath(genpath('./'));
addpath(genpath('../bin/'))

map_id = 4;
use_dstar = 1;
astar_path_w1 = cell(1);
astar_path_w2 = cell(1);
astar_path_w5 = cell(1);

switch map_id
    case 1
        % Plan path 1
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map1.txt', grid_size, grid_size, 0.0);
        start = {[0.0  -5.0 0.2]};
        stop  = {[6.0  18.0 2.0]};
    case 2
        % Plan path 2
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map2.txt', grid_size, grid_size, 0.0);
        start = {[5.0 5.0 9.0]};
        stop  = {[13.0 13.0 5.0]};
    case 3
        % Plan path 3
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map3.txt', grid_size, grid_size, 0.0);
        start = {[0.0, 5.0, 5.0]};
        stop  = {[20.0, 5.0, 5.0]};
    case 4
        % Plan path test
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map_test.txt', grid_size, grid_size, 0.0);
        start = {[8 0.2 2]};
        stop  = {[15 18 7]};
end

tic
  disp('Planning ...');
  [astar_path_w1{1},~,~,~] = cfPlanning(map_id, grid_size, margin_size, start{1}', stop{1}',~use_dstar, 1.0);
  [astar_path_w2{1},~,~,~] = cfPlanning(map_id, grid_size, margin_size, start{1}', stop{1}',~use_dstar, 2.0);
  [astar_path_w5{1},~,~,~] = cfPlanning(map_id, grid_size, margin_size, start{1}', stop{1}',~use_dstar, 5.0);
toc

load("dstar_data.mat")
dstar_path_raw = idx_to_points(map,dstar_path_data)+0.1;
dstar_path = {dstar_path_raw};

%% A*
trajectory_generator([], [], map, astar_path_w1);
trajectory = test_trajectory(map, astar_path_w1);

trajectory_generator([], [], map, astar_path_w2);
trajectory = test_trajectory(map, astar_path_w2);

trajectory_generator([], [], map, astar_path_w5);
trajectory = test_trajectory(map, astar_path_w5);

%% D* Lite
trajectory_generator([], [], map, dstar_path);
trajectory = test_trajectory(map, dstar_path);
