close all;
clear all;
clc;
addpath(genpath('./'));
addpath(genpath('../bin/'))

map_id = 2;
use_dstar = 1;
astar_path = cell(1);
dstar_path = cell(1);

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
        start = {[5.0 5.0 3.0]};
        stop  = {[13.0 13.0 3.0]};
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

total_num_step = [];
total_num_node = [];
total_run_time = [];
total_cost = [];
for i = 1:10
    [path, num_step, num_node, run_time] = cfPlanning(map_id, grid_size, margin_size, start{1}', stop{1}', ~use_dstar, 5);
    total_num_step = [total_num_step; num_step];
    total_num_node = [total_num_node; num_node];
    total_run_time = [total_run_time; run_time];

    path_len = sum(sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2)));
    path_seg_len = sqrt(sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2)));
    cost_all = cumsum(path_seg_len);
    cost = cost_all(end);
    total_cost = [total_cost; cost];
end

avg_num_step = mean(total_num_step);
avg_num_node = mean(total_num_node);
avg_run_time = mean(total_run_time);
avg_cost = mean(total_cost)
%% 
% map1 63.2626
% map2 51.9376

