% close all;
% clear all;
% clc;
addpath(genpath('./'));
addpath(genpath('../bin/'))
load("exp_data_con6.mat")

map_id = 3;
rng = 10;
use_dstar = 1;
astar_path = cell(1);
dstar_path = cell(1);

switch map_id
    case 1
        % Plan path 1
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map1.txt', grid_size, grid_size, 0.0);
        if rng == 10
            path_raw = map1_rg10_con26;
        else
            path_raw = map1_rg1_con26;
        end
    case 2
        % Plan path 2
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map2.txt', grid_size, grid_size, 0.0);
        if rng == 10
            path_raw = map2_rg10_con26;
        else
            path_raw = map2_rg1_con26;
        end
    case 3
        % Plan path 3
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map3.txt', grid_size, grid_size, 0.0);
        if rng == 10
            path_raw = map3_rg10_con26;
        else
            path_raw = map3_rg1_con26;
        end
    case 4
        % Plan path test
        grid_size = 0.2;
        margin_size = 0.2;
        map = load_map('maps/map_test.txt', grid_size, grid_size, 0.0);
        if rng == 10
            path_raw = map4_rg10_con26;
        else
            path_raw = map4_rg1_con26;
        end        
end

path = idx_to_points(map, path_raw);
path_len = sum(sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2)));
path_seg_len = sqrt(sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2)));
cost_all = cumsum(path_seg_len);
cost = cost_all(end)

%%
% map1 range1 con26 112.7909
% map1 range10 con26 62.5006
% map2 range1 con26 
% map2 range10 con26 
% map3 range1 con26 330.5887
% map3 range10 con26 73.8833
% map4 range1 con26 
% map4 range10 con26 