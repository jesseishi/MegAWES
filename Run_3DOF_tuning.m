% Copyright 2021 Delft University of Technology
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

%% Startup/Set cache folder
addpath(['Src' filesep 'Common'])
PreSim_startup();
clearvars                           % Clear workspace
close all                           % Close all figure windows
clc                                 % Clear command window

%% Set variables
Kite_DOF = 6; % Kite degrees of freedom
windspeed = 22;

[act, base_windspeed, constr, DE2019, ENVMT, Lbooth, ...
	loiterStates, params, simInit, T, winchParameter] = ...
	Get_simulation_params(windspeed, Kite_DOF);

try %#ok<TRYNC>
c = parcluster('local');
c.NumWorkers = 4; % Number of available cores
parpool(c, c.NumWorkers);
end

%% Set variables
Kps = 0:0.25:2.5;
Kis = 0:0.25:2.5;

% Js = logspace(4, 6.5, 32);

tot = 0;
clear simIn
for i = 1:numel(Kps)
    for j = 1:numel(Kis)

        params.kp_kite_Ft = Kps(i);  % TUNING
        params.ki_kite_Ft = Kis(j);  % TUNING

% % Abusing same code to loop through Js.
% for i = 1:numel(Js)
% 
%     winchParameter.inertia = Js(i);
        if Kite_DOF == 6
            model = 'Dyn_6DoF_v2_0_r2019b';
        else
            model = 'Dyn_PointMass_r2019b';
        end
        
        simInP = initAllStructs('Dyn_6DoF_v2_0_r2019b', base_windspeed, ...
            constr, ENVMT, Lbooth, loiterStates, DE2019, simInit, T, ...
            winchParameter,params,act);
    
        fname = string(params.kp_kite_Ft) + "_" + string(params.ki_kite_Ft);
        fname = replace(fname, ".", "p");
%     fname = sprintf('J%.0f', Js(i));
        simInP.UserString = fname;

        % Only add to the pool if it exists.
        code = exist(sprintf('Results\\6DOF_tuning\\data_parallel\\%s.mat', fname));
        if code == 0
            tot = tot+1;
            simIn(tot) = simInP; 
            fprintf('%s added to pool.\n', fname)
        else
            fprintf('%s already done.\n', fname)
        end
    end
end


%% Simulate.
tic
simOutAll = parsim(simIn);
t_end = toc;
fprintf('Walltime: %.1f s (%.1f s per simulation).\n', t_end, t_end/tot);

% save("Results\cover_image\data_parallel\all\simOutAll.mat", 'simOutAll')
save("Results/6DOF_tuning/data_parallel/all/simOutAll.mat", 'simOutAll');

%% Save them all also if they have errors.
% The runs with errors will be filtered out later.
for j = 1:numel(simOutAll)
    simOut = simOutAll(j);

    fname = simOutAll(j).SimulationMetadata.UserString;
    short_name = fname;
    K = replace(short_name, 'p', '.');
    K = split(K, '_');
    Kp = str2num(K(1));
    Ki = str2num(K(2));    
    name = sprintf("K_p = %.2f, K_i = %.2f", Kp, Ki);
%     name = short_name;

%     save("Results\cover_image\data_parallel\" + fname + ".mat", 'simOut', 'short_name', 'name')
    save("Results/6DOF_tuning/data_parallel/" + fname + ".mat", 'simOut', 'short_name', 'name')
end
