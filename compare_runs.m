%% Reset.
clear;
close all;

addpath('Src\Common\GroundStation\')

%% Load all results and setup stuff for plotting and saving.
figure(1)
colors = get(gca, 'colororder');

savedir = "Results/3DOF_tuning/figures/";
addpath('Results\3DOF_tuning\data\')
matfiles = dir('Results\3DOF_tuning\data\*.mat');


% runs_dict = dictionary(...
%     'baseline.mat', dictionary('name', 'Baseline', 'short_name', 'B', 'color_i', 1), ...
%     'feedforward_winch_control.mat', dictionary('name', 'Feedforward winch control', 'short_name', 'FF', 'color_i', 2), ...
%     'feedforward_winch_control_my_curve.mat', dictionary('name', 'Feedforward winch control, heavy strategy', 'short_name', 'FFheavy', 'color_i', 5), ...
%     'kite_tether_force_control.mat', dictionary('name', 'Kite tether force control', 'short_name', 'KTFC', 'color_i', 5), ...
%     'kite_tether_force_control_0p25_0p25.mat', dictionary('name', 'Kite tether force control 0.25 0.25', 'short_name', 'KTFC2', 'color_i', 4), ...
%     'kite_tether_force_control_1p0_1p25.mat', dictionary('name', 'Kite tether force control 1.0 1.25', 'short_name', 'KTFC3', 'color_i', 5), ...
%     'kite_tether_force_control_1p25_1p5.mat', dictionary('name', 'Kite tether force control 1.25 1.5', 'short_name', 'KTFC4', 'color_i', 6), ...
%     'kite_tether_force_control_1p0_1p5.mat', dictionary('name', 'Kite tether force control 1.0 1.5', 'short_name', 'KTFC5', 'color_i', 7), ...
%     'kite_tether_force_control_1p5_1p25.mat', dictionary('name', 'Kite tether force control 1.5 1.25', 'short_name', 'KTFC6', 'color_i', 2), ...
%     'kite_tether_force_control_1p0_1p0.mat', dictionary('name', 'Kite tether force control 1.0 1.0', 'short_name', 'KTFC7', 'color_i', 3), ...
%     'kite_tether_force_control_1p25_1p0.mat', dictionary('name', 'Kite tether force control 1.25 1.0', 'short_name', 'KTFC8', 'color_i', 4), ...
%     'kite_tether_force_control_1p0_0p75.mat', dictionary('name', 'Kite tether force control 1.0 0.75', 'short_name', 'KTFC9', 'color_i', 5), ...
%     'kite_tether_force_control_my_curve.mat', dictionary('name', 'Kite tether force control, heavy strategy', 'short_name', 'KTFCheavy', 'color_i', 7), ...
%     'winch_sizing_constant_x100.mat', dictionary('name', 'Winch sizing constant \times 100', 'short_name', 'x100', 'color_i', 4), ...
%     'winch_sizing_constant_x100_my_curve.mat', dictionary('name', 'Winch inertia \times 100', 'short_name', 'x100heavy', 'color_i', 4));

% For showing the tuning process.
runs_dict = dictionary(...
    "1p0_1p0.mat", dictionary('name', 'K_p = 1.0, K_i = 1.0') ...
    );



for i = 1:length(matfiles)
    fname = matfiles(i).name;
    run_dict = runs_dict(fname);

    temp = load(fname);
    runs(i) = temp.simOut;


    runs(i).name = run_dict('name');
    runs(i).short_name = run_dict('short_name');
    i_c = str2num(run_dict('color_i'));
    runs(i).color = colors(i_c, :);
end

% Select which ones we want to look at.
i_select = 1:length(matfiles);
i_select = [2, 4];
runs = runs(i_select);

% String representation for storing figures. Just put all i's after each
% other.
runs_short_name = sprintf('%s_', runs.short_name);
runs_short_name = 'tuning_';

%% Get the traction part.
% I only want to analyse the first traction phase, so I can save an index
% that indexes this nicely.
for i = 1:length(runs)
    run = runs(i);
    % Find start and finish.
    start_first_traction_idx = find(run.sub_flight_state.Data == 4, 1, 'first');
    start_first_traction = run.sub_flight_state.Time(start_first_traction_idx);
    end_first_traction_idx = find(run.sub_flight_state.Data == 5, 1, 'first');
    end_first_traction = run.sub_flight_state.Time(end_first_traction_idx);

    % Add margin.
    % This really cleans it up!
    % I think it's ok to do this because I don't focus on the transitions
    % in my work, so they're going to be a bit rough.
    margin_sec = 15;  % Tether force has then completely stabilized.
    start_first_traction = start_first_traction + margin_sec;
    end_first_traction = end_first_traction - margin_sec;

    % Since all signals are logged at the same rate, this index works for
    % all signals.
    idx = (run.v_reel.Time > start_first_traction) & (run.v_reel.Time < end_first_traction);
    runs(i).first_traction_idx = idx;

    idx_alpha = (run.alpha.Time > start_first_traction) & (run.alpha.Time < end_first_traction);
    runs(i).first_traction_alpha_idx = idx_alpha;
end


%% vr-Ft curve.
figure(1)
tiledlayout(1,1, 'TileSpacing', 'none', 'Padding', 'none'); nexttile;  % For tight_layout().

% Ideal vr-Ft curve.
% v_ro = -5:0.1:15;
% Ft_ref = Ft_ref_with_mass(v_ro, false, 0.5e6, 1e6);
% plot(v_ro, Ft_ref, ':k', 'LineWidth', 1);
% hold on

v_ro = 0:0.1:15;
C = 7.7008e3;  % Hard-coded for MegAWES.
Ft_ref_massless = 4 * C * v_ro.^2;
plot(v_ro, Ft_ref_massless, '--k', 'LineWidth', 1);
hold on
set(gca,'ColorOrderIndex',1)

for run = runs
    plot(run.v_reel.Data(run.first_traction_idx), ...
         run.TetherForce.Data(run.first_traction_idx), ...
         'Color', run.color)
%     plot(run.v_reel.Data, ...
%          run.TetherForce.Data, ...
%          'Color', run.color)
end


ylabel('Tether force, N')
xlabel('Reel-out speed, m/s')
% Ft_star_smooth_N = smoothdata(Ft_star_N, 'gaussian', 25);
% plot(vr_mps, Ft_star_smooth_N, ':r', 'LineWidth', 3);
ylim([0, 2.0e6])
% legend('heavy strategy', 'massless strategy', runs.name, 'Location', 'SouthOutside')
legend('ideal', runs.name, 'Location', 'SouthOutside')
hold off



AIAA_formatting(gcf, 0.6, 0.5)
saveas(gcf, savedir+runs_short_name+"_vrFt.png")




%% Tether force.
figure(2)
tiledlayout(1,1, 'TileSpacing', 'none', 'Padding', 'none'); nexttile;  % For tight_layout().


for run = runs
    plot(run.TetherForce.Time(run.first_traction_idx), ...
        run.TetherForce.Data(run.first_traction_idx), ...
        'Color', run.color)
    hold on
end

ylabel('Tether force, N')
xlabel('Time, s')
legend(runs.name, 'Location', 'SouthEast')
% xlim([50, 150])
hold off


AIAA_formatting(gcf, 0.6, 0.6/1.6)
saveas(gcf, savedir+runs_short_name+"_Ft.png")

%% Angle of attack
figure(3)
for run = runs
    % first_run_traction_idx does not work for alpha because its sampled at
    % a different rate...
    plot(run.alpha.Time(run.first_traction_alpha_idx), ...
        rad2deg(run.alpha.Data(run.first_traction_alpha_idx)), ...
        'Color', run.color)
    hold on
end

ylabel('Angle of attack, deg')
xlabel('Time, s')
legend(runs.name, 'Location', 'SouthEast')
grid on 
hold off

AIAA_formatting(gcf, 0.6, 0.6/1.6)
saveas(gcf, savedir+runs_short_name+"_alpha.png")

%% Power output
figure(4)
tiledlayout(1,1, 'TileSpacing', 'none', 'Padding', 'none'); nexttile;  % For tight_layout().
% subplot(2, 1, 1)
for run = runs
    plot(run.P_mech.Time(run.first_traction_idx), ...
        run.P_mech.Data(run.first_traction_idx), ...
        'Color', run.color)
    hold on
    % TODO: how to do a fair comparison of the mean? -> entire traction
    % phase is maybe better. Or traction+retraction (but retraction doesn't
    % work...) so Energy during traction? Also for a fair comparison they
    % must use similar setpoint for max tether force.

    P_mean = mean(run.P_mech.Data(run.first_traction_idx));
    P_max = max(run.P_mech.Data(run.first_traction_idx));
end

ylabel('Mechanical Power, W')
xlabel('Time, s')
legend(runs.name)
hold off

AIAA_formatting(gcf, 0.6, 0.6/1.6)
saveas(gcf, savedir+runs_short_name+"_P_mech.png")

% subplot(2, 1, 2)
% for run = runs
%     plot(run.P_elec_out.Time(run.first_traction_idx), run.P_elec_out.Data(run.first_traction_idx))
%     hold on
% end
% ylabel('electrical power [W]')
% xlabel('time [s]')
% legend(runs.name)
% grid on
% hold off

%% Winch.
% ! In winch dynamics the acceleration gets saturated, but not in the
% signal!
figure(5)
subplot(2, 1, 1)
for run = runs
    plot(run.v_reel.Time(run.first_traction_idx), ...
         run.v_reel.Data(run.first_traction_idx), ...
        'Color', run.color)
    hold on
end
legend(runs.name)
grid on
ylabel('Reel-out speed, m/s')
xlabel('Time, s')
ylim([-10, 20])

subplot(2, 1, 2)
for run = runs
    plot(run.winch_a.Time(run.first_traction_idx), ...
         run.winch_a.Data(run.first_traction_idx), ...
         'Color', run.color)
    hold on
end
ylim([-25, 25])
grid on
legend(runs.name)
ylabel('Reel-out acceleration, m/s^2')
xlabel('Time, s')
ylim([-10, 10])

hold off


AIAA_formatting(gcf, 1.0, 0.7)
saveas(gcf, savedir+runs_short_name+"_winch.png")

%% Power output and tether force distribution (the limits of the system).
% TODO: Add average power (mech and elec).
figure(6)

x_power = [];
power_mean = [];
x_elec_power = [];
elec_power_mean = [];
x_tether = [];
g = [];
g_labels = [];

% The constant sampling times are saving us here a LOT. If the sampling
% time is not constant these kind of distribution plots need to be adjusted
% for the amount of time that passes at a certain sample!
for i=1:length(runs)
    powerData = runs(i).P_mech_out.Data(runs(i).first_traction_idx);
    elecPowerData = runs(i).P_elec_out.Data(runs(i).first_traction_idx);
    tetherData = runs(i).TetherForce.Data(runs(i).first_traction_idx);

    x_power = [x_power; powerData];
    power_mean = [power_mean; mean(powerData)];
    x_elec_power = [x_elec_power; elecPowerData];
    elec_power_mean = [elec_power_mean; mean(elecPowerData)];
    x_tether = [x_tether; tetherData];
    
    g = [g; i*ones(length(powerData), 1)];
    g_labels = [g_labels; string(runs(i).name)];
end

g_labels = categorical(g, 1:length(runs), g_labels);


subplot(1, 3, 1)
boxchart(g_labels, x_tether)
grid on
ylabel('tether force [N]')

subplot(1, 3, 2)
boxchart(g_labels, x_power)
grid on
ylabel('power [W]')
hold on
plot(power_mean, '-o')

subplot(1, 3, 3)
boxchart(g_labels, x_elec_power)
grid on
ylabel('electrical power [W]')
hold on
plot(elec_power_mean, '-o')

% Only mech power distribution.
figure(7)
tiledlayout(1,1, 'TileSpacing', 'none', 'Padding', 'none'); nexttile;  % For tight_layout().

boxchart(g_labels, x_power ./ 1e6)
ylabel('Mechanical power, MW')
hold on
plot(power_mean ./ 1e6, '-o')
AIAA_formatting(gcf, 0.5, 0.5)
saveas(gcf, savedir+runs_short_name+"_P_mech_distribution.png")


%% Mean cycle power.
% TODO: check this calculation!
% TODO: mechanical vs electrical power.
% Taken from the two retraction phases of baseline.
E_mech_retraction = (-101.12e6 - 86.42e6) / 2;
E_elec_retraction = E_mech_retraction * (1/0.9/0.95);
t_retraction = (45.93 + 41.72) / 2;
P_bar_mech_retraction = E_mech_retraction / t_retraction;

for run=runs

    fprintf('%s \n', run.name)

    % Find start and finish.
    start_first_traction_idx = find(run.sub_flight_state.Data == 4, 1, 'first');
    start_first_traction = run.sub_flight_state.Time(start_first_traction_idx);
    end_first_traction_idx = find(run.sub_flight_state.Data == 5, 1, 'first');
    end_first_traction = run.sub_flight_state.Time(end_first_traction_idx);
    t_traction = end_first_traction - start_first_traction;
    t_cycle = t_traction+t_retraction;

    P_bar_mech_traction = mean(run.P_mech_out.Data(start_first_traction_idx:end_first_traction_idx));
    P_bar_mech_cycle = P_bar_mech_traction * t_traction / t_cycle + P_bar_mech_retraction * t_retraction / t_cycle;
    fprintf('%.2f MW mean traction mechanical power (%.1f %% of baseline).\n', P_bar_mech_traction/1e6, P_bar_mech_traction/7.2273/1e6*100);
    fprintf('%.2f MW mean cycle mechanical power (%.1f %% of baseline).\n', P_bar_mech_cycle/1e6, P_bar_mech_cycle/5.1036/1e6*100);

    % 2nd attempt (to check if valid). -> YUP
    start_first_traction_idx = find(run.sub_flight_state.Data == 4, 1, 'first');
    start_first_traction = run.sub_flight_state.Time(start_first_traction_idx);
    end_first_traction_idx = find(run.sub_flight_state.Data == 5, 1, 'first');
    end_first_traction = run.sub_flight_state.Time(end_first_traction_idx);
    t_traction = end_first_traction - start_first_traction;
    t_cycle = t_traction+t_retraction;

    P_mech_traction = run.P_mech.Data(start_first_traction_idx:end_first_traction_idx);
    P_elec_traction = P_mech_traction;
    P_elec_traction(P_elec_traction>0) = P_elec_traction(P_elec_traction>0) * 0.9;
    P_elec_traction(P_elec_traction<0) = P_elec_traction(P_elec_traction<0) * (1/0.9*0.95);

    E_mech_traction = trapz(P_mech_traction) * mean(diff(run.P_mech.Time));
    P_mech_cycle_bar = (E_mech_traction + E_mech_retraction) / (t_cycle);

    E_elec_traction = trapz(P_elec_traction) * mean(diff(run.P_mech.Time));
    P_elec_traction_bar = E_elec_traction / t_traction;
    P_elec_cycle_bar = (E_elec_traction + E_elec_retraction) / (t_cycle);

    fprintf('%.2f MW mean traction electrical power (%.1f %% of baseline).\n', P_elec_traction_bar/1e6, P_elec_traction_bar/6.4466/1e6*100);
    fprintf('%.2f MW mean cycle electrical power (%.1f %% of baseline).\n', P_elec_cycle_bar/1e6, P_elec_cycle_bar/4.4176/1e6*100);

    fprintf('\n')

end





%% Tether force and angle of attack:
figure(8)
tiledlayout(1,1, 'TileSpacing', 'none', 'Padding', 'none'); nexttile;  % For tight_layout().

for run = runs

    alpha = run.alpha;
    Ft = resample(run.TetherForce, run.alpha.Time);

    plot(rad2deg(alpha.Data(run.first_traction_alpha_idx)), Ft.Data(run.first_traction_alpha_idx), ...
        'Color', run.color)
    hold on
end


xlabel('Angle of attack, deg')
ylabel('Tether force, N')
legend(runs.name)
hold off

AIAA_formatting(gcf, 0.6, 0.6/1.6)
% saveas(gcf, savedir+runs_short_name+"_alpha_Ft.png")






