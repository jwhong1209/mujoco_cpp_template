clc; close all; clear;

%% load data
data_files = {
    '../csv/2504081856_stretch_hard_slow_contact_imp_fr3_control_state.csv', ...
    '../csv/2504081859_stretch_hard_slow_contact_mob_fr3_control_state.csv', ...
    '../csv/2504081857_stretch_hard_slow_contact_dob_fr3_control_state.csv'
};

dataset_names = {'Impedance Control', 'MOBI-FC', 'DOBI-FC'};

num_files = length(data_files);
datasets = repmat(struct('name', '', ...
    'q_mes', [], ...
    'dq_mes', [], ...
    'tau_des', [], ...
    'time', [], ...
    'q_init', [], ...
    'ddq_eul', [], ...
    'ddq_sosml', [], ...
    'tau_inv_dyn', [], ...
    'tau_mes', [], ...
    'tau_imp', [], ...
    'tau_fric', [], ...
    'ee_pos_des', [], ...
    'ee_pos_mes', [], ...
    'ee_rpy_des', [], ...
    'ee_rpy_mes', [], ...
    'ee_vel_des', [], ...
    'ee_vel_mes', [], ...
    'ee_acc_eul', [], ...
    'ee_acc_sosml', [], ...
    'F_ft', [], ...
    'F_d_fr3', [], ...
    'F_d_mob', [], ...
    'F_d_mob_high_fq', [], ...
    'F_d_dob', [], ...
    'F_d_dob_joint', [], ...
    'F_imp', [], ...
    'F_inertia', []), num_files, 1); 

for i = 1:num_files
    datasets(i) = load_data(data_files{i});
    datasets(i).name = dataset_names{i};
end

t_start = 0;
t_end = min(arrayfun(@(d) d.time(end), datasets));

%% Plot style setting
PS = PLOT_STANDARDS(); % The standard values for colors saved in PLOT_STANDARDS() will be accessed from the variable PS

color_fr3 = PS.Red3;
color_dob = PS.Blue4;
color_mob = PS.LGreen3;

lw_fr3 = 1.4;
lw_mob = 1.6;
lw_dob = 2;

ls_fr3 = '-.';
ls_mob = '--';
ls_dob = '-';

font = 'Times New Roman';
fs_legend = 14;
fs_label = 22;
fs_axis = 14;
fs_title = 20;

%% Normal Contact Force Estimation - Time Domain (ver 1)
figure(1); 
fig_Fz = tiledlayout(3, 1, "Padding", "compact", "TileSpacing", "compact");

fig_fold.fig = gcf;

text_xpos = t_start + 0.02 * (t_end - t_start);
text_ypos = 18;
% text_ypos = 50;

for i = 1:3
    contact_flag = datasets(i).F_ft(:,3) > 0.1;
    
    contact_regions = bwlabel(contact_flag); % export continuous range (requires Image Processing Toolbox)
    num_regions = max(contact_regions);

    nexttile;
    hold on; grid;
    
    fig_fold.p1 = plot(datasets(i).time, datasets(i).F_ft(:,3));

    fig_fold.p11 = plot(NaN, NaN, ...
        ls_dob,  'LineWidth', 2, 'Color', color_dob,  'DisplayName', 'FT Sensor');
    xlim([t_start t_end]);
    ylim([-5 22]);

    fig_fold.ax = gca;
    
    for r = 1:num_regions
        idx = find(contact_regions == r);
        if isempty(idx), continue; end
        t_region = datasets(i).time(idx);

        % if r > 9 && (i == 2)
        %     highlight_color = [1, 0, 0];
        % elseif r > 2 && i == 4
        %     highlight_color = [1, 0, 0];
        % else
        %     highlight_color = PS.Blue1;
        % end

        patch([t_region(1), t_region(end), t_region(end), t_region(1)], ...
              [fig_fold.ax.YLim(1), fig_fold.ax.YLim(1), fig_fold.ax.YLim(2), fig_fold.ax.YLim(2)], ...
              highlight_color, 'FaceAlpha', 0.08, 'EdgeColor', 'none');
    end

    text_string = dataset_names{i};
    fig_fold.plotText = text(text_xpos, text_ypos, text_string, ...
    'Interpreter', 'latex', ...
    'Color', PS.MyBlack, ...
    'BackgroundColor', PS.Grey1, ...
    'EdgeColor', PS.DGrey2, ...
    'LineWidth', 0.8, ...
    'Margin', 6, ...
    'FontName', font, ...
    'FontSize', 25, ...
    'FontWeight', 'bold');

    hold off;

    set(fig_fold.p1, 'LineStyle', '-',    'LineWidth', lw_dob);
end

xlabel(fig_Fz, 'Time (sec)', 'Interpreter', 'latex', 'FontSize', fs_label, 'FontName', font);
ylabel(fig_Fz, 'Normal Force [N]', 'Interpreter', 'latex', 'FontSize', fs_label);

%% Velocity
figure();
tiledlayout(3, 1, "Padding", "compact", "TileSpacing", "compact");
ylabels = {'Impedance Ctrl', 'MOBI-FC', 'DOBI-FC'};

fig_vel.fig = gcf;

text_xpos = t_start + 0.02 * (t_end - t_start);
text_ypos = 18;

for i = 1:3
    nexttile;
    hold on; grid;

    plot(datasets(i).time, datasets(i).ee_vel_des(:,3), 'LineStyle', '--', 'LineWidth', 2.0, ...
        'Color', 'black');
    plot(datasets(i).time, datasets(i).ee_vel_mes(:,3), 'LineStyle', '-', 'LineWidth', 1.5, ...
        'Color', 'red');
    xlim([t_start t_end]);
    ylabel(ylabels(i), 'Interpreter', 'latex', 'FontSize', fs_label);
    if i == 1
        legend('Desired', 'Measured', 'FontSize', 14, 'Interpreter', 'latex', 'Location', 'northeast');
    end
end
% title('Twist Tracking', 'FontSize', fs_title, 'FontName', font, 'Interpreter', 'latex');
xlabel('Time (sec)', 'Interpreter', 'latex', 'FontSize', fs_label, 'FontName', font);

