
%%%                           Copyright
%%% Use and modify the project as you want. Do not cancel the first five lines.

%%% Author: Giulio Savian




%% PLOT AND SIMPLE ANALYSIS OF THE DATA ACQUIRED USING J-SCOPE
% load data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
test.list = importdata("test_list.xlsx");
test.name = test.list.textdata(2:end,2);
file.path = 'C:\Users\giuli\OneDrive\Desktop\chiavetta\Data_from_Jscope';

% STM UNIT CONVERSION GAIN:

inv.Ubus = 36.22;
stconv.s16v_2_v = 1/2^15;
stconv.v_2_s16v = 2^15;
stconv.absVdq_max = inv.Ubus/sqrt(3); %circle saturation%
stconv.mA_2_s16A = 2^16*0.01*5.18/3.3/1000;

% data analysis %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cur_mean_a = nan(length(test.name),1);
cur_mean_b = nan(length(test.name),1);
cur_mean_d = nan(length(test.name),1);
cur_mean_q = nan(length(test.name),1);
angle_test = nan(length(test.name),1);

for j=1:length(test.name)
    file.name = test.name(j);
    file.full_path = fullfile(file.path, file.name);
    test_data = importdata(file.name{1});
    var_name = test_data.colheaders;
    var_data = test_data.data;
    for i=1:length(var_name)
        switch var_name{1,i}
            case {'Timestamp'}
                time = var_data(:,i);
            case {'a'}
                current_a = var_data(:,i);
            case {'b'}
                current_b = var_data(:,i);
            case {'q'}
                current_q = var_data(:,i);
            case {'d'}
                current_d = var_data(:,i);
            case {'hElAngle'}
                angle_el = var_data(:,i);
            case {'hElvelocity'}
                velocity_mech = var_data(:,i);
            otherwise
                sprintf('VARIABLE NAME NOT FOUND');
        end
    end
    
    cur_mean_a(j) = mean(current_a);
    cur_mean_b(j) = mean(current_b);
    cur_mean_d(j) = mean(current_d)/stconv.mA_2_s16A/1000;  % [A]
    cur_mean_q(j) = mean(current_q);
    angle_test(j) = mean(angle_el);
end

%% PLOT RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vd_vec = [0.1 0.2 0.3 0.4 0.6 0.8 1 1.5 2 2.5 3]'*1000*stconv.s16v_2_v*stconv.absVdq_max; % [true applied V]

% figure;
% plot(Vd_vec, cur_mean_d);
% hold on;
% plot(Vd_vec, cur_mean_d,'ro');
% xlabel('Vd reference [s16V]');
% ylabel('d current [s16A]');
% grid on;
% title('Equivalent resistance');

[xData, yData] = prepareCurveData( cur_mean_d(7:10), Vd_vec(7:10) ); %%% fit input
ft = fittype( 'poly1' );
[fitresult, gof] = fit( xData, yData, ft);

figure;
plot(cur_mean_d, Vd_vec);
hold on;
plot(cur_mean_d, Vd_vec,'ro');
h = plot( fitresult);
ylabel('Vd reference [applied V]');
xlabel('d current [A]');
grid on;
title('Equivalent resistance');

id_sim = [535.9 1072 1608 2144 3215 4287 5359 8039 10720 13400 16080]'/stconv.mA_2_s16A/1000;  % [A];
plot(id_sim, Vd_vec);
plot(id_sim, Vd_vec, 'o');
legend('Inverte non-idealities','Test points','Fitted line','Ideal inverter');


Req = fitresult.p1
Vd_dist = fitresult.p2

%% modelling inverter non idealities:


Id_LUT = [cur_mean_d(1:4); cur_mean_d(6:9)];
Vd_LUT = [Vd_vec(1:4); Vd_vec(6:9)];
Vd_LUT = Vd_LUT - Req*Id_LUT;

figure;
plot(Id_LUT, Vd_LUT);
hold on;
plot(Id_LUT, Vd_LUT, 'o');
title('INVERTER VOLTAGE DISTURBANCE');
grid on;
xlabel('current [A]');
ylabel('voltage [applied V]');

[xData, yData] = prepareCurveData( Id_LUT, Vd_LUT );
ft = fittype( 'a*exp(-b*x)+c', 'independent', 'x', 'dependent', 'y' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.StartPoint = [0.502852378693869 0.410694515205481 0.936058733712811];
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot
figure;
h = plot( fitresult, xData, yData );
xlabel('current [A]');
ylabel('voltage [applied V]');
grid on;
title('INVERTER VOLTAGE DISTURBANCE');

Vd_error = VI_error(1)
Id_error = IV_error(0.1418)

function V = VI_error(I)
    V = -0.1802*exp(-1.796*I)+0.1717
end

function I = IV_error(V)
    I = log((V- 0.1717)/(-0.1802))/(-1.796)
end