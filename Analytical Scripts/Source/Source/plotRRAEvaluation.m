%% Evaluate RRA residuals
function plotRRAEvaluation(residuals_path)
%residuals_path='C:\Users\andre\Desktop\PhD\Code\DanielsGit\fes-exo\Local Data\FES\Dynamic FES Data - 17-12-20\S1\RRA_0_100\gait2354_S1_dynamic_exp_Actuation_force.sto';
residuals_data=Data(residuals_path);

RRA_evaluation=figure();
RRA_evaluation.WindowState = 'maximized';

subplot(2,2,1);
plot(residuals_data.getColumn('time'),[residuals_data.getColumn('FX') residuals_data.getColumn('FY') residuals_data.getColumn('FZ')],'DisplayName',['FX' 'FY' 'FZ'])
x=get(gca,'XLim');
y=get(gca,'YLim');
patch('XData',[x fliplr(x)],'YData',[-25 -25 -75 -75],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'DisplayName','Bad')
patch('XData',[x fliplr(x)],'YData',[25 25 75 75],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[10 10 25 25],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'DisplayName','OK')
patch('XData',[x fliplr(x)],'YData',[-10 -10 -25 -25],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[10 10 -10 -10],'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.3,'DisplayName','Good')
ylabel('Residual Forces'); ylim([-50 50])
xlabel('Time (s)')
legend('MX','MY','MZ','Bad','OK','Good')

subplot(2,2,2);
plot(residuals_data.getColumn('time'),[residuals_data.getColumn('MX') residuals_data.getColumn('MY') residuals_data.getColumn('MZ')])
x=get(gca,'XLim');
y=get(gca,'YLim');
patch('XData',[x fliplr(x)],'YData',[-75 -75 -100 -100],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[75 75 100 100],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3)
patch('XData',[x fliplr(x)],'YData',[50 50 75 75],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[-50 -50 -75 -75],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3)
patch('XData',[x fliplr(x)],'YData',[50 50 -50 -50],'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.3)
ylabel('Residual Moments');ylim([-100 100])
xlabel('Time (s)')
legend('MX','MY','MZ','Bad','OK','Good')

% RMS

subplot(2,2,3);
bar([rms(residuals_data.getColumn('FX')), rms(residuals_data.getColumn('FY')),rms(residuals_data.getColumn('FZ'))],'HandleVisibility','off')
ylabel('RMS Residual Forces');
fname = {'FX','FY','FZ'}; 
set(gca, 'XTick', 1:length(fname),'XTickLabel',fname);
ylim([-15 15]);
x=get(gca,'XLim');
y=get(gca,'YLim');
patch('XData',[x fliplr(x)],'YData',[-10 -10 -15 -15],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[10 10 15 15],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3)
patch('XData',[x fliplr(x)],'YData',[5.0 5.0 10 10],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[-5 -5 -10 -10],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3)
patch('XData',[x fliplr(x)],'YData',[5 5 -5 -5],'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.3)
hold on
bar([rms(residuals_data.getColumn('FX')), rms(residuals_data.getColumn('FY')),rms(residuals_data.getColumn('FZ'))],'FaceColor',[0.00,0.45,0.74],'HandleVisibility','off')
legend('Bad','OK','Good')


subplot(2,2,4);
bar([rms(residuals_data.getColumn('MX')), rms(residuals_data.getColumn('MY')),rms(residuals_data.getColumn('MZ'))],'HandleVisibility','off')
ylabel('RMS Residual Moments');
fname = {'FX','FY','FZ'}; 
set(gca, 'XTick', 1:length(fname),'XTickLabel',fname);
ylim([-100 100])
x=get(gca,'XLim');
y=get(gca,'YLim');
patch('XData',[x fliplr(x)],'YData',[-50 -50 -100 -100],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[50 50 100 100],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3)
patch('XData',[x fliplr(x)],'YData',[30 30 50 50],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[-30 -30 -50 -50],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3)
patch('XData',[x fliplr(x)],'YData',[30 30 -30 -30],'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.3)
hold on
bar([rms(residuals_data.getColumn('MX')), rms(residuals_data.getColumn('MY')),rms(residuals_data.getColumn('MZ'))],'FaceColor',[0.00,0.45,0.74],'HandleVisibility','off')
legend('Bad','OK','Good')

%% plot pErr
% residuals_path(end-18:end)
pErr_path=strrep(residuals_path,'Actuation_force','pErr');
pErr_data=Data(pErr_path);
pErr_fig=figure();
pErr_fig.WindowState = 'maximized';

good_thresh=0.02;%2cm
ok_thresh=0.05;
bad_thresh=0.1;%random value for visualisation, anything above ok_thresh is bad

subplot(2,2,1);
plot(pErr_data.getColumn('time'),[pErr_data.getColumn('pelvis_tx'), pErr_data.getColumn('pelvis_ty'),pErr_data.getColumn('pelvis_tz')])
x=get(gca,'XLim');
y=get(gca,'YLim');
patch('XData',[x fliplr(x)],'YData',[-ok_thresh -ok_thresh -bad_thresh -bad_thresh],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'DisplayName','Bad')
patch('XData',[x fliplr(x)],'YData',[ok_thresh ok_thresh bad_thresh bad_thresh],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[good_thresh good_thresh ok_thresh ok_thresh],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'DisplayName','OK')
patch('XData',[x fliplr(x)],'YData',[-good_thresh -good_thresh -ok_thresh -ok_thresh],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[good_thresh good_thresh -good_thresh -good_thresh],'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.3,'DisplayName','Good')
ylabel('Translational Marker Error (m)'); 
ylim([-bad_thresh bad_thresh])
xlabel('Time (s)')
legend('pelvis\_tx','pelvis\_ty','pelvis\_tz','Bad','OK','Good')

subplot(2,2,2);
for i=5:length(pErr_data.Labels)
    plot(pErr_data.getColumn('time'),rad2deg(pErr_data.getColumn(i)),'HandleVisibility','off')
    hold on
end

good_thresh=2;%2cm
ok_thresh=5;
bad_thresh=20;%random value for visualisation, anything above ok_thresh is bad

x=get(gca,'XLim');
y=get(gca,'YLim');
patch('XData',[x fliplr(x)],'YData',[-ok_thresh -ok_thresh -bad_thresh -bad_thresh],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'DisplayName','Bad')
patch('XData',[x fliplr(x)],'YData',[ok_thresh ok_thresh bad_thresh bad_thresh],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[good_thresh good_thresh ok_thresh ok_thresh],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'DisplayName','OK')
patch('XData',[x fliplr(x)],'YData',[-good_thresh -good_thresh -ok_thresh -ok_thresh],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[good_thresh good_thresh -good_thresh -good_thresh],'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.3,'DisplayName','Good')
ylabel('Rotational Marker Error (deg)'); 
ylim([-bad_thresh bad_thresh])
xlabel('Time (s)')
legend('Bad','OK','Good')

% RMS

subplot(2,2,3);
bar([rms(pErr_data.getColumn('pelvis_tx')), rms(pErr_data.getColumn('pelvis_ty')),rms(pErr_data.getColumn('pelvis_tz'))],'HandleVisibility','off')
ylabel('RMS Trans Marker Error (m)');
good_thresh=0.02;%2cm
ok_thresh=0.04;
bad_thresh=0.1;%random value for visualisation, anything above ok_thresh is bad
ylim([-bad_thresh bad_thresh]);
x=get(gca,'XLim');
y=get(gca,'YLim');
patch('XData',[x fliplr(x)],'YData',[-ok_thresh -ok_thresh -bad_thresh -bad_thresh],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'DisplayName','Bad')
patch('XData',[x fliplr(x)],'YData',[ok_thresh ok_thresh bad_thresh bad_thresh],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[good_thresh good_thresh ok_thresh ok_thresh],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'DisplayName','OK')
patch('XData',[x fliplr(x)],'YData',[-good_thresh -good_thresh -ok_thresh -ok_thresh],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[good_thresh good_thresh -good_thresh -good_thresh],'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.3,'DisplayName','Good')
hold on
bar([rms(pErr_data.getColumn('pelvis_tx')), rms(pErr_data.getColumn('pelvis_ty')),rms(pErr_data.getColumn('pelvis_tz'))],'FaceColor',[0.00,0.45,0.74],'HandleVisibility','off')


subplot(2,2,4);
for i=5:length(pErr_data.Labels)
    bar(i-4,rms(rad2deg(pErr_data.getColumn(i))),'HandleVisibility','off')
    hold on
end
ylabel('RMS Rot Marker Error (deg)');
good_thresh=2;%2cm
ok_thresh=5;
bad_thresh=20;%random value for visualisation, anything above ok_thresh is bad
ylim([-bad_thresh bad_thresh])
x=get(gca,'XLim');
y=get(gca,'YLim');
patch('XData',[x fliplr(x)],'YData',[-ok_thresh -ok_thresh -bad_thresh -bad_thresh],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'DisplayName','Bad')
patch('XData',[x fliplr(x)],'YData',[ok_thresh ok_thresh bad_thresh bad_thresh],'FaceColor',[0.1 0.1 0.1],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[good_thresh good_thresh ok_thresh ok_thresh],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'DisplayName','OK')
patch('XData',[x fliplr(x)],'YData',[-good_thresh -good_thresh -ok_thresh -ok_thresh],'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.3,'HandleVisibility','off')
patch('XData',[x fliplr(x)],'YData',[good_thresh good_thresh -good_thresh -good_thresh],'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.3,'DisplayName','Good')
legend('Bad','OK','Good')
end