function plotRefTraj(Q_REF,LineWidthMultiplier,FontSizeMultiplier,right_or_left,black_or_coloured)
if ~exist('right_or_left','var')
    right_or_left='left';
end

if ~exist('black_or_coloured','var')
    black_or_coloured='coloured';
end

if isequal(black_or_coloured,'coloured')
    c1='g'; c2='k'; c3='k--';c4='k';c5='y'; c6='b'; c7='b--';c8='b'; 
elseif isequal(black_or_coloured,'black')
    c1='k';c2='k';c3='k';c4='k';c5='k';c6='k';c7='k';c8='k';
end

if isequal(right_or_left,'left')
    plot3(Q_REF.DS_RF(:,2),Q_REF.DS_RF(:,4),Q_REF.DS_RF(:,6),c1,'LineWidth',1*LineWidthMultiplier,'DisplayName','Double Stance - Right Forward');
    hold on
    plot3(Q_REF.ISW_L(:,2),Q_REF.ISW_L(:,4),Q_REF.ISW_L(:,6),c2,'LineWidth',1*LineWidthMultiplier,'DisplayName','Initial Swing (Left)');
    plot3(Q_REF.MSW_L(:,2),Q_REF.MSW_L(:,4),Q_REF.MSW_L(:,6),c3,'LineWidth',1*LineWidthMultiplier,'DisplayName', 'Mid-Swing');
    plot3(Q_REF.FSW_L(:,2),Q_REF.FSW_L(:,4),Q_REF.FSW_L(:,6),c4,'LineWidth',1*LineWidthMultiplier,'DisplayName', 'Terminal Swing');
    plot3(Q_REF.DS_LF(:,2),Q_REF.DS_LF(:,4),Q_REF.DS_LF(:,6),c5,'LineWidth',1*LineWidthMultiplier,'DisplayName', 'Double Stance - Left Forward');
    plot3(Q_REF.ISW_R(:,2),Q_REF.ISW_R(:,4),Q_REF.ISW_R(:,6),c6,'LineWidth',1*LineWidthMultiplier,'DisplayName','Initial Stance (Left)');
    plot3(Q_REF.MSW_R(:,2),Q_REF.MSW_R(:,4),Q_REF.MSW_R(:,6),c7,'LineWidth',1*LineWidthMultiplier,'DisplayName','Mid-Stance');
    plot3(Q_REF.FSW_R(:,2),Q_REF.FSW_R(:,4),Q_REF.FSW_R(:,6),c8,'LineWidth',1*LineWidthMultiplier,'DisplayName','Terminal Stance');
    set(gca,'FontName','cmr12','FontSize',10*FontSizeMultiplier,'TickLabelInterpreter','latex')
    xlabel('Hip angle (rad)','FontSize',11*FontSizeMultiplier,'interpreter','latex')
    ylabel('Knee angle (rad)','FontSize',11*FontSizeMultiplier,'interpreter','latex')
    zlabel('Ankle angle (rad)','FontSize',11*FontSizeMultiplier,'interpreter','latex')
    plot3(Q_REF.DS_RF(1,2),Q_REF.DS_RF(1,4),Q_REF.DS_RF(1,6),'o','DisplayName','Hill strike - Right leg (HS$_R$)','MarkerSize',14);
    plot3(Q_REF.DS_LF(1,2),Q_REF.DS_LF(1,4),Q_REF.DS_LF(1,6),'o','DisplayName','Hill strike - Left leg (HS$_L$)','MarkerSize',14);
    plot3(Q_REF.ISW_R(1,2),Q_REF.ISW_R(1,4),Q_REF.ISW_R(1,6),'o','DisplayName','Toe off - Right leg (TO$_R$)','MarkerSize',14);
    plot3(Q_REF.ISW_L(1,2),Q_REF.ISW_L(1,4),Q_REF.ISW_L(1,6),'o','DisplayName','Toe off - Left leg (TO$_L$)','MarkerSize',14);
    text(Q_REF.DS_RF(1,2),Q_REF.DS_RF(1,4),Q_REF.DS_RF(1,6),'\leftarrow HS_R','FontSize',15)
    text(Q_REF.DS_LF(1,2),Q_REF.DS_LF(1,4),Q_REF.DS_LF(1,6),'\leftarrow HS_L','FontSize',15);
    text(Q_REF.ISW_R(1,2),Q_REF.ISW_R(1,4),Q_REF.ISW_R(1,6),'TO_R \rightarrow', 'HorizontalAlignment','right','FontSize',15);
    text(Q_REF.ISW_L(1,2),Q_REF.ISW_L(1,4),Q_REF.ISW_L(1,6),'TO_L \rightarrow','HorizontalAlignment','right','FontSize',15);
elseif isequal(right_or_left,'right')
    plot3(Q_REF.DS_RF(:,1),Q_REF.DS_RF(:,3),Q_REF.DS_RF(:,5),c1,'LineWidth',1*LineWidthMultiplier,'DisplayName','Double Stance - Right Forward');
    hold on
    plot3(Q_REF.ISW_L(:,1),Q_REF.ISW_L(:,3),Q_REF.ISW_L(:,5),c6,'LineWidth',1*LineWidthMultiplier,'DisplayName','Initial Stance (Right)');
    plot3(Q_REF.MSW_L(:,1),Q_REF.MSW_L(:,3),Q_REF.MSW_L(:,5),c7,'LineWidth',1*LineWidthMultiplier,'DisplayName', 'Mid-Stance');
    plot3(Q_REF.FSW_L(:,1),Q_REF.FSW_L(:,3),Q_REF.FSW_L(:,5),c8,'LineWidth',1*LineWidthMultiplier,'DisplayName', 'Terminal Stance');
    plot3(Q_REF.DS_LF(:,1),Q_REF.DS_LF(:,3),Q_REF.DS_LF(:,5),c5,'LineWidth',1*LineWidthMultiplier,'DisplayName', 'Double Stance - Left Forward');
    plot3(Q_REF.ISW_R(:,1),Q_REF.ISW_R(:,3),Q_REF.ISW_R(:,5),c2,'LineWidth',1*LineWidthMultiplier,'DisplayName','Initial Swing (Right)');
    plot3(Q_REF.MSW_R(:,1),Q_REF.MSW_R(:,3),Q_REF.MSW_R(:,5),c3,'LineWidth',1*LineWidthMultiplier,'DisplayName','Mid-Swing');
    plot3(Q_REF.FSW_R(:,1),Q_REF.FSW_R(:,3),Q_REF.FSW_R(:,5),c4,'LineWidth',1*LineWidthMultiplier,'DisplayName','Terminal Swing');
    set(gca,'FontName','cmr12','FontSize',10*FontSizeMultiplier,'TickLabelInterpreter','latex')
    xlabel('Hip angle (rad)','FontSize',11*FontSizeMultiplier,'interpreter','latex')
    ylabel('Knee angle (rad)','FontSize',11*FontSizeMultiplier,'interpreter','latex')
    zlabel('Ankle angle (rad)','FontSize',11*FontSizeMultiplier,'interpreter','latex')
    plot3(Q_REF.DS_RF(1,1),Q_REF.DS_RF(1,3),Q_REF.DS_RF(1,5),'o','DisplayName','Hill strike - Right leg (HS$_R$)','MarkerSize',14);
    plot3(Q_REF.DS_LF(1,1),Q_REF.DS_LF(1,3),Q_REF.DS_LF(1,5),'o','DisplayName','Hill strike - Left leg (HS$_L$)','MarkerSize',14);
    plot3(Q_REF.ISW_R(1,1),Q_REF.ISW_R(1,3),Q_REF.ISW_R(1,5),'o','DisplayName','Toe off - Right leg (TO$_R$)','MarkerSize',14);
    plot3(Q_REF.ISW_L(1,1),Q_REF.ISW_L(1,3),Q_REF.ISW_L(1,5),'o','DisplayName','Toe off - Left leg (TO$_L$)','MarkerSize',14);
    text(Q_REF.DS_RF(1,1),Q_REF.DS_RF(1,3),Q_REF.DS_RF(1,5),'\leftarrow HS_R','FontSize',15)
    text(Q_REF.DS_LF(1,1),Q_REF.DS_LF(1,3),Q_REF.DS_LF(1,5),'\leftarrow HS_L','FontSize',15);
    text(Q_REF.ISW_R(1,1),Q_REF.ISW_R(1,3),Q_REF.ISW_R(1,5),'TO_R \rightarrow', 'HorizontalAlignment','right','FontSize',15);
    text(Q_REF.ISW_L(1,1),Q_REF.ISW_L(1,3),Q_REF.ISW_L(1,5),'TO_L \rightarrow','HorizontalAlignment','right','FontSize',15);

end

%     annotation('textarrow',[0.48,0.5],[0.707,0.707],'String','TO$_R$','FontSize',20,'interpreter','latex');
%     annotation('textarrow',[0.54,0.52],[0.708,0.708],'String','HS$_L$','FontSize',20,'interpreter','latex');
%     annotation('textarrow',[0.2,0.22],[0.723,0.723],'String','TO$_L$','FontSize',20,'interpreter','latex');
%     annotation('textarrow',[0.255,0.235],[0.728,0.728],'String','HS$_R$','FontSize',20,'interpreter','latex');


% plot3(Q_ref_MSW_L(1,2),Q_ref_MSW_L(1,4),Q_ref_MSW_L(1,6),'*');
    % text(Q_ref_MSW_L(1,2),Q_ref_MSW_L(1,4),Q_ref_MSW_L(1,6),' ISW \rightarrow MSW');
    % plot3(Q_ref_FSW_L(1,2),Q_ref_FSW_L(1,4),Q_ref_FSW_L(1,6),'*');
    % text(Q_ref_FSW_L(1,2),Q_ref_FSW_L(1,4),Q_ref_FSW_L(1,6),' MSW \rightarrow TSW');

    legend('FontSize',17,'Location','northeast','interpreter','latex')

end