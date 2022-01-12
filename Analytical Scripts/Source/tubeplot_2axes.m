function [tube,ax,x,y,z]=tubeplot_2axes(curve,r,n,ct,ax1,colour,xlimit,ylimit,zlimit,FontSizeMultiplier,LegendName)
% Usage: [x,y,z]=tubeplot(curve,r,n,ct)
% 
% Tubeplot constructs a tube, or warped cylinder, along
% any 3D curve, much like the build in cylinder function.
% If no output are requested, the tube is plotted.
% Otherwise, you can plot by using surf(x,y,z);
%
% Example of use:
% t=linspace(0,2*pi,50);
% tubeplot([cos(t);sin(t);0.2*(t-pi).^2],0.1);
% daspect([1,1,1]); camlight;
%
% Arguments:
% curve: [3,N] vector of curve data
% r      the radius of the tube
% n      number of points to use on circumference. Defaults to 8
% ct     threshold for collapsing points. Defaults to r/2
%
% The algorithms fails if you have bends beyond 90 degrees.
% Janus H. Wesenberg, july 2004

if nargin<3 || isempty(n), n=8;
    if nargin<2, error('Give at least curve and radius');
    end
end
if size(curve,1)~=3
    error('Malformed curve: should be [3,N]');
end
if nargin<4 || isempty(ct)
    ct=0.5.*r;
end


%Collapse points within 0.5 r of each other
npoints=1;
for k=2:(size(curve,2)-1)
    if norm(curve(:,k)-curve(:,npoints))>ct
        npoints=npoints+1;
        curve(:,npoints)=curve(:,k);
    end
end
%Always include endpoint
if norm(curve(:,end)-curve(:,npoints))>0
    npoints=npoints+1;
    curve(:,npoints)=curve(:,end);
end

%deltavecs: average for internal points.
%           first strecth for endpoitns.
dv=curve(:,[2:end,end])-curve(:,[1,1:end-1]);

%make nvec not parallel to dv(:,1)
nvec=zeros(3,1);
[buf,idx]=min(abs(dv(:,1)));
nvec(idx)=1;

xyz=repmat([0],[3,n+1,npoints+2]);

%precalculate cos and sin factors:
cfact=repmat(cos(linspace(0,2*pi,n+1)),[3,1]);
sfact=repmat(sin(linspace(0,2*pi,n+1)),[3,1]);

%Main loop: propagate the normal (nvec) along the tube
for k=1:npoints
    convec=cross(nvec,dv(:,k));
    convec=convec./norm(convec);
    nvec=cross(dv(:,k),convec);
    nvec=nvec./norm(nvec);
    %update xyz:
    xyz(:,:,k+1)=repmat(curve(:,k),[1,n+1])+...
        cfact.*repmat(r(k)*nvec,[1,n+1])...
        +sfact.*repmat(r(k)*convec,[1,n+1]);
end

%finally, cap the ends:
xyz(:,:,1)=repmat(curve(:,1),[1,n+1]);
xyz(:,:,end)=repmat(curve(:,end),[1,n+1]);

%,extract results:
x=squeeze(xyz(1,:,:));
y=squeeze(xyz(2,:,:));
z=squeeze(xyz(3,:,:));

ax=axes;

linkaxes([ax1,ax])

%   if nargout<3
tube=surf(ax,x,y,z,'FaceAlpha',0.5,'DisplayName',LegendName);
grid(ax,'off');
shading(ax,'interp');
colormap(ax,colour); %... and plot:
%   end


set(get(ax,'XLabel'),'String','Hip angle (rad)','Interpreter','latex')
set(get(ax,'YLabel'),'String','Knee angle (rad)','Interpreter','latex')
set(get(ax,'ZLabel'),'String','Ankle angle (rad)','Interpreter','latex')
set(get(ax,'Title'),'String','Left leg')
set(ax,'FontSize',10*FontSizeMultiplier,'TickLabelInterpreter','latex')

% linkprop([ax,ax1],{'XLim','YLim','ZLim','Color'})

if ~isempty(xlimit) 
    ax.XLim=xlimit;
    ax.YLim=ylimit;
    ax.ZLim=zlimit;
end
%   axis equal
ax.Visible='off';
% ax.XTick=[];
% ax.YTick=[];
% ax.ZTick=[];
end


