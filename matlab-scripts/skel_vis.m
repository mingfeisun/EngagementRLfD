function skel_vis(X,tidx,PFilter,h,mode)
%SKEL_VIS -- Visualize a skeleton in 3D coordinates.
%
% Input
%   X: (T,4*NUI_SKELETON_POSITION_COUNT) matrix from load_file.
%   tidx: time index >=1, <=T.
%   h: (optional) axes handle to draw in.
%
% Author: Sebastian Nowozin <Sebastian.Nowozin@microsoft.com>

assert(tidx >= 1);
assert(tidx <= size(X,1));
xyz_ti=X(tidx,:);

skel_model
skel=reshape(xyz_ti, 4, NUI_SKELETON_POSITION_COUNT)';
if nargin < 3
	h=axes;
end

if mode == 'true'
    plot3(skel(:,1), skel(:,2), skel(:,3), 'bo');
end

if mode == 'pred'
    plot3(skel(:,1), skel(:,2), skel(:,3), 'bo');
    for i=1:length(PFilter)
        temp = PFilter(i).Particles;
        scatter3(temp(:, 1), temp(:, 2), temp(:, 3), 'r.');
    end
end


axis equal;

for ci=1:size(nui_skeleton_conn,1)
	hold on;
	line([xyz_ti(4*nui_skeleton_conn(ci,1)+1) ; ...
		xyz_ti(4*nui_skeleton_conn(ci,2)+1)], ...
		[xyz_ti(4*nui_skeleton_conn(ci,1)+2) ; ...
		xyz_ti(4*nui_skeleton_conn(ci,2)+2)], ...
		[xyz_ti(4*nui_skeleton_conn(ci,1)+3) ; ...
		xyz_ti(4*nui_skeleton_conn(ci,2)+3)]);
end
cpos= [-0.6, -0.4, 1];

set(h,'CameraPosition',cpos);
set(h,'CameraTarget',skel(HIP_CENTER+1,1:3));
set(h,'CameraViewAngle',50);
set(h,'CameraUpVector',[0 0 1]);
set(h,'Projection','perspective');

xmin=min(min(X(:,1:4:size(X,2))));
ymin=min(min(X(:,2:4:size(X,2))));
zmin=min(min(X(:,3:4:size(X,2))));
xmax=max(max(X(:,1:4:size(X,2))));
ymax=max(max(X(:,2:4:size(X,2))));
zmax=max(max(X(:,3:4:size(X,2))));

set(h,'XLim', [xmin, xmax]);
set(h,'YLim', [ymin, ymax]);
set(h,'ZLim', [zmin, zmax]);

tpos=[0.5*(xmin+xmax), 0.5*(ymin+ymax), 0.5*(zmin+zmax)];

set(h,'CameraPosition',cpos);
set(h,'CameraTarget',tpos);

title(sprintf('Frame %d', tidx));

