function display_cameras( T, Omega, K, varargin )
% DISPLAY_CAMERAS displays camera trajectory and view frustum.
%
% Inputs:
%        T:     3-by-m (Translation)
%        Omega: 3-by-m (Rotation)
%        K:     3-by-3-by-m or 3-by-3 or 4-by-m (Calibration)
%
% Options:
%        'frustum_scale', scale    - scaling factor for camera view frustum
%        'trajectory_color', color - camera trajectory line color [R,G,B]
%        'no_frustum'              - display no camera view frustum
%        'no_trajectory'           - display no camera trajectory

num_required_parameters = 3;
if nargin < num_required_parameters
    help display_cameras.m
    return;
end

% initialize default parameters
frustum_scale    = 3;
trajectory_color = [0,0,1];
frustum          = true;
trajectory       = true;

% parse optional parameters
if nargin > num_required_parameters
    iVarargin = 1;
    while iVarargin <= nargin - num_required_parameters
        switch lower(varargin{iVarargin})
            case 'frustum_scale'
                frustum_scale = varargin{iVarargin+1};
                iVarargin = iVarargin + 1;
            case 'trajectory_color'
                trajectory_color = varargin{iVarargin+1};
                iVarargin = iVarargin + 1;
            case 'no_frustum'
                frustum = false;
            case 'no_trajectory'
                trajectory = false;
        end
        iVarargin = iVarargin + 1;
    end
end

% get information
m = size(T, 2); % m images

% get K
% make K to be 3-by-3-by-m if K is given as a 3-by-3 matrix
if size(size(K), 2) == 2
    K_ = zeros(3,3,m);
    for i = 1:m
        K_(:,:,i) = K;
    end
    K = K_;
    clear K_;
end


% compute camera center
C = zeros(3,m);
for t=1:m
%     C(:,t) = -rodrigues(Omega(:,t))' * T(:,t);
C(:,t)=T(:,t);
end

hold on;

% draw camera view frustum
if frustum
    % scaling factor of camera calibration
    sx = 100/frustum_scale; % scaling factor in x direction (in pixels/metric)
    sy = 100/frustum_scale; % scaling factor in y direction (in pixels/metric)

    % plot cameras
    overlap_point = zeros(3,m);
    overlap = zeros(1,m);
    for t=1:m
        % check if camera view is overlapping
        ox = K(1,3,t); oy = K(2,3,t);   % principal point (in pixels)
        if ox == 0
            % we assume if ox is zero, the camera is calibrated
            ox = 1; oy = 1;
        end
        f  = K(1,1,t)/sx; % focal length (in metric)
        overlap_point(:,t) = rodrigues(Omega(:,t))' * [ 0; 0; f ] + C(:,t);
        for i=1:(t-1)
            if (overlap(i) == 0) && ...
               (norm(overlap_point(:,i)-overlap_point(:,t)) < 0.8*f)
                overlap(t) = 1;
                break;
            end
        end

        if overlap(t) == 0
            % plot view frustum
            imgPlane = [ 0 -ox/sx  ox/sx ox/sx -ox/sx ; ...
                         0 -oy/sy -oy/sy oy/sy  oy/sy ; ...
                         0  f      f     f      f ];  % image plane corner points
            frustum = zeros(3,5);
            for i=1:5
                frustum(:,i) = rodrigues(Omega(:,t)) * imgPlane(:,i) + C(:,t);
            end
            frustum_struct = struct( 'Vertices', frustum', ...
                'Faces', [ 1 2 3 ; 1 3 4 ; 1 4 5 ; 1 5 2 ; 2 4 3 ; 2 5 4 ], ...
                'FaceVertexCData', [ 0.2 0.2 0.2 ; 0.4 0.4 0.4 ; 0.6 0.6 0.6 ; ...
                                     0.3 0.3 0.3 ; 0.5 0.5 0.5 ; 0.5 0.5 0.5 ], ...
                'FaceColor', 'flat', 'EdgeColor', 'blue' );
            patch(frustum_struct);      %   This gives the shit that we need
        end
    end
end

% draw a trajectory (solid line) of camera centers
if trajectory
    line( C(1,:), C(2,:), C(3,:), 'Color', trajectory_color );
end

% axis([-50 50 -50 50 -50 50]);

hold on;
% xlim('auto'), ylim('auto'), zlim('auto');
xlabel('X');
ylabel('Y');
zlabel('Z');
end
