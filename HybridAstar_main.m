clc
clear
close all


% ================== Map / Scenario ==================
% IMPORTANT:
%  - Define row/col first
%  - Then initialize sign = zeros(row,col)
%  - Then assign obstacles into sign
% This prevents "index exceeds array bounds" when you change map size.

row = 10;
col = 30;

sign = zeros(row, col);

% 小障碍物：3行×3列


% 大障碍物：这里是 3×6两个
sign(4:9, 23:24) = 1;
sign(1:7, 17:19) = 1;

startPose = [1.5 2.0 0];    % 左侧，朝向 +X
goalPose  = [28.0 8.0 0];   % 右侧，朝向 +X

% 防呆检查：确保 sign 尺寸与 row/col 一致
assert(all(size(sign) == [row, col]), 'sign size mismatch: sign must be row-by-col');


% ================== 4WIS simplified kinematic model parameters ==================
% State: X = [x, y, phi]^T
% Control: u = [v, delta_f, delta_r]^T
% Kinematics: phi_dot = (v/L) * (tan(delta_f) - tan(delta_r))
%
% We keep the original planner's "min_r" interface by converting 4WIS steering
% limits into an equivalent minimum turning radius using the counter-phase mode:
%   delta_f = +delta_max, delta_r = -delta_max  (small-radius turning)
% Then:
%   kappa_max = (tan(delta_f) - tan(delta_r))/L = 2*tan(delta_max)/L
%   R_min     = 1/kappa_max = L/(2*tan(delta_max))
L = 2.7;                 % wheelbase [m]
delta_max = deg2rad(30); % max steering angle [rad]
min_r = L/(2*tan(delta_max)); % equivalent minimum turning radius [m]

% ================== 4WS independent steering: extra parameters (primitives + visualization) ==================
W = 1.6;                     % track width [m]
beta = deg2rad(20);          % crab steering angle [rad]
dpsi = deg2rad(10);          % spin in-place heading step [rad]

% "Ackermann-equivalent" larger turning radius (proxy for same-phase wide turn)
min_r_ack = L / tan(delta_max);

% action set:
% 1..3: counter-phase tight left/straight/tight right (existing)
% 7..8: same-phase wide left/right
% 9..10: crab left/right
% 11..12: spin left/right
actions = [1 2 3 7 8 9 10 11 12];

% ================== speed-up: gate crab/spin actions near goal ==================
R_gate = 8; % [m] enable crab/spin only within this distance to goal
actions_far = [1 2 3 7 8];
actions_near = actions;

% ================== steering actuator rate limit ==================
deltaRateMax_deg_s = 60;              % [deg/s]
deltaRateMax = deg2rad(deltaRateMax_deg_s); % [rad/s]
v_min_for_dt = 0.2;                   % [m/s] prevent dt blow-up when v->0

% ================== curvature transition (clothoid-like, post-process) ==================
% Insert a short transition segment when curvature jump is too large, to improve
% curvature continuity at primitive boundaries.
Lt = 0.6;                 % [m] transition length (change this to 0.8 / 1.0 if needed)

% Make transitions appear "a little" (trigger only on moderately large jumps).
% The 3-pt curvature estimate tends to be small; 0.25 may be too strict.
kappaJumpThresh = 0.25;   % [1/m] curvature jump threshold
M = 21;                   % number of points in transition segment

% visualize transition points on path
transitionModeName = "transition";
transitionColor = [0.95 0.35 0.95]; % magenta-ish
transitionLineWidth = 3.2;

% ================== planner / collision parameters ==================
% Reduce inflated obstacle margin to improve feasibility in narrow passages.
safe_dis=1.0;%

% Keep a safety margin from the map boundary (grid units).
% A value of 0.5 means the vehicle's reference point must stay at least
% half a cell away from the outer border.
boundary_margin = 1.0;


% Step length along arc/line segment for each node expansion.
% Recommended to scale with turning radius to avoid overly coarse arcs.
% Here we choose ~10 degrees of heading change per turning step at kappa_max:
%   step = R_min * (10deg)
step = 1.2;

P3=0.01;%

ob_coo=[];

% ================== Hybrid A*: visited pruning via (x,y,theta) discretization ==================
% This dramatically reduces node explosion in narrow corridors / S-turn scenarios.
% Use a finer heading discretization to reduce over-pruning in tight maneuvers.
thetaBinSize = deg2rad(10);           % 10 deg per bin

thetaBins = round(2*pi/thetaBinSize); % 36 bins

visited = false(row, col, thetaBins); % visited(iy, ix, ith)

% NOTE: Clamp bin index to [1, thetaBins] to avoid rare floating-point / NaN issues.
binWidth = 2*pi/thetaBins;
theta_to_bin = @(th) min(thetaBins, max(1, floor(mod(th, 2*pi) / binWidth) + 1));
% ================== Visualization / performance ==================
doPlotSearch = false;   % 规划阶段不画扩展过程（最快）
plotEveryN  = 50;       % doPlotSearch=true 时，每 N 个节点画一次

% ================== Mode colors (default) ==================
% Used for: path segmentation coloring + wheel-angle plot background shading.
modeAlpha = 0.12;
modeColors = containers.Map;
modeColors("straight") = [0.35 0.35 0.35];
modeColors("counter_phase") = [0.85 0.20 0.20];
modeColors("same_phase") = [0.20 0.35 0.90];
modeColors("crab") = [0.20 0.70 0.20];
modeColors("spin") = [0.60 0.25 0.75];
modeColors("rs_counter_phase") = [1.00 0.55 0.15];
modeColors("start") = [0.00 0.00 0.00];
modeColors(transitionModeName) = transitionColor;

% ========== 

figure(1)%
hold on%
axis equal%

% map grid is 1m x 1m
xlabel('x (m)');
ylabel('y (m)');
set(gca,'XTick',0:1:col);
set(gca,'YTick',0:1:row);

grid on;
set(gca,'GridColor',[0 0 0]);
set(gca,'GridAlpha',0.15);

for i=1:row
    for j=1:col
        if sign(i,j)==1%% 
            y=[i-1,i-1,i,i];%
            x=[j-1,j,j,j-1];% 
            h=fill(x,y,'k');
            set(h,'facealpha',1)
            ob_coo=[ob_coo;[j-0.5,i-0.5]];% 
        end
    end
end
axis([0 col 0 row])%
for i=1:row%
    plot([0 col],[i i],'k-');%
end
for i=1:col %
    plot([i i],[0 row],'k-');%
end
plot(startPose(1),startPose(2), 'p','markersize', 10,'markerfacecolor','b','MarkerEdgeColor', 'm')
plot(goalPose(1),goalPose(2),'o','markersize', 10,'markerfacecolor','g','MarkerEdgeColor', 'c')


% ================== Hybrid A* search ==================
opened=[startPose 0 0 0 0 1 0];%x,y,sita,

dis_astar=Astar_fun(fliplr(ceil(startPose(1:2))),fliplr(ceil(goalPose(1:2))),sign);
[dis_rs,~]=reeds_shepp_fun(startPose,goalPose,min_r); % Reeds-Shepp
opened(1,6)=max(dis_astar,dis_rs); % h
opened(1,7)=opened(1,5)+opened(1,6); % f

now_point=opened; % 
nodeIndex = 1;%

nodeArray(nodeIndex).x=now_point(1);
nodeArray(nodeIndex).y=now_point(2);
nodeArray(nodeIndex).sita=now_point(3);
nodeArray(nodeIndex).D=now_point(4);
nodeArray(nodeIndex).g=now_point(5);
nodeArray(nodeIndex).h=now_point(6);
nodeArray(nodeIndex).f=now_point(7);
nodeArray(nodeIndex).ind=now_point(8);
nodeArray(nodeIndex).parent=now_point(9);
nodeArray(nodeIndex).route=[];% 
nodeArray(nodeIndex).mode="start";
nodeArray(nodeIndex).wheelAngles=[0 0 0 0]; % [delta_fl delta_fr delta_rl delta_rr]
nodeIndex = nodeIndex + 1;% 

% mark start node as visited
ix = round(now_point(1)); iy = round(now_point(2));
ith = theta_to_bin(now_point(3));
if ix>=1 && ix<=col && iy>=1 && iy<=row
    visited(iy, ix, ith) = true;
end

% ================== planning time ==================
tPlanStart = tic;

[min_num,index] = min(opened(:,7));
while ceil(now_point(1))~=ceil(goalPose(1))||ceil(now_point(2))~=ceil(goalPose(2))   %
    opened(index,:) = []; % 

    dist2goal = norm(now_point(1:2) - goalPose(1:2));
    if dist2goal > R_gate
        actionsLocal = actions_far;
    else
        actionsLocal = actions_near;
    end

    for a = actionsLocal
        [isok,x,y,sita,route,meta] = find_route_fun(now_point,a,step,min_r,min_r_ack,L,W,beta,dpsi,safe_dis,ob_coo);

        % Keep a margin from the boundary
        if x<=boundary_margin || x>=(col-boundary_margin) || y<=boundary_margin || y>=(row-boundary_margin)
            isok=1;
        end

        % Guard against invalid headings from route generation
        if isok==0 && ~isfinite(sita)
            isok = 1;
        end

        % visited pruning: avoid expanding near-duplicate states
        if isok==0
            nix = round(x); niy = round(y);
            nith = theta_to_bin(sita);
            if nix>=1 && nix<=col && niy>=1 && niy<=row
                if visited(niy, nix, nith)
                    isok = 1;
                end
            end
        end

        if isok==0
            temp=[x,y,sita,0,0,0,0,nodeIndex,now_point(8)];

            temp(5)=now_point(5)+step+sum(abs(temp(3)-now_point(3)))*P3;

            dis_astar=Astar_fun(fliplr(ceil(temp(1:2))),fliplr(ceil(goalPose(1:2))),sign);
            [dis_rs,~]=reeds_shepp_fun(temp(1:3),goalPose,min_r);
            temp(6)=max(dis_astar,dis_rs);
            temp(7)=temp(5)+temp(6);

            opened=[opened;temp];

            nodeArray(nodeIndex).x=temp(1);
            nodeArray(nodeIndex).y=temp(2);
            nodeArray(nodeIndex).sita=temp(3);
            nodeArray(nodeIndex).D=temp(4);
            nodeArray(nodeIndex).g=temp(5);
            nodeArray(nodeIndex).h=temp(6);
            nodeArray(nodeIndex).f=temp(7);
            nodeArray(nodeIndex).ind=temp(8);
            nodeArray(nodeIndex).parent=temp(9);
            nodeArray(nodeIndex).route=route;
            nodeArray(nodeIndex).mode=meta.mode;
            nodeArray(nodeIndex).wheelAngles=meta.wheelAngles;
            nodeIndex = nodeIndex + 1;

            if doPlotSearch && mod(nodeIndex, plotEveryN) == 0
                plot(route(:,1),route(:,2),'b-')
                plot(route(end,1),route(end,2), 'o','markersize', 2,'markerfacecolor','r','MarkerEdgeColor', 'm')
                drawnow limitrate
            end
        end
    end

    if isempty(opened)
        error('Open list is empty. No feasible path found with current constraints.');
    end

    [min_num,index] = min(opened(:,7));
    now_point=opened(index,:);

    % mark current node as visited/closed
    ix = round(now_point(1)); iy = round(now_point(2));
    ith = theta_to_bin(now_point(3));
    if ix>=1 && ix<=col && iy>=1 && iy<=row
        visited(iy, ix, ith) = true;
    end

    if norm(now_point(1:2)-goalPose(1:2))<2*step
        break
    end
end

planTime = toc(tPlanStart);
fprintf('Planning time: %.3f s\n', planTime);


% ================== Reconstruct route (+ wheel angles per sample) ==================
node_temp=nodeArray(now_point(8));
index=now_point(8);
while node_temp.parent~=0
    node_temp=nodeArray(node_temp.parent);
    index=[index node_temp.ind];
end
index=fliplr(index);

route_all=startPose;
wheel_all = zeros(1,4); % match route_all length (start)
mode_all = strings(1,1);
mode_all(1) = "start";

for i=1:length(index)
    route=nodeArray(index(i)).route;
    wa = nodeArray(index(i)).wheelAngles; % 1x4, constant for this primitive
    m  = nodeArray(index(i)).mode;

    route_all=[route_all;route(2:end,:)];
    wheel_all=[wheel_all; repmat(wa, size(route,1)-1, 1)];
    mode_all=[mode_all; repmat(string(m), size(route,1)-1, 1)];
end

% append RS curve
[~,route_rs]=reeds_shepp_fun(now_point(1:3),goalPose,min_r);
route_all=[route_all;route_rs(2:end,:)];

% --- RS wheel angles: infer from curvature, counter-phase assumption ---
wa_rs = infer_wheel_angles_from_route(route_rs, L, W, delta_max, true);
wheel_all=[wheel_all; wa_rs(2:end,:)];
mode_all=[mode_all; repmat("rs_counter_phase", size(route_rs,1)-1, 1)];

% ================== Curvature transition insertion (post-process) ==================
% Insert clothoid-like transition segments at curvature discontinuities.
% This step modifies route_all / wheel_all / mode_all by adding intermediate samples.
[route_all, wheel_all, mode_all] = insert_curvature_transitions(route_all, wheel_all, mode_all, Lt, kappaJumpThresh, M, transitionModeName);

% ================== Build arc-length s and plot wheel angles ==================
xy = route_all(:,1:2);
N = size(xy,1);

ds = sqrt(sum(diff(xy,1,1).^2,2));
s_all = [0; cumsum(ds)];

% ---------------- Speed planning first (needed for dt in steering rate limit) ----------------
% 1 grid = 1 m
% straight: ~15 km/h constant
% obstacle/curve: if kappa > 0.1 1/m then v <= 10 km/h
% U-turn segment (turn > 150 deg): v <= 5 km/h for the whole segment

kmh2ms = @(v_kmh) v_kmh / 3.6;
v15 = kmh2ms(15);
v10 = kmh2ms(10);
v5  = kmh2ms(5);

% --- curvature via 3-point formula ---
kappa = zeros(N,1);
for ii = 2:N-1
    p0 = xy(ii-1,:); p1 = xy(ii,:); p2 = xy(ii+1,:);
    a = norm(p1-p0);
    b = norm(p2-p1);
    c = norm(p2-p0);
    if a < 1e-6 || b < 1e-6 || c < 1e-6
        kappa(ii) = 0;
        continue;
    end
    area2 = abs( (p1(1)-p0(1))*(p2(2)-p0(2)) - (p1(2)-p0(2))*(p2(1)-p0(1)) ); % 2*Area
    kappa(ii) = area2 / (a*b*c); % = 2*Area/(a*b*c)
end
if N >= 2
    kappa(1) = kappa(2);
    kappa(end) = kappa(end-1);
end

% --- heading for U-turn detection ---
psi = zeros(N,1);
dxy = diff(xy,1,1);
if ~isempty(dxy)
    psi(1:end-1) = atan2(dxy(:,2), dxy(:,1));
    psi(end) = psi(end-1);
end

% --- base speed limits from rules ---
v_max = v15 * ones(N,1);

% curve/avoidance slow-down
v_max(kappa > 0.1) = min(v_max(kappa > 0.1), v10);

% U-turn detection (whole segment): cumulative heading change over window
turnThresh = deg2rad(150);
windowLen = 20; % points ~= meters (you said 1 point ~ 1m)

psi_u = unwrap(psi);
dpsi_abs = abs(diff(psi_u));

isUTurn = false(N,1);
for ii = 1:max(1, N-windowLen)
    i2 = min(N-1, ii+windowLen-1);
    cumTurn = sum(dpsi_abs(ii:i2));
    if cumTurn >= turnThresh
        isUTurn(ii:min(N, ii+windowLen)) = true;
    end
end

% bridge small gaps
idx = find(isUTurn);
gapBridge = 3;
if ~isempty(idx)
    for kk = 2:numel(idx)
        if idx(kk) - idx(kk-1) <= gapBridge
            isUTurn(idx(kk-1):idx(kk)) = true;
        end
    end
end

v_max(isUTurn) = min(v_max(isUTurn), v5);

% smooth speed by acceleration limits (optional but recommended)
a_acc = 1.0;  % m/s^2
a_dec = 1.5;  % m/s^2

v_profile = v_max;

% forward pass
for ii = 2:N
    ds_i = norm(xy(ii,:) - xy(ii-1,:));
    v_profile(ii) = min(v_profile(ii), sqrt(max(0, v_profile(ii-1)^2 + 2*a_acc*ds_i)));
end

% backward pass
for ii = N-1:-1:1
    ds_i = norm(xy(ii+1,:) - xy(ii,:));
    v_profile(ii) = min(v_profile(ii), sqrt(max(0, v_profile(ii+1)^2 + 2*a_dec*ds_i)));
end

a_lat = (v_profile.^2) .* abs(kappa);

fprintf('Speed profile: v in [%.2f, %.2f] m/s (%.1f..%.1f km/h)\n', ...
    min(v_profile), max(v_profile), min(v_profile)*3.6, max(v_profile)*3.6);
fprintf('Max lateral accel (from v^2*kappa): %.2f m/s^2\n', max(a_lat));

% ---------------- Steering rate limit (actuator model) ----------------
% wheel_all is command (piecewise-constant). Create wheel_act that follows wheel_all
% with bounded rate: |d(delta)/dt| <= deltaRateMax.
wheel_act = apply_steering_rate_limit(wheel_all, ds, v_profile, deltaRateMax, v_min_for_dt);

% Convert to deg for plotting
wheel_cmd_deg = wheel_all * 180/pi;
wheel_act_deg = wheel_act * 180/pi;

% ---------------- Segmentation indices from mode_all ----------------
[segStartIdx, segEndIdx, segModes] = segment_by_mode(mode_all);

% ================== Plot segmented path with colored segments ==================
% Keep map/obstacles as drawn; overlay segmented path.
segLegendNames = strings(0,1);
segLegendHandles = gobjects(0,1);

for k = 1:numel(segStartIdx)
    i1 = segStartIdx(k);
    i2 = segEndIdx(k);
    m = segModes(k);

    if isKey(modeColors, m)
        c = modeColors(m);
        lw = 2.2;
        if m == transitionModeName
            lw = transitionLineWidth;
        end
    else
        c = [0 0 0];
        lw = 2.2;
    end

    hSeg = plot(route_all(i1:i2,1), route_all(i1:i2,2), '-', 'LineWidth', lw, 'Color', c);

    % legend: only one entry per mode
    if ~any(segLegendNames == m)
        segLegendNames(end+1,1) = m; %#ok<AGROW>
        segLegendHandles(end+1,1) = hSeg; %#ok<AGROW>
    end
end

% re-draw transition segments on top to ensure visibility
idxT = (mode_all == transitionModeName);
if any(idxT)
    plot(route_all(idxT,1), route_all(idxT,2), '-', 'Color', transitionColor, 'LineWidth', transitionLineWidth, 'HandleVisibility', 'off');
end

if ~isempty(segLegendHandles)
    legend(segLegendHandles, cellstr(segLegendNames), 'Location', 'best');
end

% ================== Plot wheel angles with background mode shading ==================
figure(2); 
clf;
hax = gca;
hold(hax, 'on');

% background shading: draw patches first
for k = 1:numel(segStartIdx)
    i1 = segStartIdx(k);
    i2 = segEndIdx(k);
    m  = segModes(k);

    if isKey(modeColors, m)
        c = modeColors(m);
    else
        c = [0 0 0];
    end

    s1 = s_all(i1);
    s2 = s_all(i2);

    % placeholder y-limits; will be updated after plotting lines
    patch(hax, [s1 s2 s2 s1], [-1 -1 1 1], c, 'FaceAlpha', modeAlpha, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

% plot wheel angle curves: command (dashed) + actuator (solid)
p1 = plot(s_all, wheel_act_deg(:,1), 'LineWidth', 1.6);
p2 = plot(s_all, wheel_act_deg(:,2), 'LineWidth', 1.6);
p3 = plot(s_all, wheel_act_deg(:,3), 'LineWidth', 1.6);
p4 = plot(s_all, wheel_act_deg(:,4), 'LineWidth', 1.6);

% command lines should not appear as extra legend items (e.g. data1)
plot(s_all, wheel_cmd_deg(:,1), '--', 'LineWidth', 1.0, 'Color', p1.Color, 'HandleVisibility', 'off');
plot(s_all, wheel_cmd_deg(:,2), '--', 'LineWidth', 1.0, 'Color', p2.Color, 'HandleVisibility', 'off');
plot(s_all, wheel_cmd_deg(:,3), '--', 'LineWidth', 1.0, 'Color', p3.Color, 'HandleVisibility', 'off');
plot(s_all, wheel_cmd_deg(:,4), '--', 'LineWidth', 1.0, 'Color', p4.Color, 'HandleVisibility', 'off');

% fix patch y-range to actual y limits
yl = ylim(hax);
patches = findobj(hax, 'Type', 'patch');
for pp = 1:numel(patches)
    xdata = patches(pp).XData;
    patches(pp).YData = [yl(1) yl(1) yl(2) yl(2)];
    patches(pp).XData = xdata;
end

grid on;
xlabel('Arc length s (m)');
ylabel('Steering angle (deg)');
legend([p1 p2 p3 p4], {'\delta_{fl} (act)','\delta_{fr} (act)','\delta_{rl} (act)','\delta_{rr} (act)'}, 'Location', 'best');
title(sprintf('4-wheel steering angles along path (mode-shaded), rate limited: %.0f deg/s', deltaRateMax_deg_s));


% ================== Animate ==================
size_car=[3 1.8 1.4];
[h1,h2]=plot_car(route_all(1,:),size_car);
dis_all=0;

% Draw full path once (faster animation)这里原来是画红线plot(route_all(:,1), route_all(:,2), 'r-', 'LineWidth', 2);


autoPause = false; % set true if you want fixed frame rate
fps = 30;

for ii=2:size(route_all,1)
    dis_all=dis_all+norm(route_all(ii,1:2)-route_all(ii-1,1:2));
    r_lin=route_all(ii,:);

    delete(h1)
    delete(h2)
    [h1,h2]=plot_car(r_lin,size_car);

    drawnow limitrate;
    if autoPause
        pause(1/fps);
    end
end

disp(['total distance: ', num2str(dis_all)])


% ================== helper: infer wheel angles from route curvature ==================
function wa = infer_wheel_angles_from_route(route, L, W, delta_max, counter_phase)
% route: Nx3
% returns Nx4 wheel angles [fl fr rl rr] (rad)
xy = route(:,1:2);
N = size(xy,1);

% curvature via 3-point formula
kappa = zeros(N,1);
for ii = 2:N-1
    p0 = xy(ii-1,:); p1 = xy(ii,:); p2 = xy(ii+1,:);
    a = norm(p1-p0);
    b = norm(p2-p1);
    c = norm(p2-p0);
    if a < 1e-6 || b < 1e-6 || c < 1e-6
        kappa(ii) = 0;
        continue;
    end
    area2 = abs( (p1(1)-p0(1))*(p2(2)-p0(2)) - (p1(2)-p0(2))*(p2(1)-p0(1)) );
    kappa(ii) = area2 / (a*b*c);
end
if N >= 2
    kappa(1) = kappa(2);
    kappa(end) = kappa(end-1);
end

wa = zeros(N,4);

for ii=1:N
    k = kappa(ii);
    if abs(k) < 1e-9
        wa(ii,:) = 0;
        continue;
    end

    R = 1/abs(k);
    signTurn = sign(k); % + left, - right (approx)

    % Ackermann inner/outer
    delta_in  = atan(L / max(1e-6, (R - W/2)));
    delta_out = atan(L / (R + W/2));

    if signTurn > 0
        d_fl = +delta_in;
        d_fr = +delta_out;
    else
        d_fl = -delta_out;
        d_fr = -delta_in;
    end

    if counter_phase
        d_rl = -d_fl;
        d_rr = -d_fr;
    else
        d_rl = d_fl;
        d_rr = d_fr;
    end

    % saturate
    d_fl = max(-delta_max, min(delta_max, d_fl));
    d_fr = max(-delta_max, min(delta_max, d_fr));
    d_rl = max(-delta_max, min(delta_max, d_rl));
    d_rr = max(-delta_max, min(delta_max, d_rr));

    wa(ii,:) = [d_fl d_fr d_rl d_rr];
end
end

function [segStartIdx, segEndIdx, segModes] = segment_by_mode(mode_all)
% segment consecutive identical strings into segments
mode_all = string(mode_all(:));
N = numel(mode_all);

if N == 0
    segStartIdx = []; segEndIdx = []; segModes = strings(0,1);
    return
end

changeIdx = find(mode_all(2:end) ~= mode_all(1:end-1)) + 1;
segStartIdx = [1; changeIdx];
segEndIdx = [changeIdx-1; N];
segModes = mode_all(segStartIdx);
end

function wheel_act = apply_steering_rate_limit(wheel_cmd, ds, v_profile, deltaRateMax, v_min)
% wheel_cmd: Nx4 (rad)
% ds: (N-1)x1 (m) step distances
% v_profile: Nx1 (m/s)
% deltaRateMax: rad/s
% v_min: m/s minimum speed for dt computation

N = size(wheel_cmd,1);
wheel_act = zeros(size(wheel_cmd));
wheel_act(1,:) = wheel_cmd(1,:);

for i = 2:N
    ds_i = ds(i-1);
    v_i = max(v_min, v_profile(i));
    dt = ds_i / v_i;
    maxDelta = deltaRateMax * dt;

    for w = 1:4
        err = wheel_cmd(i,w) - wheel_act(i-1,w);
        err = max(-maxDelta, min(maxDelta, err));
        wheel_act(i,w) = wheel_act(i-1,w) + err;
    end
end
end

function [route2, wheel2, mode2] = insert_curvature_transitions(route, wheel, mode, Lt, kappaJumpThresh, M, transitionModeName)
% Insert a short transition segment (clothoid-like) when curvature jump is large.
% route: Nx3 [x y psi]
% wheel: Nx4 wheel angles command (rad)
% mode: Nx1 string mode

route2 = route; wheel2 = wheel; mode2 = mode;

if Lt <= 0 || M < 3
    return
end

delta_s = Lt / (M-1);

% iterative insertion (update indices after insertion)
i = 2;
while i <= size(route2,1) - 2
    xy = route2(:,1:2);

    % local curvature estimates around i
    k1 = local_curvature_3pt(xy(i-1,:), xy(i,:), xy(i+1,:));
    k2 = local_curvature_3pt(xy(i,:), xy(i+1,:), xy(i+2,:));

    if ~isfinite(k1) || ~isfinite(k2)
        i = i + 1;
        continue
    end

    if abs(k2 - k1) <= kappaJumpThresh
        i = i + 1;
        continue
    end

    % Use the heading at i as starting heading
    x0 = route2(i,1);
    y0 = route2(i,2);
    psi0 = route2(i,3);

    % Build transition points (exclude the first point to avoid duplicates)
    trans = zeros(M,3);
    trans(1,:) = [x0 y0 psi0];

    for mm = 2:M
        s = (mm-2) * delta_s; % s in [0, Lt]
        k = k1 + (k2-k1) * (s / Lt);
        psi = trans(mm-1,3) + k * delta_s;
        x = trans(mm-1,1) + cos(psi) * delta_s;
        y = trans(mm-1,2) + sin(psi) * delta_s;
        trans(mm,:) = [x y psi];
    end

    % wheel & mode interpolation across transition
    wa1 = wheel2(i,:);
    wa2 = wheel2(i+1,:);
    wtrans = zeros(M,4);
    mtrans = strings(M,1);
    for mm = 1:M
        t = (mm-1)/(M-1);
        wtrans(mm,:) = (1-t)*wa1 + t*wa2;
        mtrans(mm) = transitionModeName;
    end

    % Insert transition after i (skip first trans point to avoid duplicating i)
    route2 = [route2(1:i,:); trans(2:end,:); route2(i+1:end,:)];
    wheel2 = [wheel2(1:i,:); wtrans(2:end,:); wheel2(i+1:end,:)];
    mode2  = [mode2(1:i,:);  mtrans(2:end,:); mode2(i+1:end,:)];

    % Jump past the inserted segment
    i = i + (M-1);
end
end

function k = local_curvature_3pt(p0, p1, p2)
% signed curvature estimate from 3 points
v1 = p1 - p0;
v2 = p2 - p1;
a = norm(v1);
b = norm(v2);
c = norm(p2 - p0);

if a < 1e-9 || b < 1e-9 || c < 1e-9
    k = 0;
    return
end

area2 = (p1(1)-p0(1))*(p2(2)-p0(2)) - (p1(2)-p0(2))*(p2(1)-p0(1));

k = area2 / (a*b*c); % signed
end

