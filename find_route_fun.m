function [isok,x,y,sita,route,meta]=find_route_fun(now_point,ind,step,min_r,min_r_ack,L,W,beta,dpsi,safe_dis,ob_coo)
% ind primitives:
% 1: counter-phase tight left  (R=min_r)
% 2: straight forward
% 3: counter-phase tight right (R=min_r)
% 7: same-phase / wide left    (R=min_r_ack, proxy)
% 8: same-phase / wide right   (R=min_r_ack, proxy)
% 9: crab left  (lateral +y in body frame), psi unchanged
% 10: crab right (lateral -y in body frame), psi unchanged
% 11: spin in-place left  (psi += dpsi), x,y unchanged
% 12: spin in-place right (psi -= dpsi), x,y unchanged

isok=0;
theta=now_point(3);
n2=20; % segment discretization

% default meta
meta.mode = "unknown";
meta.wheelAngles = [0 0 0 0]; % [delta_fl delta_fr delta_rl delta_rr]

% helpers
wheelAnglesFromRadius = @(R,signTurn,modeSamePhase) localWheelAngles(R,signTurn,modeSamePhase,L,W);

switch ind
    case 1  % tight left (counter-phase)
        R = min_r;  signTurn = +1;
        [route,x,y,sita] = arcPrimitive(now_point,step,R,signTurn,n2);
        meta.mode = "counter_phase";
        meta.wheelAngles = wheelAnglesFromRadius(R,signTurn,false);

    case 2  % straight
        [route,x,y,sita] = straightPrimitive(now_point,step,n2);
        meta.mode = "straight";
        meta.wheelAngles = [0 0 0 0];

    case 3  % tight right (counter-phase)
        R = min_r;  signTurn = -1;
        [route,x,y,sita] = arcPrimitive(now_point,step,R,signTurn,n2);
        meta.mode = "counter_phase";
        meta.wheelAngles = wheelAnglesFromRadius(R,signTurn,false);

    case 7  % wide left (same-phase proxy using larger radius)
        R = min_r_ack; signTurn = +1;
        [route,x,y,sita] = arcPrimitive(now_point,step,R,signTurn,n2);
        meta.mode = "same_phase";
        meta.wheelAngles = wheelAnglesFromRadius(R,signTurn,true);

    case 8  % wide right (same-phase proxy)
        R = min_r_ack; signTurn = -1;
        [route,x,y,sita] = arcPrimitive(now_point,step,R,signTurn,n2);
        meta.mode = "same_phase";
        meta.wheelAngles = wheelAnglesFromRadius(R,signTurn,true);

    case 9  % crab left: translate +y in body frame, psi unchanged
        [route,x,y,sita] = crabPrimitive(now_point,step,+1,n2);
        meta.mode = "crab";
        meta.wheelAngles = [beta beta beta beta];

    case 10 % crab right: translate -y in body frame
        [route,x,y,sita] = crabPrimitive(now_point,step,-1,n2);
        meta.mode = "crab";
        meta.wheelAngles = [-beta -beta -beta -beta];

    case 11 % spin in-place left
        [route,x,y,sita] = spinPrimitive(now_point,+1,dpsi,n2);
        meta.mode = "spin";
        meta.wheelAngles = localWheelAngles(min_r,+1,false,L,W);

    case 12 % spin in-place right
        [route,x,y,sita] = spinPrimitive(now_point,-1,dpsi,n2);
        meta.mode = "spin";
        meta.wheelAngles = localWheelAngles(min_r,-1,false,L,W);

    otherwise
        % unsupported primitive
        isok=1;
        x=now_point(1); y=now_point(2); sita=now_point(3);
        route=[x y sita];
        return
end

% collision check (keep original method)
for i=1:size(route,1)
    temp=route(i,1:2);
    for j=1:size(ob_coo,1)
        if norm(temp-ob_coo(j,:))<safe_dis+(2^0.5)/2
            isok=1;
            return
        end
    end
end

end

% ---------------- local helper functions ----------------

function [route,x,y,sita] = straightPrimitive(now_point,step,n2)
theta = now_point(3);
dvec = [step*cos(theta), step*sin(theta)];
dx = now_point(1)+linspace(0,dvec(1),n2);
dy = now_point(2)+linspace(0,dvec(2),n2);
angle = linspace(theta,theta,n2);
route=[dx' dy' angle'];
x=dx(end); y=dy(end); sita=angle(end);
end

function [route,x,y,sita] = arcPrimitive(now_point,step,R,signTurn,n2)
% signTurn: +1 left, -1 right
theta = now_point(3);
if signTurn > 0
    cenx = now_point(1)-R*sin(theta);
    ceny = now_point(2)+R*cos(theta);
    t = theta-pi/2+linspace(0,step/R,n2);
    dx = cenx+R*cos(t);
    dy = ceny+R*sin(t);
    angle = t+pi/2;
else
    cenx = now_point(1)+R*sin(theta);
    ceny = now_point(2)-R*cos(theta);
    t = theta+pi/2-linspace(0,step/R,n2);
    dx = cenx+R*cos(t);
    dy = ceny+R*sin(t);
    angle = t-pi/2;
end
route=[dx' dy' angle'];
x=dx(end); y=dy(end); sita=angle(end);
end

function [route,x,y,sita] = crabPrimitive(now_point,step,dirSign,n2)
% translate laterally in body frame by step, keep psi unchanged
% dirSign: +1 left, -1 right
x0=now_point(1); y0=now_point(2); psi=now_point(3);
moveDir = psi + dirSign*pi/2;
dx = x0 + linspace(0, step*cos(moveDir), n2);
dy = y0 + linspace(0, step*sin(moveDir), n2);
angle = linspace(psi, psi, n2);
route=[dx' dy' angle'];
x=dx(end); y=dy(end); sita=psi;
end

function [route,x,y,sita] = spinPrimitive(now_point,signTurn,dpsi,n2)
% in-place rotation: x,y fixed, psi changes
x0=now_point(1); y0=now_point(2); psi=now_point(3);
angle = psi + signTurn*linspace(0,dpsi,n2);
dx = x0*ones(1,n2);
dy = y0*ones(1,n2);
route=[dx' dy' angle'];
x=x0; y=y0; sita=angle(end);
end

function wa = localWheelAngles(R,signTurn,modeSamePhase,L,W)
% Returns [delta_fl delta_fr delta_rl delta_rr]
% signTurn: +1 left, -1 right
%
% Ackermann geometry (approx):
% delta_in  = atan(L/(R - W/2))
% delta_out = atan(L/(R + W/2))
delta_in  = atan(L / max(1e-6, (R - W/2)));
delta_out = atan(L / (R + W/2));

if signTurn > 0
    d_fl = +delta_in;
    d_fr = +delta_out;
else
    d_fl = -delta_out;
    d_fr = -delta_in;
end

if modeSamePhase
    % same-phase: rear wheels steer in same direction as front
    d_rl = d_fl;
    d_rr = d_fr;
else
    % counter-phase: rear wheels steer opposite to front
    d_rl = -d_fl;
    d_rr = -d_fr;
end

wa = [d_fl d_fr d_rl d_rr];
end
