function Ft_ref = Ft_ref_with_mass(v_ro, control_max_tether_force, Ft_min, Ft_max)
% Calculate the reference tether force for the current reel-out speed.
% The ideal curve was found using a quasi-steady model with mass and taking
% an average operating condition. This was combined with a minimum tether
% force to keep the kite from stalling. For more details, see my MSc
% thesis.
% Ideally the kite makes sure the system does not exceed the max tether
% force (in the 2-phase strategy). Before that's implemented, can test just
% the winch by enabling control_max_tether_force.
% Can calculate the reference tether force for single inputs or arrays.
% author: Jesse Hummel
theta = [146429.5 28806.3 28242.9];

% For a certain vr, Ft_star gets below the minimum tether force, so we
% command the minimum tether force. max(Ft_star, Ft_min) does not work as
% for low vr the quadratic relation for Ft_star might get higher than
% Ft_min again but it is not valid in that region.
% TODO: Potential improvement: make the transition smooth.
Ft_ref = theta(1) + theta(2)*v_ro + theta(3)*v_ro.^2;

% Calculate the reeling speed where we hit the min and max tether force.
v_ro_Ft_min = abc(theta(3), theta(2), theta(1)-Ft_min);
v_ro_Ft_max = abc(theta(3), theta(2), theta(1)-Ft_max);

Ft_ref(v_ro < v_ro_Ft_min) = Ft_min;
if control_max_tether_force
    Ft_ref(v_ro > v_ro_Ft_max) = Ft_max;
end
end

function x_pos = abc(a, b, c)
    d = b.^2 - 4*a.*c;
    x_pos = (-b + sqrt(d)) / (2*a);
end