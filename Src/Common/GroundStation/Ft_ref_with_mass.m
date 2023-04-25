function Ft_ref = Ft_ref_with_mass(v_ro, control_max_tether_force)
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
% TODO: Obtain from winch parameters/constraints.
theta = [146429.5 28806.3 28242.9];
Ft_min = 0.5e6;
Ft_max = 1e6;

% For a certain vr, Ft_star gets below the minimum tether force, so we
% command the minimum tether force. max(Ft_star, Ft_min) does not work as
% for low vr the quadratic relation for Ft_star might get higher than
% Ft_min again but it is not valid in that region.
% TODO: Potential improvement: make the transition smooth.
Ft_ref = theta(1) + theta(2)*v_ro + theta(3)*v_ro.^2;
Ft_ref(v_ro < 3.0648) = Ft_min;
if control_max_tether_force
    Ft_ref(v_ro > 5.0111) = Ft_max;
end
