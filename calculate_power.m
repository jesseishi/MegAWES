function [P_mech_in, P_mech_out, P_elec_out] = calculate_power(Ft, tau, v_r, winchParameter)
%calculate_power Calculates power input of kite to winch, output of winch
%to generator and output of generator to electricity.

P_mech_in = Ft * v_r;

P_mech_out = v_r / winchParameter.radius * -tau;

P_elec_out = P_mech_out;
P_elec_out.Data(P_elec_out.Data>0) = P_elec_out.Data(P_elec_out.Data>0) * params.n_generator;
P_elec_out.Data(P_elec_out.Data<0) = P_elec_out.Data(P_elec_out.Data<0) / (params.n_storage * params.n_motor);

end
