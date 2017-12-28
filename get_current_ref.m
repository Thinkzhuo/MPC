%input :time;
%output: return current reference data;
function [cur_x_ref,cur_y_ref,cur_phi_ref,cur_velocity_ref,cur_delta_ref] = get_current_ref(t)

global angle_ref angle_time Theta_ref Time velocity_ref X_ref_final Y_ref_final

%using interpolation method to get the current reference data to construct
%the linear model;
cur_x_ref = interp1(Time,X_ref_final,t);
cur_y_ref = interp1(Time,Y_ref_final,t);
cur_phi_ref = interp1(Time,Theta_ref,t);
cur_velocity_ref = interp1(Time,velocity_ref,t);
cur_delta_ref = interp1(angle_time,angle_ref,t);