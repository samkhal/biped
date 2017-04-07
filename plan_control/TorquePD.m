function  torques = TorquePD(q_act, q_d, dq_act, dq_d) 
    Kp = eye(10,10)*5;
    Kd = eye(10,10)*1.5;
    K_grav = 1.1;
    global r torques_temp states_temp
    [H C B dH dC dB] = r.manipulatorDynamics([0; 0; 0.6010; 0.0001; -0.0028; 0;q_d], zeros(r.getNumVelocities,1));
    N = C(7:16);
    torques = -Kp*(q_act - q_d) + N*K_grav;% -Kd*(dq_act - dq_d);
    
    torques_temp = [torques_temp;torques'];
    states_temp = [states_temp;q_d'];
end


