% script for saving data for testing scs apo evaluator 

close all

t = [data_ascent.integration.t data_bal.integration.t];
p = [data_ascent.air.P data_bal.air.P];
h = - [data_ascent.state.Y(:,3) ; data_bal.state.Y(:,3)];
y = - [data_ascent.state.Y(:,2) ; data_bal.state.Y(:,2)];
x = - [data_ascent.state.Y(:,1) ; data_bal.state.Y(:,1)];
v_vertical = [data_ascent.velocities(:,3) ; data_bal.velocities(:,3)];
body_acc = [data_ascent.accelerations.body_acc ; data_bal.accelerations.body_acc];
ang_acc = [data_ascent.accelerations.ang_acc ; data_bal.accelerations.ang_acc];
ang_vel = [data_ascent.state.Y(:,7:9) ; data_bal.state.Y(:,7:9)];
thrust = [data_ascent.forces.T]';
mass=[4.5+0.345 4.5];
mfr=0.345/1.61;



save('test_apo.mat','t','p','h','x','y','v_vertical','body_acc','ang_acc','ang_vel','thrust','mass','mfr','pitch_angle','roll_angle','yaw_angle');

clearvars 


