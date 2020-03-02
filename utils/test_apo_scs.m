% script for saving data for testing scs apo evaluator 

close all

t = [data_ascent.integration.t data_bal.integration.t];
p = [data_ascent.air.P data_bal.air.P];
h = - [data_ascent.state.Y(:,3)' data_bal.state.Y(:,3)'];
v_vertical = [data_ascent.velocities(:,3)' data_bal.velocities(:,3)'];

save('test_apo.mat','t','p','h','v_vertical');

clearvars 


