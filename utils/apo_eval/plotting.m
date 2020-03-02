load data

figure()


plot(data.ms1,data.apogee1,'-o')
hold on 
plot(data.ms2,data.apogee2,'-o') 
xlabel('structural mass [kg]')
ylabel('apogee [m]')
legend('RMS 38/720 - I327', 'RMS 38/600 - I366')