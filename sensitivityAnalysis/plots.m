%{

PLOTS - this script plots the computed data

Author: Luca Facchini
Skyward Experimental Rocketry | AFD Dept | crd@skywarder.eu
email: luca.facchini@skywarder.eu
Release date: 23/12/2020

%}

Ncoeff = length(settings.sensitivity.param);
Delta = settings.sensitivity.MeanCoeffVarPerc;
ND = length(settings.sensitivity.MeanCoeffVarPerc);



if settings.sensitivity.stoch == 0
    zapo = squeeze(X(3,:,:));
    if Ncoeff == 1
        zapo = zapo';
    end
    
    %%% Apogee variation
    figure('Name','Apogee - Ascent Phase'); hold on
    colors = colormap(hsv(Ncoeff));
    markertype = {'o','x','d','v','^','*','+','s','p','h','>','<'};
    for i=1:Ncoeff
        if Ncoeff <= 12
            plot(settings.sensitivity.MeanCoeffVarPerc*100,zapo(:,i),...
                'color',colors(i,:),'marker',markertype{i},'linestyle','-');
        else % Use same colors but different linestyle
            plot(settings.sensitivity.MeanCoeffVarPerc*100,zapo(:,i),...
                'color',colors(i,:),'marker',markertype{i-12},'linestyle','--');
        end
    end
    legend(settings.sensitivity.param);
    xlabel('Deviation % w.r.t. nominal value');
    ylabel('Apogee [m]');

%     %%% Percentage gain wrt nominal value
%     indNominal = find(Delta==0);
%     ApoNominal = zapo(indNominal,:);
%     ApoDiffValue = zapo-ApoNominal;
%     ApoDiffPerc = ApoDiffValue./ApoNominal*100;
% 
%     figure, hold on
%     for i=1:Ncoeff
%         if Ncoeff <= 12
%             plot(settings.sensitivity.MeanCoeffVarPerc*100,ApoDiffPerc(:,i),...
%                 'color',colors(i,:),'marker',markertype{i},'linestyle','-');
%         else
%             plot(settings.sensitivity.MeanCoeffVarPerc*100,ApoDiffPerc(:,i),...
%                 'color',colors(i,:),'marker',markertype{i-12},'linestyle','--');
%         end
%     end
%     xlabel('Deviation % w.r.t. nominal value');
%     ylabel('Apogee Gain [%]');
%     legend(settings.sensitivity.param);
    
    %%% Max Aero Forces
    figure('Name','Forces - Ascent Phase');
   
    subplot(2,2,[1 2]), hold on
    for i=1:Ncoeff
        temp = {data_ascent{:,i}}';
        MaxAeroForces = zeros(ND,3);
        for j = 1:ND
            MaxAeroForces(j,:) = max(temp{j,1}.forces.AeroDyn_Forces);
        end
        plot(settings.sensitivity.MeanCoeffVarPerc*100, MaxAeroForces(:,1),...
                'color',colors(i,:),'marker',markertype{i},'linestyle','-'),
    end
    grid on
    xlabel('Deviation % w.r.t. nominal value'); ylabel('Max X-body force [N]')
    legend(settings.sensitivity.param);
    
    subplot(2,2,3), hold on
    for i=1:Ncoeff
        temp = {data_ascent{:,i}}';
        MaxAeroForces = zeros(ND,3);
        for j = 1:ND
            MaxAeroForces(j,:) = max(temp{j,1}.forces.AeroDyn_Forces);
        end
        plot(settings.sensitivity.MeanCoeffVarPerc*100, MaxAeroForces(:,2),...
                'color',colors(i,:),'marker',markertype{i},'linestyle','-'),
    end
    grid on
    xlabel('Deviation % w.r.t. nominal value'); ylabel('Max Y-body force [N]')
    legend(settings.sensitivity.param);
    
    subplot(2,2,4), hold on
    for i=1:Ncoeff
        temp = {data_ascent{:,i}}';
        MaxAeroForces = zeros(ND,3);
        for j = 1:ND
            MaxAeroForces(j,:) = max(temp{j,1}.forces.AeroDyn_Forces);
        end
        plot(settings.sensitivity.MeanCoeffVarPerc*100, MaxAeroForces(:,3),...
                'color',colors(i,:),'marker',markertype{i},'linestyle','-'),
    end
    grid on
    xlabel('Deviation % w.r.t. nominal value'); ylabel('Max Z-body force [N]')
    legend(settings.sensitivity.param);
    
    %%% Max acceleration
    figure('Name','Acceleration - Ascent Phase');
    hold on
    for i=1:Ncoeff
        temp = {data_ascent{:,i}}';
        MaxAcc = zeros(ND,1);
        for j = 1:ND
            MaxAcc(j) = max(temp{j,1}.accelerations.body_acc(:,1));
        end
        MaxAcc = MaxAcc/settings.g0;
        plot(settings.sensitivity.MeanCoeffVarPerc*100, MaxAcc,...
                'color',colors(i,:),'marker',markertype{i},'linestyle','-'),
    end
    grid on
    xlabel('Deviation % w.r.t. nominal value'); ylabel('Max acceleration [g]')
    legend(settings.sensitivity.param);
    
    %%% Max velocity
    figure('Name','Max velocity - Ascent Phase'); hold on
    for i=1:Ncoeff
        temp = {data_ascent{:,i}}';
        V_max = zeros(ND,1);
        for j = 1:ND
            V_max(j) =  max(vecnorm(temp{j,1}.state.Y(:, 4:6),2,2));
        end
        plot(settings.sensitivity.MeanCoeffVarPerc*100, V_max,...
                'color',colors(i,:),'marker',markertype{i},'linestyle','-'),
    end
    grid on
    xlabel('Deviation % w.r.t. nominal value'); ylabel('Max velocity [m/s]')
    legend(settings.sensitivity.param);
    
    
    %%% Velocities at apogee
    figure('Name','Apogee velocity - Ascent Phase'); hold on
    for i=1:Ncoeff
        temp = {data_ascent{:,i}}';
        V_apo = zeros(ND,1);
        for j = 1:ND
            V_apo(j) =  norm(temp{j,1}.state.Y(end, 4:6) - temp{j,1}.wind.body_wind(1:3, end)');
        end
        plot(settings.sensitivity.MeanCoeffVarPerc*100, V_apo,...
                'color',colors(i,:),'marker',markertype{i},'linestyle','-'),
    end
    grid on
    xlabel('Deviation % w.r.t. nominal value'); ylabel('Apogee velocity [m/s]')
    legend(settings.sensitivity.param);
    
    
else
    
    zapom = mean(X(3,:));
    zstd = std(X(3,:));
    ApoTimem = mean(ApoTime);
    ApoTimestd = std(ApoTime);
    text = ['Mean Altitude: %3.3f m || STD: %3.3f m\n',...
        'Mean Apogee Time: %3.3f s || STD: %3.3f s\n'];
    fprintf(text, zapom, zstd, ApoTimem, ApoTimestd);
    
    h = histogram(X(3,:),30,'normalization','count'); hold on
    xlabel('Apogee [m]');
    ylabel('Count');
    plot([zapom zapom],[0 max(h.Values)*1.1],'-b','linewidth',1);
    errorbar(zapom,max(h.Values)/2,zstd,'horizontal','-r','linewidth',1)
    ylim([0 max(h.Values)*1.1]);
    legend('Apogee distribution','Mean apogee','Mean apogee \pm std','location','nw')
    title('Apogee distribution');
end



