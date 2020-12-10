function [AMtime] = autoMatricesProtub(datcom,vars)

tic

n_hprot = length(vars.hprot);
Geometry.Chord1 = datcom.Chord1;
Geometry.Chord2 = datcom.Chord2;
Geometry.Height = datcom.Height;
Geometry.shape = datcom.shape;
Geometry.D = datcom.D;
Geometry.Lnose = datcom.Lnose;
Geometry.Lcenter = datcom.Lcenter;
Geometry.Npanel = datcom.Npanel;
Geometry.OgType = datcom.OgType;
Geometry.xcg = vars.xcg(1);
datcom.config = 'AllRocket';

%% datcom
for k = 1:1:3
    if k == 1 % datcom matrices in full configuration (without aerobrakes)
        datcom.xcg = vars.xcg(k);
        datcom.hprot = vars.hprot(1);
        clc
        fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
        createFor006(datcom);
        [CoeffsF, State] = datcomParser5('full',Geometry);
        clc
        perc = round(100/(n_hprot + 1)) ;
        fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
        fprintf(' Progress %d %% \n', perc);
        State.hprot = vars.hprot;
        CoeffsE = struct();
        fn = fieldnames(CoeffsF);
        for f = 1:numel(fn)
            CoeffsE.(fn{f}) = zeros([size(CoeffsF.(fn{f})),n_hprot]);
        end
    elseif k == 2 % datcom matrices in empty configuration with aerobrakes variations
        datcom.xcg = vars.xcg(k);
        for n = 1:n_hprot
            datcom.hprot = vars.hprot(n);
            createFor006(datcom);
            clc
            perc = round((n)/(n_hprot+1)*(100)) ;
            fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
            fprintf(' Progress %d %% \n', perc);
            currentCoeffs = datcomParser5();

            for f = 1:numel(fn)
                CoeffsE.(fn{f})(:,:,:,:,n) = currentCoeffs.(fn{f});
            end
        end
    elseif k == 3 % datcom descending matrices used for descending phase
        datcom.xcg = vars.xcg(1) - datcom.Lnose;
        datcom.hprot = vars.hprot(1);
        datcom.Lnose = 0;
        datcom.d = datcom.Lcenter - datcom.Chord1;
        Geometry_DesBoOn.Chord1 = datcom.Chord1;
        Geometry_DesBoOn.Chord2 = datcom.Chord2;
        Geometry_DesBoOn.Height = datcom.Height;
        Geometry_DesBoOn.shape = datcom.shape;
        Geometry_DesBoOn.D = datcom.D;
        Geometry_DesBoOn.Lnose = datcom.Lnose;
        Geometry_DesBoOn.Lcenter = datcom.Lcenter;
        Geometry_DesBoOn.Npanel = datcom.Npanel;
        Geometry_DesBoOn.OgType = datcom.OgType;
        Geometry_DesBoOn.xcg = datcom.xcg;
        datcom.config = 'CenterBodyOnly_descent';
        clc
        fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
        createFor006(datcom);
        [CoeffsF, State] = datcomParser5('emptyDescent_BodyOnly',Geometry_DesBoOn);
        clc
        perc = round(100/(n_hprot + 1)) ;
        fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
        fprintf(' Progress %d %% \n', perc);
    end
end

clc
fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
fprintf(' Progress %d %% \n', 100);

%% Save joined empty .mat file
Coeffs = CoeffsE;
Geometry.xcg = vars.xcg(2);
save('empty','State','Coeffs','Geometry');

%%

delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat')

AMtime = toc;