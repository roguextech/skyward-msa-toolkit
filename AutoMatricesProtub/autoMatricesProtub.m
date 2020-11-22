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

%% datcom
for k = 1:2
    datcom.xcg = vars.xcg(k);
    if k == 1
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
    else
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
    end
end

%% Save joined empty .mat file
Coeffs = CoeffsE;
Geometry.xcg = datcom.xcg;
save('empty','State','Coeffs','Geometry');

%%

delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat')

AMtime = toc;