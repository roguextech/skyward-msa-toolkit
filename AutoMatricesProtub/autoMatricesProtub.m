function [AMtime] = autoMatricesProtub(datcom,vars)

tic

n_hprot = length(vars.hprot);

%% datcom
for k = 1:2
    datcom.xcg = vars.xcg(k);
    if k == 1
        datcom.hprot = vars.hprot(1);
        clc
        fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
        createFor006(datcom);
        [CoeffsF, State] = datcomParser5('full');
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
save('empty','State','Coeffs');

%%

delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat')

AMtime = toc;