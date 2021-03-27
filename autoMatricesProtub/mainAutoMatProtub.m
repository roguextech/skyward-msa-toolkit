%{

%}

close all
clear 
clc

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));

%% LOAD DATA
dataPath = '../data/';
addpath(dataPath);
simulationsData;
configAutoMatProtub;

tic

Geometry.Chord1 = settings.Chord1;
Geometry.Chord2 = settings.Chord2;
Geometry.Height = settings.Height;
Geometry.shape = settings.shape;
Geometry.D = settings.C;
Geometry.Lnose = settings.Lnose;
Geometry.Lcenter = settings.Lcenter;
Geometry.Npanel = settings.Npanel;
Geometry.OgType = settings.OgType;
Geometry.xcg = settings.xcg(1);

datcom.Chord1 = settings.Chord1;
datcom.Chord2 = settings.Chord2;
datcom.Height = settings.Height;
datcom.shape = settings.shape;
datcom.OgType = settings.OgType;

%% datcom
n_hprot = length(vars.hprot);
for k = 1:2
    datcom.xcg = settings.xcg(k);
    if k == 1
        datcom.hprot = vars.hprot(1);
        clc
        fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
        createFor006(datcom, settings);
        [CoeffsF, State] = datcomParser5('full',Geometry);
        clc
        perc = round(100/(n_hprot + 1)) ;
        fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
        fprintf(' Progress %d %% \n', perc);
        State.hprot = datcom.hprot;
        CoeffsE = struct();
        fn = fieldnames(CoeffsF);
        for f = 1:numel(fn)
            CoeffsE.(fn{f}) = zeros([size(CoeffsF.(fn{f})),n_hprot]);
        end
    else
        for n = 1:n_hprot
            datcom.hprot = vars.hprot(n);
            createFor006(datcom, settings);
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

clc
fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
fprintf(' Progress %d %% \n', 100);

%% Save joined empty .mat file
Coeffs = CoeffsE;
Geometry.xcg = datcom.xcg;
save('empty','State','Coeffs','Geometry');

%%

delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat')

AMtime = toc;