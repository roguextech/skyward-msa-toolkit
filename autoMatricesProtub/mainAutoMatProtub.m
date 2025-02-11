%{
mainAutoMatProtub - Main script to compute aerodynamic coefficients using
                    Missile DATCOM. Two MAT-file are created: "full.mat"
                    and "empty.mat" containing the data of full and empty
                    missile configurations.

CALLED SCRIPTS: simulationsData, configAutoMatProtub.

CALLED FUNCTIONS: createFor006, datcomParser.

CALLED DATA FILES: -

REVISIONS:
- 0     18/10/2019, release     Adriano Filippo Inno, Giulio Pacifici

- 1     27/03/2021, Compatibility with common functions folder
                    Adriano Filippo Inno
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

datcomPath = '../commonFunctions/Datcom/';
if ismac
    if ~isfile(fullfile(datcomPath, 'datcom'))
        error('datcom is missing')
    end
else
    if ~isfile(fullfile(datcomPath, 'datcom.exe'))
        error('datcom.exe is missing')
    end
end

%% LOAD DATA
dataPath = '../data/';
addpath(dataPath);
commonFunctionsPath = '../commonFunctions/';
addpath(genpath(commonFunctionsPath));
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
        createFor006(datcom, settings, datcomPath);
        [CoeffsF, State] = datcomParser('full', Geometry);
        clc
        perc = round(100/(n_hprot + 1)) ;
        fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
        fprintf(' Progress %d %% \n', perc);
        State.hprot = datcom.hprot;
        CoeffsE = struct();
        fn = fieldnames(CoeffsF);
        for f = 1:numel(fn)
            CoeffsE.(fn{f}) = zeros([size(CoeffsF.(fn{f})), n_hprot]);
        end
    else
        for n = 1:n_hprot
            datcom.hprot = vars.hprot(n);
            createFor006(datcom, settings, datcomPath);
            clc
            perc = round((n)/(n_hprot+1)*(100)) ;
            fprintf('----------------- Aerobrakes Aerodynamics Prediction ----------------- \n')
            fprintf(' Progress %d %% \n', perc);
            currentCoeffs = datcomParser();

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
save('empty', 'State', 'Coeffs', 'Geometry');

%%
cd(datcomPath)
delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat')

AMtime = toc;