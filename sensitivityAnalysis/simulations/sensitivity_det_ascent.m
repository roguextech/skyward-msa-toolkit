function [X, ApoTime, data_ascent] = sensitivity_det_ascent(settings)
%{ 

SENSITIVITY_DET_ASCENT - This function runs a simulation of the
ascent phase with determistic variations of the coefficients

INTPUTS: 
            - settings, rocket data structure;

OUTPUTS:
            - X, Apogee coordinates in NED
            - ApoTime, times when the apogee is reached; 
            - data_ascent, Usefull values for the plots.

Author: Luca Facchini
Skyward Experimental Rocketry | AFD Dept
email: luca.facchini@skywarder.eu
Revision date: 22/12/2020

%}


%% STARTING CONDITIONS (EQUAL FOR ALL)

% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

tf = settings.ode.final_time;

Az = settings.wind.Az;
El = settings.wind.El;
Magw = settings.wind.Mag;

% ascent phase computation
OMEGA = settings.OMEGA;
PHI = settings.PHI;

% WIND GENERATION
[uw, vw, ww, Azw] = wind_const_generator(Az, Az, El, El, Magw, Magw);

% Attitude
if settings.upwind
    PHI = mod(Azw + pi, 2*pi);
end

Q0 = angle2quat(PHI, OMEGA, 0*pi/180, 'ZYX')';
Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];

% Nominal values
CoeffsNominalF = settings.CoeffsF;
CoeffsNominalE = settings.CoeffsE;
msNominal = settings.ms;

%% PREALLOCATION
Nperc = length(settings.sensitivity.MeanCoeffVarPerc);
N = settings.sensitivity.N;
Ncoeffs = length(settings.sensitivity.param);

% Preallocation 
ApoTime = zeros(Nperc,Ncoeffs);
X = zeros(3,Nperc,Ncoeffs);
DeltaMean = settings.sensitivity.MeanCoeffVarPerc;


%% PARFOR LOOP
dimCF = size(CoeffsNominalF.CA);
dimCE = size(CoeffsNominalE.CA);

parfor_progress(Ncoeffs);
parpool;

parfor j=1:Ncoeffs
    COEFF_NAME = string(settings.sensitivity.param{j});
    for i = 1:Nperc
        
        if strcmp(COEFF_NAME,'m') || strcmp(COEFF_NAME,'ms')
            ms = msNominal*(1 + DeltaMean(i));
            
            % ASCENT
            [Ta,Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, ...
                settings, uw, vw, ww, COEFF_NAME, ms);
        else
            % Reassing default values
            CoeffsF = CoeffsNominalF;
            CoeffsE = CoeffsNominalE;
            % Pertubation of the proper field
            CoeffsF.(COEFF_NAME) = CoeffsNominalF.(COEFF_NAME).*(1 + DeltaMean(i));
            CoeffsE.(COEFF_NAME) = CoeffsNominalE.(COEFF_NAME).*(1 + DeltaMean(i));
            
            % ASCENT
            [Ta,Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, ...
                settings, uw, vw, ww, COEFF_NAME, CoeffsF, CoeffsE);
        end
        
        X(:,i,j) = [Ya(end,1); Ya(end,2); -Ya(end,3)];
        ApoTime(i,j) = Ta(end);
        
        if strcmp(COEFF_NAME,'m') || strcmp(COEFF_NAME,'ms')
            [data_ascent{i,j}] = RecallOdeFcn(@ascent, Ta, Ya, settings, ...
                uw, vw, ww, COEFF_NAME, ms);
        else
            [data_ascent{i,j}] = RecallOdeFcn(@ascent, Ta, Ya, settings, ...
                uw, vw, ww, COEFF_NAME, CoeffsF, CoeffsE);
        end
        data_ascent{i,j}.state.Y = Ya;
        data_ascent{i,j}.state.T = Ta;
    end
    parfor_progress;
end

% if settings.sensitivity.stoch == 0
%     
% else
%      parfor_progress(N);
%      parpool;
%      COEFF_NAME = string(settings.sensitivity.param{1});  
%      parfor i = 1:N
%          if strcmp(COEFF_NAME,'m') || strcmp(COEFF_NAME,'ms')
%             ms = msNominal + msNominal*DeltaMax.*(rand()-0.5)*2;
%                 
%             % ASCENT
%             [Ta,Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, ...
%                 settings, uw, vw, ww, COEFF_NAME, ms);
%          else
%              % Reassing default values
%              CoeffsF = CoeffsNominalF;
%              CoeffsE = CoeffsNominalE;
%          
%              deltaF = CoeffsNominalF.(COEFF_NAME).*DeltaMax.*(rand(dimCF)-0.5)*2;
%              deltaE = CoeffsNominalE.(COEFF_NAME).*DeltaMax.*(rand(dimCE)-0.5)*2;
%              CoeffsF.(COEFF_NAME) = CoeffsNominalF.(COEFF_NAME) + deltaF;
%              CoeffsE.(COEFF_NAME) = CoeffsNominalE.(COEFF_NAME) + deltaE;
%              
%              % ASCENT
%              [Ta,Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, settings, ...
%                  uw, vw, ww, COEFF_NAME, CoeffsF, CoeffsE);
%          end
%          
%          X(:,i) = [Ya(end,1); Ya(end,2); -Ya(end,3)];
%          ApoTime(i) = Ta(end);
%          parfor_progress;
%      end     
% end
delete(gcp('nocreate'))
delete('parfor_progress.txt')
