function [allSteps] = recallOdeFcn(fun, T, Y, varargin)
%{

RECALLODEFCN - This function allows to compute some parameters used
inside the ODE integrations

INPUTS:
            - fun, ode function used;
            - T, integration time vector;
            - Y, state matrix.

OUTPUTS:
            - all_steps, structure which contains all the parameters needed.

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept
email: adriano.filippo.inno@skywarder.eu
Release date: 16/11/2018

%}
[~,firstStep] = fun(T(1),Y(1,:),varargin{:});

namesFields = fieldnames(firstStep);
NT = length(T);

steps = cell(NT,1);
for k = 1:NT
    [~,steps{k}] = fun(T(k),Y(k,:),varargin{:});
end

for i = 1:numel(namesFields)
    currentFieldName = namesFields{i};
    currentField = firstStep.(currentFieldName);
    
    if isstruct(currentField)
        namesArrays = fieldnames(currentField);
        
        for j = 1:numel(namesArrays)
            currentArrayName = namesArrays{j};
            currentArray = currentField.(currentArrayName);
            sizeArray = size(currentArray);
            sizeArrayCut = sizeArray;
            
            if any(sizeArray == 1)
                sizeArrayCut = max(sizeArray);
            end
            
            allSteps.(currentFieldName).(currentArrayName) = zeros([sizeArrayCut,NT]);
            
            for k = 1:NT
                currentStep = steps{k};
                
                if all(sizeArray ~= 1)
                    allSteps.(currentFieldName).(currentArrayName)(:,:,k) =...
                        currentStep.(currentFieldName).(currentArrayName);
                else
                    allSteps.(currentFieldName).(currentArrayName)(:,k) =...
                        currentStep.(currentFieldName).(currentArrayName);
                end
            end
            
        end
        
    else
        sizeArray = size(currentField);
        
        sizeArrayCut = sizeArray;
        if any(sizeArray == 1)
            sizeArrayCut = max(sizeArray);
        end
        
        allSteps.(currentFieldName) = zeros([sizeArrayCut,NT]);
        
        for k = 1:NT
            currentStep = steps{k};
            
            if all(sizeArray ~= 1)
                allSteps.(currentFieldName)(:,:,k) =...
                    currentStep.(currentFieldName);
            else
                allSteps.(currentFieldName)(:,k) =...
                    currentStep.(currentFieldName);
            end
        end
        
    end
    
end
end

