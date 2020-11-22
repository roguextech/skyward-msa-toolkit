clear; clc; close all;
%% Load .mat file:
load('empty.mat');

% Find indeces for alpha = 0, beta = 0:
iAlpha = find(State.Alphas == 0); iBeta = find(State.Betas == 0);
if isempty(iAlpha) 
    error('State.Alphas must contain 0.');
end
if isempty(iBeta) 
    error('State.Betas must contain 0.');
end

CA = squeeze(Coeffs.CA(iAlpha,:,iBeta,:,:));

mach = State.Machs(:); alt = State.Altitudes(:); hProt = State.hprot(:);
nMach = length(mach); nAlt = length(alt); nHProt = length(hProt);

%% Body Alone:
CABody = CA(:,:,1);

mAlt = floor(nAlt/2); % mean index in altitude array

pMBody = polyfit(mach,CABody(:,mAlt),6); % interpolation wrt mach

% Add dependence on altitude:
% Create variables grids:
[altGrid, machGrid] = meshgrid(alt,mach);

% Interpolating function:
interpCABody = @(mach,alt,pMBody,pABody) polyval(pMBody,mach) + polyval(pABody,alt);

% Cost function to optimize pABody:
costFun1 = @(x) norm(CABody(:) - interpCABody(machGrid(:),altGrid(:),pMBody,x));

x0 = [0;0]; % Initial Guess

[pABody,fval] = fminsearch(costFun1,x0);

% Compute error:
CABodyInterp = interpCABody(machGrid,altGrid,pMBody,pABody);
errorBody = (CABodyInterp'-CABody');
errMaxBody = max(errorBody(:))

%% Protuberance Alone:
CAProt = CA - CABodyInterp;

% Degrees of polynomials:
degM = 5;  degHP = 2;

coeffsM = zeros(nHProt, degM+1); coeffsHP = zeros(degM + 1,degHP + 1);

% Interpolation wrt mach:
CAFixAlt = CAProt(:,mAlt,:);

for iHP = 1:nHProt
    
    coeffsM(iHP,:) = polyfit(mach,squeeze(CAFixAlt(:,iHP)),degM);
    
end

% Interpolation wrt hProt:
for iDM = 1:degM+1
    
    coeffsHP(iDM,:) = polyfit(hProt,squeeze(coeffsM(:,iDM)),degHP);
    
end

% Add dependence on altitude:
x0 = [0]; % Initial guess

opts = optimset('TolFun',1e-5,'Display','iter');

pAProt = fminsearch(@(x) costFun2(x,coeffsHP,mach,alt,hProt,CAProt),x0,opts);

[J,errMaxProt,CAProtInterp] = costFun2(pAProt,coeffsHP,mach,alt,hProt,CAProt);
errMaxProt

%% Total CA:
CAInterp = CABodyInterp + CAProtInterp;

errMax = max(abs(CA(:) - CAInterp(:)))
normErr = norm(CA(:) - CAInterp(:))

%% Group coefficients:
% CABody = mb0 + mb1*M + mb2*M^2 ... + mb6*M^6 + ab0 + ab1*A =
%          p0 + p1*M + p2*M^2 ... + p6*M^6 + p7*A

% CAProt = mp0 + ... + mp5*M^5 + ap1*A with mpi = hi0 + hi1*h + hi2*h^2
% CAProt = h00 + h10*M + h20*M^2 + h30*M^3....+h50*M^5 + ap1*A

% CA = (mb0 + ab0 + h00) + (mb1 + h10)*M +...+(mb5+h50)*M^5 + mb6*M^6 +
%       hi1*h*M^i + hi2*h^2*M^i (i = 0,...,5) + (ab1 + ap1)*A



% syms M A H 
% 
% CABM = sum(pMBody(:).*M.^[length(pMBody)-1:-1:0]');
% 
% CABA = sum(pABody(:).*A.^[length(pABody)-1:-1:0]');
% 
% 
% for l = 1:size(coeffsHP,1)
% 
%     cTemp = coeffsHP(l,:);
%     CAPM(l) = sum(cTemp(:).*H.^[length(cTemp)-1:-1:0]');
% 
% end
% 
% CAPMH = sum(CAPM(:).*M.^[length(CAPM)-1:-1:0]');
% 
% CAP = CAPMH + sum(pAProt(:).*A.^[length(pAProt)-1:-1:0]');
% 
% symCA = CAP + CABA + CABM;
% 
% myFunCA = matlabFunction(symCA,'File','myFunCA','Vars',[M,A,H]);
% 
% [M,A,H] = ndgrid(mach,alt,hProt);
coeffs.n000 = (pMBody(7) + pABody(2) + coeffsHP(6,3));
coeffs.n100 = (pMBody(6) + coeffsHP(5,3));
coeffs.n200 = (pMBody(5) + coeffsHP(4,3));
coeffs.n300 = (pMBody(4) + coeffsHP(3,3)); 
coeffs.n400 = (pMBody(3) + coeffsHP(2,3));
coeffs.n500 = (pMBody(2) + coeffsHP(1,3));
coeffs.n600 = pMBody(1);
coeffs.n010 = coeffsHP(6,2);
coeffs.n020 = coeffsHP(6,1);
coeffs.n110 = coeffsHP(5,2);
coeffs.n120 = coeffsHP(5,1);
coeffs.n210 = coeffsHP(4,2);
coeffs.n220 = coeffsHP(4,1);
coeffs.n310 = coeffsHP(3,2);
coeffs.n320 = coeffsHP(3,1);
coeffs.n410 = coeffsHP(2,2);
coeffs.n420 = coeffsHP(2,1);
coeffs.n510 = coeffsHP(1,2);
coeffs.n520 = coeffsHP(1,1);
coeffs.n001 = (pABody(1) + pAProt(1));

save coeffs coeffs

%%
function [J,errMax,CAProtInterp] = costFun2(x,coeffsHP,mach,alt,hProt,CAProt)

CAProtInterp = interpCAProt(mach,alt,hProt,coeffsHP,x);

J = norm(abs(CAProt(:) - CAProtInterp(:)));
errMax = max(abs(CAProt(:) - CAProtInterp(:)));

end

function CAProtInterp = interpCAProt(mach,alt,hProt,coeffsHP,pAProt)

nMach = length(mach); nAlt = length(alt); nHProt = length(hProt);

CAProtInterp = zeros(nMach,nAlt,nHProt);

for i = 1:nMach
    for j = 1:nAlt
        for k = 1:nHProt
            
            coeffsM = zeros(1,size(coeffsHP,1));
            for l = 1:size(coeffsHP,1)
                
                coeffsM(l) = polyval(coeffsHP(l,:),hProt(k));
                
            end
            
            CAProtInterp(i,j,k) = polyval(coeffsM,mach(i)) + pAProt*alt(j);
            
        end
    end
end

end





