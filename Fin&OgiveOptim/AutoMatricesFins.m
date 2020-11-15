%{

Auto_Matrices - This script compiles many aerodynamic prediction using MISSILE
DATCOM to optimize rocket fins.
The output is a cell array composed by structures in which the aerodynamic
coefficients are stored.
In each prediction just the fin parameters are varying.
Check section " Design Parameters " to compose the for loop

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 18/10/2019

Author: Mauro De Francesco
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: mauro.defrancesco@skywarder.eu

%}
function [data, AMtime] = AutoMatricesFins(datcom, print)

tic

%% recalling the variables
MD = struct();
MD.Mach = datcom.s.Mach;
MD.Alpha = datcom.s.Alpha;
MD.Beta = datcom.s.Beta;
MD.Alt = datcom.s.Alt;

Nn = numel(datcom.design.Lnose);
if Nn ~= 1
    MD.Lnose = datcom.design.Lnose(ceil(Nn/2));
else
    MD.Lnose = datcom.design.Lnose;
end

Chord1 = datcom.design.Chord1;
Chord2 = datcom.design.Chord2;
Heigth = datcom.design.Heigth;
shape = datcom.design.Shape;

xcg = datcom.para.xcg;
MD.D = datcom.para.D;

MD.Lcenter = datcom.para.Lcenter;
MD.Npanel = datcom.para.Npanel;
MD.Phif = datcom.para.Phif;
MD.Ler = datcom.para.Ler;
MD.d = datcom.para.d;
MD.zup_raw = datcom.para.zup_raw;
MD.Lmaxu_raw = datcom.para.Lmaxu_raw;

MD.OgType = 'KARMAN';

N1 = length(Chord1);
N2 = length(Chord2);
N3 = length(Heigth);
Ns = length(shape);

%% datcom
data = cell(N1, N2, N3, Ns);
mass_condition = {'full', 'empty'};

for i = 1:N1
    C1 = Chord1(i);
    
    for j = 1:N2
        C2 = Chord2(j);
        
        if C2 >= C1
            break
        end
        
        for hh = 1:N3
            H = Heigth(hh);
            
            for s = 1:Ns
                
                MD.shape = shape(s);
                
                for k = 1:2
                    if print
                        clc
                        perc = round((i-1)/N1*100 + (j-1)/N2*(100/N1) + (hh-1)/N3*(100/N1/N2) + (s-1)/Ns*(100/N1/N2/N3) + (k-1)/2*(100/N1/N2/N3/Ns));
                        fprintf('----------------- Fins Aerodynamics Prediction ----------------- \n')
                        fprintf(' Progress %d %% \n', perc);
                    end
                    
                    MD.xcg = xcg(k);
                    
                    MD.Chord1 = C1;
                    MD.Chord2 = C2;
                    MD.Height = H;
                    
                    createFor006(MD);
                    
                    %%% parsing
                    [Coeffs, State] = datcomParser5();
                    
                    data{i, j, hh, s}.(mass_condition{k}).Coeffs = Coeffs;
                    data{i, j, hh, s}.(mass_condition{k}).State = State;
                    if k == 1
                        data{i, j, hh, s}.c_max = C1;
                        data{i, j, hh, s}.c_min = C2;
                        data{i, j, hh, s}.h = H;
                        data{i, j, hh, s}.shape = shape(s);
                    end
                end
            end
        end
        
    end
    
end

data = reshape(data, [N1*N2*N3*Ns, 1]);
data = data(~cellfun('isempty', data));
delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat')

AMtime = toc;