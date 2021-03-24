%{

Auto_Matrices - This script compiles many aerodynamic prediction using MISSILE
DATCOM to optimize rocket ogive.
The output is a cell array composed by structures in which the aerodynamic
coefficients are stored.
In each prediction just the ogive parameters are varying.

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 21/10/20

%}
function [data, AMtime] = AutoMatricesOgive(datcom)

tic

%% recalling the variables
MD.Mach = datcom.s.Mach;
MD.Alpha = datcom.s.Alpha;
MD.Beta = datcom.s.Beta;
MD.Alt = datcom.s.Alt;

xcg = datcom.para.xcg;
MD.D = datcom.para.D;
MD.Lcenter = datcom.para.Lcenter;
MD.Npanel = datcom.para.Npanel;
MD.Phif = datcom.para.Phif;
MD.Ler = datcom.para.Ler;
MD.d = datcom.para.d;
MD.zup_raw = datcom.para.zup_raw;
MD.Lmaxu_raw = datcom.para.Lmaxu_raw;

MD.shape = datcom.design.shape;
MD.Chord1 = datcom.design.Chord1;
MD.Chord2 = datcom.design.Chord2;
Lnose = datcom.design.Lnose;
Pow = datcom.design.NosePower;
OgType = datcom.design.OgType;
MD.Height = datcom.design.Heigth;

No = length(Lnose);
Nt = length(OgType);

%% datcom
data = cell(No, Nt);
mass_condition = {'full', 'empty'};

for i = 1:No
    MD.Lnose = Lnose(i);
    for j = 1:Nt
        
        MD.OgType = OgType(j);
        if j <= length(Pow)
            MD.NosePower = Pow(j);
        end
        
        for k = 1:2
            
            clc
            perc = round((i-1)/No*100 + (j-1)/Nt*(100/No) + (k-1)/2*(100/Nt/No));
            fprintf('----------------- Fins Aerodynamics Prediction ----------------- \n')
            fprintf(' Progress %d %% \n\n', 100);
            fprintf('-----------------       Fins Simulation         ----------------- \n')
            fprintf(' Progress %d %% \n\n', 100);
            fprintf('----------------- Ogive Aerodynamics Prediction ----------------- \n')
            fprintf(' Progress %d %% \n\n', perc);
            
            MD.xcg = xcg(k);
            
            createFor006(MD);
            
            [Coeffs, State] = datcomParser5();
            
            data{i, j}.(mass_condition{k}).Coeffs = Coeffs;
            data{i, j}.(mass_condition{k}).State = State;
            if k == 1
                data{i, j}.Lnose = Lnose(i);
                data{i, j}.Type = OgType(j);
                if j <= length(Pow)
                    data{i, j}.Power = Pow(j);
                end
            end
        end
    end
end
data = reshape(data, [No*Nt, 1]);
data = data(~cellfun('isempty', data));
delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat')

AMtime = toc;