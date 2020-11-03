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
Mach = datcom.s.Mach;
Alpha = datcom.s.Alpha;
Beta = datcom.s.Beta;
Alt = datcom.s.Alt;

xcg = datcom.para.xcg;
D = datcom.para.D;
r = D/2;
S = datcom.para.S;
Lcenter = datcom.para.Lcenter;
Npanel = datcom.para.Npanel;
Phif = datcom.para.Phif;
Ler = datcom.para.Ler;
d = datcom.para.d;
zup_raw = datcom.para.zup_raw;
Lmaxu_raw = datcom.para.Lmaxu_raw;

shape = datcom.design.shape;
C1 = datcom.design.Chord1;
C2 = datcom.design.Chord2;
Lnose = datcom.design.Lnose;
Pow = datcom.design.NosePower;
OgType = datcom.design.OgType;
H = datcom.design.Heigth;


Nm = length(Mach);
Na = length(Alpha);
Nb = length(Beta);
Nalt = length(Alt);
No = length(Lnose);
Nt = length(OgType);

%% datcom
data = cell(No, Nt);
mass_condition = {'full', 'empty'};

diffC = C1-C2;

%%% Defining Fin Section
Zup = [zup_raw/C1, zup_raw/C2];
Lmaxu = [Lmaxu_raw/C1, Lmaxu_raw/C2];
Lflatu = [(C1 - 2*Lmaxu_raw)/C1, (C2 - 2*Lmaxu_raw)/C2];
for i = 1:No
    Xle1 = Lcenter + Lnose(i) - d - C1;
    switch shape
        case 'rect'
            Xle2 = Lcenter + Lnose(i) - d - C2;
            
        case 'iso'
            Xle2 = Lcenter + Lnose(i) - d - diffC/2;
            
        case 'parall'
            Xle2 = Lcenter + Lnose(i) - d;
    end
    for j = 1:Nt
        for k = 1:2
            
            clc
            perc = round((i-1)/No*100 + (j-1)/Nt*(100/No) + (k-1)/2*(100/Nt/No));
            fprintf('----------------- Fins Aerodynamics Prediction ----------------- \n')
            fprintf(' Progress %d %% \n\n', 100);
            fprintf('-----------------       Fins Simulation         ----------------- \n')
            fprintf(' Progress %d %% \n\n', 100);
            fprintf('----------------- Ogive Aerodynamics Prediction ----------------- \n')
            fprintf(' Progress %d %% \n\n', perc);
            
            XCG = xcg(k);
            
            %%% Creating for005.dat file from previous data
            if ismac   % mac procedure
                fid  =  fopen(strcat(pwd, '/for005.dat'),'w+');
            else
                fid  =  fopen(strcat(pwd, '\for005.dat'),'w+');
            end
            
            %%%%%%%%%%%% Flight Conditions
            %%%% Beta
            fprintf(fid, '\n $FLTCON\r\n');
            fprintf(fid, '  BETA = ');
            fprintf(fid, '%.2f,\r\n', Beta(1));
            %%%% Alt
            fprintf(fid, ' ALT = ');
            fprintf(fid, '%d', Nm);
            fprintf(fid, '*');
            fprintf(fid, '%d.,\r\n', Alt(1));
            %%%% Nmach
            fprintf(fid, '  NMACH = ');
            fprintf(fid, '%d., \r\n', Nm);
            %%%% Mach
            fprintf(fid, '  MACH = ');
            if Nm > 11
                for M = 1:11
                    fprintf(fid, '%.2f,',Mach(M));
                end
                fprintf(fid,  ' \r\n MACH(12) = ');
                for M = 12:Nm
                    fprintf(fid, '%.2f',Mach(M));
                    fprintf(fid, ',');
                end
            else
                for M = 1:Nm
                    fprintf(fid, '%.2f,',Mach(M));
                end
            end
            
            fprintf(fid, '\r\n');
            %%%% Nalpha
            fprintf(fid, '  NALPHA = ');
            fprintf(fid, '%d., \r\n', Na);
            %%%% Alpha
            fprintf(fid, '  ALPHA = ');
            if Na > 9
                for a = 1:9
                    fprintf(fid, '%.1f,', Alpha(a));
                end
                fprintf(fid,  ' \r\n ALPHA(10) = ');
                for a = 10:Na
                    fprintf(fid, '%.1f,', Alpha(a));
                end
            else
                for a = 1:Na
                    fprintf(fid, '%.1f,', Alpha(a));
                end
            end
            
            fprintf(fid, '$');
            
            %%%%%%%%%%%% Reference Quantities
            fprintf(fid, '\r\n $REFQ\r\n');
            %%%% XCG
            fprintf(fid, '  XCG = ');
            fprintf(fid, '%.4f,\r\n', XCG);
            %%%% SREF
            fprintf(fid, '  SREF = ');
            fprintf(fid, '%.5f,\r\n', S);
            %%%% LREF
            fprintf(fid, '  LREF = ');
            fprintf(fid, '%.3f,\r\n', D);
            %%%% LATREF
            fprintf(fid, '  LATREF = ');
            fprintf(fid, '%.3f,$', D);
            
            %%%%%%%%%%%% Axisymmetric Body Geometry
            fprintf(fid, '\r\n $AXIBOD\r\n');
            %%%% TNOSE
            fprintf(fid, '  TNOSE = %s, \r\n', OgType(j));
            if j <= length(Pow)
                fprintf(fid, '  POWER = %.3f, \r\n', Pow(j)); 
            end
            %%%% LNOSE
            fprintf(fid, '  LNOSE = ');
            fprintf(fid, '%.3f, \r\n', Lnose(i));
            %%%% DNOSE
            fprintf(fid, '  DNOSE = ');
            fprintf(fid, '%.3f, \r\n', D);
            %%%% LCENTR
            fprintf(fid, '  LCENTR = ');
            fprintf(fid, '%.3f, \r\n', Lcenter);
            %%%% DCENTR
            fprintf(fid, '  DCENTR = ');
            fprintf(fid, '%.3f, \r\n', D);
            %%%% DEXIT
            fprintf(fid, '  DEXIT = 0.,');
            fprintf(fid, '\r\n');
            %%%% BASE
            fprintf(fid, '  BASE = .FALSE.,$');
            
            %%%%%%%%%%%% Finset
            fprintf(fid, '\r\n $FINSET1 \r\n');
            %%%% XLE
            fprintf(fid, '  XLE = ');
            fprintf(fid, '%.3f,', Xle1);
            fprintf(fid, '%.3f, \r\n', Xle2);
            %%%% NPANEL
            fprintf(fid, '  NPANEL = ');
            fprintf(fid, '%.1f,\r\n', Npanel);
            %%%% PHIF
            fprintf(fid, '  PHIF = ');
            for P = 1:length(Phif)
                fprintf(fid, '%.1f', Phif(P));
                fprintf(fid, ',');
            end
            fprintf(fid, '\r\n');
            %%%% LER
            fprintf(fid, '  LER = 2*');
            fprintf(fid, '%.4f,\r\n', Ler);
            %%%% SSPAN
            fprintf(fid, '  SSPAN = ');
            fprintf(fid, '%.3f,', r);
            fprintf(fid, '%.3f,\r\n', r + H);
            %%%% CHORD
            fprintf(fid, '  CHORD = ');
            fprintf(fid, '%.3f,', C1);
            fprintf(fid, '%.3f,\r\n', C2);
            %%%% SECTYP
            fprintf(fid, '  SECTYP = HEX, \r\n');
            %%%% ZUPPER
            fprintf(fid, '  ZUPPER = ');
            fprintf(fid, '%.4f,', Zup(1));
            fprintf(fid, '%.4f,\r\n', Zup(2));
            %%%% LMAXU
            fprintf(fid, '  LMAXU = ');
            fprintf(fid, '%.4f,', Lmaxu(1));
            fprintf(fid, '%.4f,\r\n', Lmaxu(2));
            %%%% LMAXU
            fprintf(fid, '  LFLATU = ');
            fprintf(fid, '%.4f,', Lflatu(1));
            fprintf(fid, '%.4f,$ \r\n', Lflatu(2));
            
            %%%%%%%%%%%% Options
            fprintf(fid, 'DERIV RAD \r\n');
            fprintf(fid, 'DIM M \r\n');
            fprintf(fid, 'DAMP \r\n');
            fprintf(fid, 'SAVE \r\n');
            fprintf(fid, 'NEXT CASE \r\n');
            
            %%%%%%%%%%%% Cases
            for A = 1:Nalt
                for B = 1:Nb
                    if A == 1 && B == 1
                    else
                        fprintf(fid,' $FLTCON \r\n');
                        fprintf(fid,'  BETA = ');
                        fprintf(fid, '%.1f, \r\n', Beta(B));
                        fprintf(fid, ' ALT = ');
                        fprintf(fid, '%d', Nm);
                        fprintf(fid, '*');
                        fprintf(fid, '%d.,$ \r\n', Alt(A));
                        fprintf(fid, 'DERIV RAD\r\n');
                        fprintf(fid, 'DIM M\r\n');
                        fprintf(fid, 'DAMP\r\n');
                        fprintf(fid, 'SAVE\r\n');
                        fprintf(fid, 'NEXT CASE\r\n');
                    end
                end
            end
            fclose(fid);
            
            %%% Datcom and parsing
            if ismac
                [~, ~] = system('./datcom for005.dat' );
            else
                [~, ~] = system('datcom.exe for005.dat' );
            end
            
            value = 0;
            while value == 0
                value = exist('for006.dat','file');
                pause(0.01);
            end

            if k == 1
                mat_name = 'full';
            else
                mat_name = 'empty';
            end
            
            [Coeffs, State] = DatcomParser(mat_name);
            
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
    'for010.dat', 'for011.dat', 'for012.dat', 'full.mat', 'empty.mat')

AMtime = toc;