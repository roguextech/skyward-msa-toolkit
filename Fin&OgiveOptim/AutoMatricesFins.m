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
Mach = datcom.s.Mach;
Alpha = datcom.s.Alpha;
Beta = datcom.s.Beta;
Alt = datcom.s.Alt;

Nn = numel(datcom.design.Lnose);
if Nn ~= 1
    Lnose = datcom.design.Lnose(ceil(Nn/2));
else
    Lnose = datcom.design.Lnose;
end

Chord1 = datcom.design.Chord1;
Chord2 = datcom.design.Chord2;
Heigth = datcom.design.Heigth;
shape = datcom.design.Shape;

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

Nm = length(Mach);
Na = length(Alpha);
Nb = length(Beta);
Nalt = length(Alt);
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
        
        Xle1 = Lcenter + Lnose - d - C1;
        diffC = C1-C2;
        
        for hh = 1:N3
            H = Heigth(hh);
            
            for s = 1:Ns
                if strcmp('rect', shape(s))
                    Xle2 = Lcenter + Lnose - d - C2;
                end
                
                if strcmp('iso', shape(s))
                    Xle2 = Lcenter + Lnose - d - diffC/2;
                end
                
                if strcmp('parall', shape(s))
                    Xle2 = Lcenter + Lnose - d;
                end
                
                %%% Defining Fin Section
                Zup = [zup_raw/C1, zup_raw/C2];
                Lmaxu = [Lmaxu_raw/C1, Lmaxu_raw/C2];
                Lflatu = [(C1 - 2*Lmaxu_raw)/C1, (C2 - 2*Lmaxu_raw)/C2];
                
                for k = 1:2
                    if print
                        clc
                        perc = round((i-1)/N1*100 + (j-1)/N2*(100/N1) + (hh-1)/N3*(100/N2) + (s-1)/Ns*(100/N1/N2/N3) + (k-1)/2*(100/N1/N2/N3/Ns));
                        fprintf('----------------- Fins Aerodynamics Prediction ----------------- \n')
                        fprintf(' Progress %d %% \n', perc);
                    end
                    
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
                    fprintf(fid, '  TNOSE = KARMAN, \r\n');
                    %%%% LNOSE
                    fprintf(fid, '  LNOSE = ');
                    fprintf(fid, '%.3f, \r\n', Lnose);
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
                        [~,~] = system('./datcom for005.dat' );
                    else
                        [~,~] = system('datcom.exe for005.dat' );
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
    'for010.dat', 'for011.dat', 'for012.dat', 'full.mat', 'empty.mat')

AMtime = toc;