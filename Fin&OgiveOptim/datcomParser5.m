function [Coeffs, State] = datcomParser5(varargin)
%{ 

DATCOMPARSER - function to parse the 'for006.dat' file and to generate the
               matrices of the aerodynamic coefficients.

INPUTS:
            - varargin: it can be either a string that contains the name of
                        .mat file that is saved in the Current Folder or it
                        can be an empty field and the .mat file is not
                        saved. (Read NOTES for more informations.)

OUTPUTS: 
            - Coeffs:   struct in which the fields are the matrices 
                        of the aerodynamic coefficients. (e.g. Coeffs.CA)
            - State:    struct in which the fields are the arrays of the
                        aerodynamic states (Alpha, Mach, Beta, Altitude)
                        used to compute the aerodynamic coefficients.

NOTES: If the function is used with the syntax 
'[Coeffs, State] = datcomParser()' the file .mat is not saved in the 
Current Folder. If the function is used with the syntax 
'[Coeffs, State] = datcomParser('name')' a file called 'name.mat' is saved
in the Current Folder. This file contains the two structs Coeffs and
State that can be loaded into the workspace in any future moment using the
command MATLAB function 'load'. Note that the file 'for006.dat' that has to
be parsed must be in the Current Folder.

Author: Giulio Pacifici
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: giulio.pacifici@skywarder.eu
Release date: 14/10/2019

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Update date: 21/10/20

%}

if not(isempty(varargin))
    mat_name = varargin{1};
    savemat = true;
else
    savemat = false;
end

linestring = fileread('for006.dat');

%% blocksplit
pattern = '\*+ \<FLIGHT CONDITIONS AND REFERENCE QUANTITIES \*+';
blocks = regexp(linestring, pattern, 'split');
block1 = blocks(1);
blocks = blocks(2:end);

%% error check
error_check = regexp(block1, '(** ERROR **)', 'tokens');
if not(isempty(error_check{1}))
    error('Attention: Error in the ''for006.dat'' file.');
end

%% length check
pattern = '*** END OF JOB ***';
if not(contains(linestring, pattern))
    error('Datcom didn''t complete the computations, probably the for005 contains too many cases (more than 97)')
end

%% get_coeffs_name
pattern =  ' *\<ALPHA\> *([\w -/]*)';
names = cell(26, 1);
index = 1;

for i = 1:4
    block = blocks{i};
    token = regexp(block, pattern, 'tokens');
    
    % convert cells inside token into strings
    for k = 1:length(token)
        token{k} = char(token{k});
    end
    
    dummy = strjoin(token);
    dummy = split(dummy);
    
    % replaceBadChars
    correct = dummy;
    pattern1 = '[\./-]';
    for j = 1:length(dummy)
        name = dummy{j};
        a = regexprep(name(1:end-1), pattern1, '_');
        b = regexprep(name(end), pattern1, '');
        correct{j} = [a, b];
    end
    
    names(index:index+length(correct)-1) = correct;
    index = index+length(correct);
end
NL = length(names);

%% get the aerodynamic states
pattern1 = ' *\<MACH NO\> *= * (-*[\d]+.[\d]*)';
pattern2 = ' *\<ALTITUDE\> *= * (-*[\d]+.[\d]*)';
pattern3 = ' *\<SIDESLIP\> *= * (-*[\d]+.[\d]*)';

Nb = length(blocks);
NM = min([21, Nb/4]);                                % initial guess
M = zeros(1, NM);
for i = 1:NM
    jj = (i - 1)*4 + 1;
    block = blocks{jj};
    mach = regexp(block, pattern1, 'tokens');
    M(i) = sscanf(char(mach{1}), '%f');
    if i > 1 && M(i) == M(1)
        M = M(1:i-1);
        break;
    end
end
NM = length(M);

NB = min([97, Nb/4/NM]);              % initial guess
B = zeros(1, NB);
for i = 1:NB
    jj = (i - 1)*4*NM + 1;
    block = blocks{jj};
    sslip = regexp(block, pattern3, 'tokens');
    B(i) = sscanf(char(sslip{1}), '%f');
    if i > 1 && B(i) == B(1)
        B = B(1:i-1);
        break;
    end
end
NB = length(B);
    
NA = Nb/4/NM/NB; 
A = zeros(1, NA);
for i = 1:NA
    jj = (i - 1)*4*NM*NB + 1;
    block = blocks{jj};
    alt = regexp(block, pattern2, 'tokens');
    A(i) = sscanf(char(alt{1}), '%f');
end
A = A(1:i);

M = repmat(M, 1, NB*NA);
B = repmat(repelem(B, 1, NM), 1, NA);
A = repelem(A, 1, NM*NB);

%% get alphas
pattern = '^[-\d](?=[\d\.])';

block = blocks{2};
lines = splitlines(block);
index = 0;
new_1 = cell(200, 1);

for j = 1:length(lines)
    line = lines{j};
    line = strtrim(line);
    
    if regexp(line, pattern, 'once')
        index = index + 1;
        new_1{index} =  sscanf(line, '%f');
    end
    
end

alpha = zeros(1, index);

for j = 1:index
    row = new_1{j};
    alpha(j) = row(1);
end
Na = length(alpha);

%% get the aerodynamic data
raw_data = zeros(Na, Nb*NL/4);
ll = 1;
for i = 1:Nb
    block = blocks{i};
    lines = splitlines(block);
    
    j = 1;
    line = [];
    while isempty(line) || not(all(not(isletter(line))))
        line = strtrim(lines{j});
        j = j + 1;
    end
    
    lineScan = sscanf(line, '%f');
    Nls = length(lineScan) - 1;
    raw_data(1, ll:ll+Nls-1) = lineScan(2:end)';
    charMatr = join(lines(j:j+Na-1));
    Matr = sscanf(charMatr{1}, '%f', [Nls+1, Na-1]);
    raw_data(2:end, ll:ll+Nls-1) = Matr(2:end, :)';
    j = j + Na - 1;
    
    if not(all(not(contains(lines(j:end), 'ALPHA'))))
        line = [];
        while isempty(line) ||not(all(not(isletter(line))))
            line = strtrim(lines{j});
            j = j + 1;
        end
        
        lineScan = sscanf(line, '%f');
        Nls2 = length(lineScan) - 1;
        raw_data(1, ll+Nls:ll+Nls+Nls2-1) = lineScan(2:end)';
        
        charMatr = join(lines(j:j+Na-1));
        Matr = sscanf(charMatr{1}, '%f', [Nls2+1, Na-1]);
        raw_data(2:end, ll+Nls:ll+Nls+Nls2-1) = Matr(2:end, :)';
    else
        Nls2 = 0;
    end
    ll = ll + Nls + Nls2;
    
end

%% savemat
realM = [M(1), NaN(1, 200)];
realA = [A(1), NaN(1, 200)];
realB = [B(1), NaN(1, 200)];

iM = 1;
iA = 1;
iB = 1;

for i = 2:length(M)
    if not(any(realM == M(i)))
        iM = iM + 1;
        realM(iM) = M(i);
    end
    if not(any(realA == A(i)))
        iA = iA + 1;
        realA(iA) = A(i);
    end
    if not(any(realB == B(i)))
        iB = iB + 1;
        realB(iB) = B(i);
    end
end

realM = realM(1:iM);
realA = realA(1:iA);
realB = realB(1:iB);

for j = 1:NL
    Coeffs.(names{j}) = zeros(length(alpha), iM, iB, iA);
end

for i = 1:Nb/4
    index = i;
    iA = realA==A(index);
    iB = realB==B(index);
    iM = realM==M(index);
    
    for j = 1:NL
        Coeffs.(names{j})(:, iM, iB, iA) = raw_data(:, length(names)*(i-1)+j);
    end
    
    
end

State.Machs = realM;
State.Alphas = alpha;
State.Betas = realB;
State.Altitudes = realA;

if savemat
    save(mat_name, 'State', 'Coeffs');
end


