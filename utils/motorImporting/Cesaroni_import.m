clear; clc;

linestring = fileread('names.txt');
LinesNameMot = splitlines(compose(linestring));

linestring = fileread('Cesaroni');
TxtLines = splitlines(compose(linestring));


N = length(TxtLines);
Data = {};
Nmot = 0;
for i = 1:N
    
    currentTxt = TxtLines(i);
    if ~cellfun(@isempty, regexp(currentTxt, "\d CTI" )) || ~cellfun(@isempty, regexp(currentTxt, 'Cesaroni_Technology_Inc.'))
        Nmot = Nmot+1;
        DataTxt(Nmot) = currentTxt;
        Start(Nmot) = i+1;
        n = i;
        whileTxt = currentTxt;
        
        while cellfun(@isempty, regexp(whileTxt, ";" ))
            n = n+1;
            whileTxt = TxtLines(n);
            End(Nmot) = n-1;
        end
        
        
    end
end

Data = cell(Nmot, 1);
for i = 1:Nmot
    Nl = End(i) - Start(i);
%     Data{i}.Name = LinesNameMot{i};
    Data{i}.t = zeros(Nl, 1);
    Data{i}.T = zeros(Nl, 1);
    Datasplit = split(DataTxt(i));
    Data{i}.D = str2num(Datasplit{2});
    Data{i}.L = str2num(Datasplit{3});
    Data{i}.mp = str2num(Datasplit{5});
    Data{i}.mm = str2num(Datasplit{6});
    for j = 1:Nl
        pos = Start(i) + j;
        Datamot_j = str2num(TxtLines{pos});
        Data{i}.t(j) = Datamot_j(1);
        Data{i}.T(j) = Datamot_j(2);
    end
    
end

Itot = zeros(Nmot, 1);
for i = 1:Nmot
    if Data{i}.t(1) ~= 0
        Data{i}.t = [0; Data{i}.t];
        Data{i}.T = [0; Data{i}.T];
    end
    Data{i}.Itot = trapz(Data{i}.t, Data{i}.T);
    Itot(i) = Data{i}.Itot;
end

for i = 1:Nmot
    Name = LinesNameMot{i};
    MotorsByName.(Name) = Data{i};
end

[Itot_sort, indexes] = sort(Itot);

for i = 1:Nmot
    Name = strcat('I', num2str(round(Itot_sort(i))));
    MotorsByItot.(Name) = Data{indexes(i)};
    MotorsByItot.(Name).MotorName = LinesNameMot{indexes(i)};
end
    
save('MotorsList.mat', 'MotorsByItot', 'MotorsByName');
