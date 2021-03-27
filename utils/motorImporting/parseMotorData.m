clear; close all; clc;

Aerotech = parseRSE("motorData/Aerotech_7-5-18.rse");

Cesaroni = parseRSE("motorData/cesaroni_11-27-14.rse");

save Motors Aerotech Cesaroni

%% Functions:
function  out = parseRSE(filename)

str = extractFileText(filename);
blocks = regexp(str,'(<engine mfg)*(</engine>)','split');

out = struct();

NBlocks = length(blocks);
for k = 1:NBlocks
    engData = extractBetween(blocks(k),'<eng-data ','/>');
    nt = length(engData);
    out(k).t = zeros(1,nt);
    out(k).T = zeros(1,nt);
    out(k).m = zeros(1,nt);
    for it = 1:nt
        out(k).t(it) = str2double(extractBetween(engData(it),'t="','"'));
        out(k).T(it) = str2double(extractBetween(engData(it),'f="','"'));
        out(k).m(it) = str2double(extractBetween(engData(it),'m="','"'))*1e-3;
    end
    out(k).mp = str2double(extractBetween(blocks(k),'propWt="','"'));
    out(k).mp = out(k).mp.*1e-3; % convert to Kg
    out(k).mm = str2double(extractBetween(blocks(k),'initWt="','"'));
    out(k).mm = out(k).mm.*1e-3; % convert to Kg
    out(k).D = str2double(extractBetween(blocks(k),'dia="','"'));
    out(k).L = str2double(extractBetween(blocks(k),'len="','"'));
    out(k).Itot = str2double(extractBetween(blocks(k),'Itot="','"'));
    out(k).MotorName = extractBetween(blocks(k),'code="','"');
    
    if nt == 0
        out(k) = [];
    end
end

end
