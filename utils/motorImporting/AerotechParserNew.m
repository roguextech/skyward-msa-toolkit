clear
close all
clc
str = extractFileText("Aerotech_7-5-18.rse");
blocks = regexp(str,'(<engine mfg)*(</engine>)','split');

motors = struct();

NBlocks = length(blocks);
for k = 1:NBlocks
    engData = extractBetween(blocks(k),'<eng-data ','/>');
    nt = length(engData);
    motors(k).t = zeros(1,nt);
    motors(k).T = zeros(1,nt);
    for it = 1:nt
        motors(k).t(it) = str2double(extractBetween(engData(it),'t="','"'));
        motors(k).T(it) = str2double(extractBetween(engData(it),'f="','"'));
    end
    motors(k).mp = str2double(extractBetween(blocks(k),'propWt="','"'));
    motors(k).mp = motors(k).mp.*1e-3; % convert to Kg
    motors(k).mm = str2double(extractBetween(blocks(k),'initWt="','"'));
    motors(k).mm = motors(k).mm.*1e-3; % convert to Kg
    motors(k).D = str2double(extractBetween(blocks(k),'dia="','"'));
    motors(k).L = str2double(extractBetween(blocks(k),'len="','"'));
    motors(k).Itot = str2double(extractBetween(blocks(k),'Itot="','"'));
    motors(k).MotorName = extractBetween(blocks(k),'code="','"');
end




