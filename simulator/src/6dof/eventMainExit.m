function [value, isterminal, direction] = eventMainExit(~, Y, settings)
    para = settings.paraN;
    
    pos_para = [Y(23) Y(24) Y(25)];
    rel_pos = norm(pos_para - ([Y(1) Y(2) Y(3)] + ...
        quatrotate(quatconj([Y(10) Y(11) Y(12) Y(13)]),...
        [(settings.xcg(2)-settings.Lnose) 0 0])));
    
    value = settings.para(para(2)).L - rel_pos;
    
    isterminal = 1;
    direction = 0;
end