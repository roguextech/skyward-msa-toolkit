classdef RocketGUI_V2_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                       matlab.ui.Figure
        HeaderLabel                    matlab.ui.control.Label
        TabGroup                       matlab.ui.container.TabGroup
        GeometryTab                    matlab.ui.container.Tab
        GeneralparametersPanel         matlab.ui.container.Panel
        NumberoffinsEditFieldLabel     matlab.ui.control.Label
        NumberoffinsEditField          matlab.ui.control.NumericEditField
        XCGFullEditFieldLabel          matlab.ui.control.Label
        XCGFullEditField               matlab.ui.control.NumericEditField
        ZCGEditFieldLabel              matlab.ui.control.Label
        ZCGEditField                   matlab.ui.control.NumericEditField
        NoseshapeDropDownLabel         matlab.ui.control.Label
        NoseshapeDropDown              matlab.ui.control.DropDown
        NosePowerEditFieldLabel        matlab.ui.control.Label
        PowerEditField                 matlab.ui.control.NumericEditField
        DiameterEditFieldLabel         matlab.ui.control.Label
        DiameterEditField              matlab.ui.control.NumericEditField
        RocketLengthEditFieldLabel     matlab.ui.control.Label
        RocketLengthEditField          matlab.ui.control.NumericEditField
        NoseLengthEditFieldLabel       matlab.ui.control.Label
        NoseLengthEditField            matlab.ui.control.NumericEditField
        XCGEmptyEditFieldLabel         matlab.ui.control.Label
        XCGEmptyEditField              matlab.ui.control.NumericEditField
        FinsPanel                      matlab.ui.container.Panel
        ShapeDropDownLabel             matlab.ui.control.Label
        FinShapeDropDown               matlab.ui.control.DropDown
        RootChordEditFieldLabel        matlab.ui.control.Label
        MaxChordEditField              matlab.ui.control.NumericEditField
        TipChordEditFieldLabel         matlab.ui.control.Label
        MinChordEditField              matlab.ui.control.NumericEditField
        XLE1EditFieldLabel             matlab.ui.control.Label
        XLE1EditField                  matlab.ui.control.NumericEditField
        XLE2EditFieldLabel             matlab.ui.control.Label
        XLE2EditField                  matlab.ui.control.NumericEditField
        HeightEditFieldLabel           matlab.ui.control.Label
        FinHeightEditField             matlab.ui.control.NumericEditField
        ThicknessEditFieldLabel        matlab.ui.control.Label
        FinThicknessEditField          matlab.ui.control.NumericEditField
        LMaxEditFieldLabel             matlab.ui.control.Label
        LMaxEditField                  matlab.ui.control.NumericEditField
        FlightconditionsTab            matlab.ui.container.Tab
        AddAlphaButton                 matlab.ui.control.Button
        UITableAlpha                   matlab.ui.control.Table
        UITableBeta                    matlab.ui.control.Table
        UITableMach                    matlab.ui.control.Table
        UITableAlt                     matlab.ui.control.Table
        AddBetaButton                  matlab.ui.control.Button
        AddMachButton                  matlab.ui.control.Button
        AddAltitudeButton              matlab.ui.control.Button
        RemoveAlphaButton              matlab.ui.control.Button
        RemoveBetaButton               matlab.ui.control.Button
        RemoveMachButton               matlab.ui.control.Button
        RemoveAltitudeButton           matlab.ui.control.Button
        LocalaltitudemEditFieldLabel   matlab.ui.control.Label
        ZLocEditField                  matlab.ui.control.NumericEditField
        ExecDATCOMParseMatricesButton  matlab.ui.control.Button
        Panel                          matlab.ui.container.Panel
        RocketUIAxes                   matlab.ui.control.UIAxes
        FinsUIAxes                     matlab.ui.control.UIAxes
    end

    
    properties (Access = public)
        D
        NFins
        RocketLength
        NoseLength
        BodyLength
        FinMaxChord 
        FinMinChord 
        FinHeight
        BottomDist 
        FinShape
        XLe
        XCGE
        XCGF
        ZCG
        NShape 
        NPower
        
        % Fin cross section
        FinT 
        LDiagSection 
        LmaxMaxChord 
        LmaxMinChord 
        LflatMinChord
        LflatMaxChord
        
        % GUI object properties
        selectedAlpha % Defines which alpha is selected in the table
        selectedBeta
        selectedMach
        selectedAlt
    end
   
    
    methods (Access = private)
        
        function updatePlot(app)
            % Nose
            xn = linspace(0,app.NoseLength);
            switch app.NShape
                case 'KARMAN'
                    theta = acos(1-2.*xn./app.NoseLength);
                    yn = app.D/2./sqrt(pi).*sqrt(theta-sin(2*theta)/2);
                case 'CONICAL'
                    yn = xn.*app.D/2/app.NoseLength;
                case 'OGIVE'
                    rho = ((app.D/2)^2+(app.NoseLength)^2)/app.D;
                    yn = sqrt(rho^2-(app.NoseLength-xn).^2)+app.D/2-rho;
                case 'POWER'
                    yn = app.D/2.*(xn/app.NoseLength).^app.NPower;
            end
        
            % Body coordinates
            xb = [xn app.RocketLength app.RocketLength];
            yb = [yn app.D/2 0];
            % Fins coordinates
            xf = [app.XLe(1)  app.XLe(1)+app.FinMaxChord app.XLe(2)+app.FinMinChord ...
                app.XLe(2) app.XLe(1)];
            yf = [app.D/2 app.D/2 app.D/2+app.FinHeight app.D/2+app.FinHeight app.D/2];
            h1 = plot(app.RocketUIAxes,xb,yb,'-b');
            hold(app.RocketUIAxes,'on');
            plot(app.RocketUIAxes,xb,-yb,'-b');
            plot(app.RocketUIAxes,xf,yf,'-b');
            plot(app.RocketUIAxes,xf,-yf,'-b');
            h2 = plot(app.RocketUIAxes,app.XCGF,app.ZCG,'or');
            plot(app.RocketUIAxes,app.XCGF,app.ZCG,'+r');
            h3 = plot(app.RocketUIAxes,app.XCGE,app.ZCG,'ok');
            plot(app.RocketUIAxes,app.XCGE,app.ZCG,'+k');
            
            app.RocketUIAxes.YLim = [-app.D*2-3 app.D*2+3];
            app.RocketUIAxes.XLim = [-5 (5+app.RocketLength)];
            xlabel(app.RocketUIAxes, 'X [cm]')
            ylabel(app.RocketUIAxes, 'Z [cm]')
            legend([h1,h2,h3],'Rocket','CG Full','CG Empty');
            axis(app.RocketUIAxes,'equal');
            grid(app.RocketUIAxes,'on');
            title(app.RocketUIAxes,'Rocket Geometry')
            hold(app.RocketUIAxes,'off');
            
            
            % Draw fins
            xf = [-app.FinMaxChord/2 , -app.FinMaxChord/2+app.LmaxMaxChord , ...
                app.FinMaxChord/2-app.LmaxMaxChord , app.FinMaxChord/2];
            yf = [0 app.FinT/2 app.FinT/2 0];
            h1 = plot(app.FinsUIAxes,xf,yf,'-b');
            hold(app.FinsUIAxes,'on');
            plot(app.FinsUIAxes,xf,-yf,'b');
            
            xf = [-app.FinMinChord/2 , -app.FinMinChord/2+app.LmaxMinChord , ...
                app.FinMinChord/2-app.LmaxMinChord , app.FinMinChord/2];
            yf = [0 app.FinT/2 app.FinT/2 0];
            h2 = plot(app.FinsUIAxes,xf,yf,'-r');
            plot(app.FinsUIAxes,xf,-yf,'r');
            app.FinsUIAxes.XLim = [-app.FinMaxChord/2-0.5 app.FinMaxChord/2+0.5];
            %         app.FinsUIAxes.YLim = [-app.FinT-0.1 app.FinT+0.1];
            xlabel(app.FinsUIAxes, 'X [cm]')
            ylabel(app.FinsUIAxes, 'Y [cm]')
            grid(app.FinsUIAxes,'minor');
            axis(app.FinsUIAxes,'equal');
            legend([h1,h2],'Cross section at fin root','Cross section at fin tip');
            title(app.FinsUIAxes,'Fin Cross Section Geometry')
            hold(app.FinsUIAxes,'off');
        end
        
        
        function Geometry = createGeometryStrcut(app,xcgtype)
            Geometry.Chord1 = app.FinMaxChord;
            Geometry.Chord2 = app.FinMinChord;
            Geometry.Height = app.FinHeight;
            Geometry.shape = datcom.shape;
            Geometry.D = app.D;
            Geometry.Lnose = app.NoseLength;
            Geometry.Lcenter = app.RocketLength-app.NoseLength;
            Geometry.Npanel = app.NFins;
            Geometry.OgType = app.NShape;
            if strcp(xcgtype,'full')
                Geometry.xcg = app.XCGF;
            else
                Geometry.xcg = app.XCGE;
            end
        end
        
        function str = createFor005(app,xcgtype)
            % Print DATCOM output

            %%%% FLIGHT CONDITIONS
            Nalpha = length(app.UITableAlpha.Data);
            Nbeta = length(app.UITableBeta.Data);
            Nmach = length(app.UITableMach.Data);
            Nalt = length(app.UITableAlt.Data);
            Alpha = app.UITableAlpha.Data;
            Beta = app.UITableBeta.Data;
            Mach = app.UITableMach.Data;
            Alt = app.UITableAlt.Data + app.ZLocEditField.Value;
            
            str = sprintf('$FLTCON\n');
            %%%% Beta
            str = sprintf('%s  BETA = ',str);
            str = sprintf('%s%.2f,\n', str,Beta(1));
            %%%% Alt
            str = sprintf('%s  ALT = ',str);
            str = sprintf('%s%d', str,Nmach);
            str = sprintf('%s*',str);
            str = sprintf('%s%d.,\n',str, Alt(1));
            %%%% Nmach
            str = sprintf('%s  NMACH = ',str);
            str = sprintf('%s%d., \n',str, Nmach);
            %%%% Mach
            str=sprintf('%s  MACH = ',str);
            if Nmach>11
                for M = 1:11
                    str = sprintf('%s%.2f',str,Mach(M));
                    str= sprintf('%s,',str);
                end
                str=sprintf('%s\n  MACH(12) = %.2f,',str,Mach(12));
                for M = 13:Nmach
                    str=sprintf('%s%.2f',str,Mach(M));
                    str=sprintf('%s,',str);
                end
            else
                for M = 1:Nmach
                    str = sprintf('%s%.2f',str,Mach(M));
                    str= sprintf('%s,',str);
                end
            end
            str=sprintf('%s\n',str);
            %%% Nalpha
            str = sprintf('%s  NALPHA = ',str);
            str = sprintf('%s%d., \n',str, Nalpha);
            %%% Alpha
            str = sprintf('%s  ALPHA = ',str);
            for a = 1:min(Nalpha,9)
                str=sprintf('%s%.1f,', str,Alpha(a));
            end
            if Nalpha >=10
                str=sprintf('%s \n  ALPHA(10) = ',str);
                for a =     10:Nalpha
                    str=sprintf('%s%.1f,', str,Alpha(a));
                end 
            end
            str=sprintf('%s$\n',str);
            
            %%%%%% GEOMETRY
            str = sprintf("%s$REFQ\n",str);
            if strcmp(xcgtype,'full')
                str = sprintf('%s  XCG=%4.3f,\n',str,app.XCGF/100);
            elseif strcmp(xcgtype,'empty')
                str = sprintf('%s  XCG=%4.3f,\n',str,app.XCGE/100);
            end
            str = sprintf('%s  SREF=%6.5f,\n',str,pi*(app.D/100)^2/4);
            str = sprintf('%s  LREF=%4.3f,\n',str,app.D/100);
            str = sprintf('%s  LATREF=%4.3f,$\n',str,app.D/100);
            str = sprintf('%s$AXIBOD\n',str);
            str = sprintf('%s  TNOSE=%s,\n',str,app.NShape);
            if strcmp(app.NShape,'POWER')
                str = sprintf('%s POWER=%4.3f,\n',str,app.NPower);
            end
            str = sprintf('%s  LNOSE=%4.3f,\n',str,app.NoseLength/100);
            str = sprintf('%s  DNOSE=%4.3f,\n',str,app.D/100);
            str = sprintf('%s  LCENTR=%4.3f,\n',str,(app.RocketLength-app.NoseLength)/100);
            str = sprintf('%s  DCENTR=%4.3f,\n',str,app.D/100);
            str = sprintf('%s  DEXIT=0.,\n',str);
            str = sprintf('%s  BASE=.FALSE.,$\n',str);

            str = sprintf('%s$FINSET1\n',str);
            if app.NFins == 4 || app.NFins == 3
                str = sprintf('%s  NPANEL=%d.0,\n',str,app.NFins);
                angles = (0:1:app.NFins-1).*360/app.NFins;
                strphi = sprintf('PHIF=');
                for k=1:app.NFins
                    strphi = sprintf('%s%d.0,',strphi,angles(k));
                end 
                strphi=strcat(strphi,'\n');
            else
                % Do nothing
            end
            str = sprintf('%s  %s',str,strphi);
            str = sprintf('%s  XLE=%4.3f,%4.3f,\n',str,app.XLe(1)/100,app.XLe(2)/100);
            str = sprintf('%s  LER=2*0.003,\n',str);
%             str = sprintf('%s STA=0.0,\n',str);
            str = sprintf('%s  SSPAN=%4.3f,%4.3f,\n',str,app.D/2/100,(app.D/2+app.FinHeight)/100);
            str = sprintf('%s  CHORD=%4.3f,%4.3f,\n',str,app.FinMaxChord/100,app.FinMinChord/100);
            str = sprintf('%s  SECTYP=HEX,\n',str);
            str = sprintf('%s  ZUPPER=%4.4f,%4.4f,\n',str,app.FinT/2/app.FinMaxChord,app.FinT/2/app.FinMinChord);
            str = sprintf('%s  LMAXU=%4.4f,%4.4f,\n',str,app.LmaxMaxChord/app.FinMaxChord,...
                app.LmaxMinChord/app.FinMinChord);
            str = sprintf('%s  LFLATU=%.4f,%.4f,$\n',str,app.LflatMaxChord/app.FinMaxChord,...
                app.LflatMinChord/app.FinMinChord);
            
            %%%%%%%%%%%% Options
            str=sprintf('%sDERIV RAD\n',str);
            str=sprintf('%sDIM M\n',str);
            str=sprintf('%sDAMP \n',str);
            str=sprintf('%sSAVE \n',str);
            str=sprintf('%sNEXT CASE \n',str);
            
            %%%%%%%%%%%% Cases
            for A = 1:Nalt
                for B = 1:Nbeta
                    if A == 1 && B == 1
                    else
                        str=sprintf('%s $FLTCON \n',str);
                        str=sprintf('%s  BETA = ',str);
                        str=sprintf('%s%.1f, \n', str,Beta(B));
                        str=sprintf('%s  ALT = ',str);
                        str=sprintf('%s%d', str, Nmach);
                        str=sprintf('%s*',str);
                        str=sprintf('%s%d.,$ \n',str, Alt(A));
                        str=sprintf('%sDERIV RAD\n',str);
                        str=sprintf('%sDIM M\n',str);
                        str=sprintf('%sDAMP\n',str);
                        str=sprintf('%sSAVE\n',str);
                        str=sprintf('%sNEXT CASE\n',str);
                    end
                end
            end
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.D = 15;
            app.NFins = 3;
            app.RocketLength = 268;
            app.NoseLength = 28;
            app.BodyLength = app.RocketLength - app.NoseLength;
            app.FinMaxChord = 26;
            app.FinMinChord = 13;
            app.FinHeight = 15;
            app.BottomDist = 0.0;
%             app.XLe = app.RocketLength - app.BottomDist - [app.FinMaxChord ; ...
%                 app.FinMaxChord-(app.FinMaxChord-app.FinMinChord)/2];
            app.XLe = [app.RocketLength-app.FinMaxChord app.RocketLength];
            app.XCGF = 163;
            app.XCGE = 150;
            app.ZCG = 0;
            app.NShape = 'KARMAN';
            app.NPower = 0.5;
            
            % Fin cross section
            app.FinShape = 'TRAPEZOID';
            app.FinT = 0.3;
            app.LDiagSection = 0.15; % Horizontal length of diagonal part of the section
            app.LmaxMaxChord = app.LDiagSection;
            app.LmaxMinChord = app.LDiagSection;
            app.LflatMinChord = app.FinMinChord-2*app.LDiagSection;
            app.LflatMaxChord = app.FinMaxChord-2*app.LDiagSection;
            
            % Setup Strings
            app.RocketLengthEditField.Value = app.RocketLength;
            app.DiameterEditField.Value = app.D;
            app.NumberoffinsEditField.Value = app.NFins;
            app.NoseLengthEditField.Value = app.NoseLength;
            app.XCGFullEditField.Value = app.XCGF;
            app.XCGEmptyEditField.Value = app.XCGE;
            app.ZCGEditField.Value = app.ZCG;
            app.PowerEditField.Value = app.NPower;
            app.NoseshapeDropDown.Value = app.NShape;
            app.FinShapeDropDown.Value = app.FinShape;
            app.XLE1EditField.Value = app.XLe(1);
            app.XLE2EditField.Value = app.XLe(2);
            app.MaxChordEditField.Value = app.FinMaxChord;
            app.MinChordEditField.Value = app.FinMinChord;
            app.FinHeightEditField.Value = app.FinHeight;
            app.FinThicknessEditField.Value = app.FinT;
            app.LMaxEditField.Value = app.LmaxMaxChord;
            
            updatePlot(app);
            
             % Initialise UITableAlpha
            app.UITableAlpha.Data = [-25 -15 -10 -7.5 -5.0 -2 -1 -0.5 -0.1 ...
                0.1 0.5 1 2 5 7.5 10 15 25]';
            app.UITableBeta.Data = [-0.5 -0.1 0.1 0.5]';
            app.UITableMach.Data = [0.05 0.10 0.15 0.20 0.25 0.30 0.35 ...
                0.40 0.45 0.50 0.55 0.60 0.65 0.70 0.75 0.80 0.85 0.90 ...
                0.95 1.00]';
            app.UITableAlt.Data = [0 150 300 600 900 1200 1500 2000 2500 3000 3500]';
            
            % Local altitude 
            app.ZLocEditField.Value = 109;
            
            app.PowerEditField.Enable = false;
        end

        % Value changed function: NoseshapeDropDown
        function NoseshapeDropDownValueChanged(app, event)
            value = app.NoseshapeDropDown.Value;
            if ~strcmp(value,'POWER')
                app.PowerEditField.Enable = false;
            else
                app.PowerEditField.Enable = true;
                app.NPower = app.PowerEditField.Value;
            end
            app.NShape = value;
            updatePlot(app);
        end

        % Value changed function: FinShapeDropDown
        function FinShapeDropDownValueChanged(app, event)
            value = app.FinShapeDropDown.Value;
            if strcmp(value,'RECTANGULAR')
                % Update rocket values (only 1 xle and 1 chord)
                app.FinMinChord = app.FinMaxChord;
                app.XLe(2) = app.XLe(1);
                % Update textbox and disable the other
                app.MaxChordEditField.Value = app.FinMaxChord;
                app.MinChordEditField.Value = app.FinMaxChord;
                app.MinChordEditField.Enable = false;                  
                app.XLE2EditField.Value = app.XLE1EditField.Value;
                app.XLE2EditField.Enable = false;
            else    
                app.MinChordEditField.Enable = true;
                app.XLE2EditField.Enable = true;
            end
            app.FinShape = value;
            updatePlot(app);
        end

        % Value changed function: DiameterEditField, 
        % FinHeightEditField, FinThicknessEditField, LMaxEditField, 
        % MaxChordEditField, MinChordEditField, 
        % NoseLengthEditField, NumberoffinsEditField, 
        % PowerEditField, RocketLengthEditField, XCGEmptyEditField, 
        % XCGFullEditField, XLE1EditField, XLE2EditField, 
        % ZCGEditField
        function TextBoxEdit(app, event)
            src = event.Source;
            value = src.Value;
            % Determine which textbox was activated
            switch src.Tag
                case 'RocketLength'
                    app.RocketLength = value;
                case 'NoseLength'
                    if value > app.RocketLength
                        msgbox('Invalid value of nose length. Cannot be > than rocket length','Warning');
                        % Reassign previous value
                        src.Value = app.NoseLength;
                    else % update value
                        app.NoseLength = value;
                    end
                case 'MaxChord'
                    if value > app.RocketLength
                        msgbox('Invalid value of chord. Cannot be bigger than rocket length','Warning');
                        % Reassign previous value
                        src.Value = app.FinMaxChord;
                    elseif value < app.FinMinChord & ~strcmp(app.FinShape,'RECTANGULAR')
                        msgbox('Invalid value of chord. Cannot be smaller than the tip chord','Warning');
                        src.Value = app.FinMaxChord;
                    elseif value < app.FinMinChord & strcmp(app.FinShape,'RECTANGULAR')
                        app.FinMaxChord = value;
                        app.FinMinChord = value;
                        app.MaxChordEditField.Value = value;
                        app.MinChordEditField.Value = value;
                    else
                        app.FinMaxChord = value;
                        app.LflatMaxChord = app.FinMaxChord - 2*app.LmaxMaxChord;
                    end
                case 'MinChord'
                    if value > app.RocketLength
                        msgbox('Invalid value of chord. Cannot be > than rocket length','Warning');
                        % Reassign previous value
                        src.Value = app.FinMinChord;
                    elseif value > app.FinMaxChord & ~strcmp(app.FinShape,'RECTANGULAR')
                        msgbox('Invalid value of chord. Cannot be > than the root chord','Warning');
                        % Reassign previous value
                        src.Value = app.FinMinChord;
                    elseif value > app.FinMaxChord & strcmp(app.FinShape,'RECTANGULAR')
                        app.FinMaxChord = value;
                        app.FinMinChord = value;
                        app.MaxChordEditField.Value = value;
                        app.MinChordEditField.Value = value;
                    else
                        app.FinMinChord = value;
                        app.LflatMinChord = app.FinMinChord - 2*app.LmaxMinChord;
                    end
                case 'XLE1'
                    if value > app.RocketLength(1)
                        msgbox('Invalid value of XLE(1). Cannot be bigger than rocket length','Warning');
                        src.Value = app.XLe(1);
                    else 
                        if strcmp(app.FinShape,'TRAPEZOID')
                            app.XLe(1) = value;
                        else % rectangular fins
                            app.XLe(1) = value;
                            app.XLe(2) = value; 
                            app.XLE1EditField.Value = value;
                            app.XLE2EditField.Value = value;
                        end
                     end
                case 'XLE2'
                    % check if value is ok
                    if value > app.RocketLength(1)
                        msgbox('Invalid value of XLE(2). Cannot be bigger than rocket length','Warning');
                        src.Value = app.XLe(2);
                    else 
                        if strcmp(app.FinShape,'TRAPEZOID')
                            app.XLe(2) = value;
                        else % rectangular fins
                            app.XLe(1) = value;
                            app.XLe(2) = value; 
                            app.XLE1EditField.Value = value;
                            app.XLE2EditField.Value = value;
                        end
                    end
                case 'FinHeight'
                    app.FinHeight = value;
                case 'XCGF'
                    if value > app.RocketLength || value < 0
                        msgbox('Invalid value of XCG. Cannot be > than rocket length or < 0','Warning');
                        src.Value = app.XCGF;
                    else
                        app.XCGF = value;
                    end
                case 'XCGE'
                    if value > app.RocketLength || value < 0
                        msgbox('Invalid value of XCG. Cannot be > than rocket length or < 0','Warning');
                        src.Value = app.XCGE;
                    else
                        app.XCGE = value;
                    end
                case 'ZCG'
                    if abs(value) >= app.D/2
                        msgbox('Invalid value of XCG. Cannot be >= than rocket radius','Warning');
                        src.Value = app.ZCG;
                    else
                        app.ZCG = value;
                    end
                case 'NosePower'
                    if value > 1
                        msgbox('Invalid value of nose power. Cannot be > 1','Warning');
                        src.Value = app.NPower;
                    else
                        app.NPower = value;
                    end
                case 'FinThickness'
                    if value > app.D/2
                        msgbox('Invalid value of fin thickness. Cannot be bigger than rocket radius','Warning');
                        src.Value = app.FinT;
                    else
                        app.FinT = value;
                    end  
                case 'LMax'
                    app.LDiagSection  = value;
                    app.LmaxMaxChord = value;
                    app.LmaxMinChord = value;
                    app.LflatMinChord = app.FinMinChord-2*app.LmaxMinChord;
                    app.LflatMaxChord = app.FinMaxChord-2*app.LmaxMaxChord;
                case 'NFins'
                    if value <= 1
                        msgbox('Invalid numer of fins. Use at least 2 fins','Warning');
                        src.Value = app.NFins;
                    else
                        app.NFins = value;
                    end
                case 'Diameter'
                    if value < 0
                        msgbox('Invalid diameter. Cannot be < 0','Warning');
                        src.Value = app.D;
                    else
                        app.D = value;
                    end
                otherwise
            end
            updatePlot(app);
        end

        % Cell edit callback: UITableAlpha
        function UITableAlphaCellEdit(app, event)
            indices = event.Indices;
            newData = event.NewData;
            % Sort in ascending order 
            sortedValues = sort(app.UITableAlpha.Data);
            app.UITableAlpha.Data = sortedValues;
        end

        % Selection change function: TabGroup
        function TabGroupSelectionChanged(app, event)
            selectedTab = app.TabGroup.SelectedTab;
            if strcmp(selectedTab.Title,'Flight conditions')
                app.HeaderLabel.Visible = false;
            else
                app.HeaderLabel.Visible = true;
            end
        end

        % Button pushed function: AddAlphaButton, 
        % AddAltitudeButton, AddBetaButton, AddMachButton
        function AddFLTCOND(app, event)
            buttonPressed = event.Source.Text;
            switch buttonPressed
                case 'Add Alpha'
                    app.UITableAlpha.Data = [app.UITableAlpha.Data; 0];
                    app.selectedAlpha = length(app.UITableAlpha.Data);
                case 'Add Beta'
                    app.UITableBeta.Data = [app.UITableBeta.Data; 0];
                    app.selectedBeta = length(app.UITableBeta.Data);
                case 'Add Mach'
                    app.UITableMach.Data = [app.UITableMach.Data; 0];
                    app.selectedMach = length(app.UITableMach.Data);
                case 'Add Altitude'
                    app.UITableAlt.Data = [app.UITableAlt.Data; 0];
                    app.selectedAlt = length(app.UITableAlt.Data);
                otherwise
                    % do nothing
            end
        end

        % Button pushed function: RemoveAlphaButton, 
        % RemoveAltitudeButton, RemoveBetaButton, RemoveMachButton
        function RemoveFLTCOND(app, event)
            buttonPressed = event.Source.Text;
            switch buttonPressed
                case 'Remove Alpha'
                    if app.selectedAlpha>0
                        app.UITableAlpha.Data(app.selectedAlpha) = [];
                    end
                case 'Remove Beta'
                    if app.selectedBeta >0
                        app.UITableBeta.Data(app.selectedBeta) = [];
                    end
                case 'Remove Mach'
                    if app.selectedMach > 0
                        app.UITableMach.Data(app.selectedMach) = [];
                    end
                case 'Remove Altitude'
                    if app.selectedAlt > 0
                        app.UITableAlt.Data(app.selectedAlt) = [];
                    end
                otherwise
                    % do nothing
            end
            % Deselect all cells
            app.selectedAlpha = 0;
            app.selectedBeta = 0;
            app.selectedMach = 0;
            app.selectedAlt = 0;
        end

        % Cell edit callback: UITableBeta
        function UITableBetaCellEdit(app, event)
            indices = event.Indices;
            newData = event.NewData;
            % Sort in ascending order 
            sortedValues = sort(app.UITableBeta.Data);
            app.UITableBeta.Data = sortedValues;
        end

        % Cell edit callback: UITableMach
        function UITableMachCellEdit(app, event)
            indices = event.Indices;
            newData = event.NewData;
            % Sort in ascending order 
            sortedValues = sort(app.UITableMach.Data);
            app.UITableMach.Data = sortedValues;
        end

        % Cell edit callback: UITableAlt
        function UITableAltCellEdit(app, event)
            indices = event.Indices;
            newData = event.NewData;
            % Sort in ascending order 
            sortedValues = sort(app.UITableAlt.Data);
            app.UITableAlt.Data = sortedValues;
        end

        % Button pushed function: ExecDATCOMParseMatricesButton
        function PrintDATCOMtextButtonPushed(app, event)

            str = createFor005(app,'full');
                 
            % Check if DATCOM file exists
            if isfile('datcom.exe') || isfile('..\..\AutoMatricesProtub\datcom.exe')
                %%% FULL CONFIGURATION
                str = createFor005(app,'full');
                fid = fopen('for005.dat','w');
                fprintf(fid,str);
                fclose(fid);
    
                % Run DATCOM and create for006
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
                [CoeffsF, State] = datcomParser5('full');
                
                %%% EMPTY CONFIGURATION
                str = createFor005(app,'empty');
                fid = fopen('for005.dat','w');
                fprintf(fid,str);
                fclose(fid);
    
                % Run DATCOM and create for006
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
                [CoeffsF, State] = datcomParser5('empty');
    
            else
                % File does not exist.
                error('datcom.exe does not exists. Copy it in the current folder')
            end
                
        end

        % Cell selection callback: UITableAlpha
        function UITableAlphaCellSelection(app, event)
            indices = event.Indices;
            app.selectedAlpha=indices(1); % only the row
        end

        % Cell selection callback: UITableBeta
        function UITableBetaCellSelection(app, event)
            indices = event.Indices;
            app.selectedBeta=indices(1); % only the row
        end

        % Button down function: UIFigure
        function UIFigureButtonDown(app, event)
            % Deselect cells table
            app.selectedAlpha = 0;
            app.selectedBeta = 0;
            app.selectedMach = 0;
            app.selectedAlt = 0;
        end

        % Cell selection callback: UITableMach
        function UITableMachCellSelection(app, event)
            indices = event.Indices;
            app.selectedMach=indices(1); % only the row
        end

        % Cell selection callback: UITableAlt
        function UITableAltCellSelection(app, event)
            indices = event.Indices;
            app.selectedAlt=indices(1);
        end

        % Callback function
        function ZLocEditFieldValueChanged(app, event)

        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1043 698];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.ButtonDownFcn = createCallbackFcn(app, @UIFigureButtonDown, true);

            % Create HeaderLabel
            app.HeaderLabel = uilabel(app.UIFigure);
            app.HeaderLabel.FontSize = 16;
            app.HeaderLabel.FontWeight = 'bold';
            app.HeaderLabel.Position = [676 665 162 22];
            app.HeaderLabel.Text = 'All measures in [cm]';

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.SelectionChangedFcn = createCallbackFcn(app, @TabGroupSelectionChanged, true);
            app.TabGroup.Position = [507 77 516 570];

            % Create GeometryTab
            app.GeometryTab = uitab(app.TabGroup);
            app.GeometryTab.Title = 'Geometry';

            % Create GeneralparametersPanel
            app.GeneralparametersPanel = uipanel(app.GeometryTab);
            app.GeneralparametersPanel.Title = 'General parameters';
            app.GeneralparametersPanel.Position = [19 337 413 189];

            % Create NumberoffinsEditFieldLabel
            app.NumberoffinsEditFieldLabel = uilabel(app.GeneralparametersPanel);
            app.NumberoffinsEditFieldLabel.HorizontalAlignment = 'right';
            app.NumberoffinsEditFieldLabel.Position = [213 108 84 22];
            app.NumberoffinsEditFieldLabel.Text = 'Number of fins';

            % Create NumberoffinsEditField
            app.NumberoffinsEditField = uieditfield(app.GeneralparametersPanel, 'numeric');
            app.NumberoffinsEditField.Limits = [0 Inf];
            app.NumberoffinsEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.NumberoffinsEditField.Tag = 'NFins';
            app.NumberoffinsEditField.Position = [300 108 100 22];
            app.NumberoffinsEditField.Value = 4;

            % Create XCGFullEditFieldLabel
            app.XCGFullEditFieldLabel = uilabel(app.GeneralparametersPanel);
            app.XCGFullEditFieldLabel.Tag = 'XCGF';
            app.XCGFullEditFieldLabel.HorizontalAlignment = 'right';
            app.XCGFullEditFieldLabel.Position = [31 46 54 22];
            app.XCGFullEditFieldLabel.Text = 'XCG Full';

            % Create XCGFullEditField
            app.XCGFullEditField = uieditfield(app.GeneralparametersPanel, 'numeric');
            app.XCGFullEditField.Limits = [0 Inf];
            app.XCGFullEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.XCGFullEditField.Tag = 'XCG';
            app.XCGFullEditField.Position = [91 46 100 22];
            app.XCGFullEditField.Value = 117;

            % Create ZCGEditFieldLabel
            app.ZCGEditFieldLabel = uilabel(app.GeneralparametersPanel);
            app.ZCGEditFieldLabel.HorizontalAlignment = 'right';
            app.ZCGEditFieldLabel.Position = [50 14 31 22];
            app.ZCGEditFieldLabel.Text = 'ZCG';

            % Create ZCGEditField
            app.ZCGEditField = uieditfield(app.GeneralparametersPanel, 'numeric');
            app.ZCGEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.ZCGEditField.Tag = 'ZCG';
            app.ZCGEditField.Position = [91 14 100 22];

            % Create NoseshapeDropDownLabel
            app.NoseshapeDropDownLabel = uilabel(app.GeneralparametersPanel);
            app.NoseshapeDropDownLabel.HorizontalAlignment = 'right';
            app.NoseshapeDropDownLabel.Position = [5 77 70 22];
            app.NoseshapeDropDownLabel.Text = 'Nose shape';

            % Create NoseshapeDropDown
            app.NoseshapeDropDown = uidropdown(app.GeneralparametersPanel);
            app.NoseshapeDropDown.Items = {'KARMAN', 'POWER', 'OGIVE', 'CONICAL'};
            app.NoseshapeDropDown.ValueChangedFcn = createCallbackFcn(app, @NoseshapeDropDownValueChanged, true);
            app.NoseshapeDropDown.Position = [92 77 100 22];
            app.NoseshapeDropDown.Value = 'KARMAN';

            % Create NosePowerEditFieldLabel
            app.NosePowerEditFieldLabel = uilabel(app.GeneralparametersPanel);
            app.NosePowerEditFieldLabel.HorizontalAlignment = 'right';
            app.NosePowerEditFieldLabel.Position = [224 77 71 22];
            app.NosePowerEditFieldLabel.Text = 'Nose Power';

            % Create PowerEditField
            app.PowerEditField = uieditfield(app.GeneralparametersPanel, 'numeric');
            app.PowerEditField.Limits = [0 Inf];
            app.PowerEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.PowerEditField.Tag = 'NosePower';
            app.PowerEditField.Position = [301 77 100 22];
            app.PowerEditField.Value = 0.5;

            % Create DiameterEditFieldLabel
            app.DiameterEditFieldLabel = uilabel(app.GeneralparametersPanel);
            app.DiameterEditFieldLabel.HorizontalAlignment = 'right';
            app.DiameterEditFieldLabel.Position = [240 139 54 22];
            app.DiameterEditFieldLabel.Text = 'Diameter';

            % Create DiameterEditField
            app.DiameterEditField = uieditfield(app.GeneralparametersPanel, 'numeric');
            app.DiameterEditField.Limits = [0 Inf];
            app.DiameterEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.DiameterEditField.Tag = 'Diameter';
            app.DiameterEditField.Position = [300 139 100 22];
            app.DiameterEditField.Value = 9;

            % Create RocketLengthEditFieldLabel
            app.RocketLengthEditFieldLabel = uilabel(app.GeneralparametersPanel);
            app.RocketLengthEditFieldLabel.HorizontalAlignment = 'right';
            app.RocketLengthEditFieldLabel.Position = [4 139 83 22];
            app.RocketLengthEditFieldLabel.Text = 'Rocket Length';

            % Create RocketLengthEditField
            app.RocketLengthEditField = uieditfield(app.GeneralparametersPanel, 'numeric');
            app.RocketLengthEditField.Limits = [0 Inf];
            app.RocketLengthEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.RocketLengthEditField.Tag = 'RocketLength';
            app.RocketLengthEditField.Position = [91 139 100 22];

            % Create NoseLengthEditFieldLabel
            app.NoseLengthEditFieldLabel = uilabel(app.GeneralparametersPanel);
            app.NoseLengthEditFieldLabel.HorizontalAlignment = 'right';
            app.NoseLengthEditFieldLabel.Position = [15 108 74 22];
            app.NoseLengthEditFieldLabel.Text = 'Nose Length';

            % Create NoseLengthEditField
            app.NoseLengthEditField = uieditfield(app.GeneralparametersPanel, 'numeric');
            app.NoseLengthEditField.Limits = [0 Inf];
            app.NoseLengthEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.NoseLengthEditField.Tag = 'NoseLength';
            app.NoseLengthEditField.Position = [93 108 100 22];

            % Create XCGEmptyEditFieldLabel
            app.XCGEmptyEditFieldLabel = uilabel(app.GeneralparametersPanel);
            app.XCGEmptyEditFieldLabel.HorizontalAlignment = 'right';
            app.XCGEmptyEditFieldLabel.Position = [225 46 69 22];
            app.XCGEmptyEditFieldLabel.Text = 'XCG Empty';

            % Create XCGEmptyEditField
            app.XCGEmptyEditField = uieditfield(app.GeneralparametersPanel, 'numeric');
            app.XCGEmptyEditField.Limits = [0 Inf];
            app.XCGEmptyEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.XCGEmptyEditField.Tag = 'XCGE';
            app.XCGEmptyEditField.Position = [300 46 100 22];
            app.XCGEmptyEditField.Value = 127;

            % Create FinsPanel
            app.FinsPanel = uipanel(app.GeometryTab);
            app.FinsPanel.Title = 'Fins';
            app.FinsPanel.Position = [20 47 413 246];

            % Create ShapeDropDownLabel
            app.ShapeDropDownLabel = uilabel(app.FinsPanel);
            app.ShapeDropDownLabel.HorizontalAlignment = 'right';
            app.ShapeDropDownLabel.Position = [23 194 40 22];
            app.ShapeDropDownLabel.Text = 'Shape';

            % Create FinShapeDropDown
            app.FinShapeDropDown = uidropdown(app.FinsPanel);
            app.FinShapeDropDown.Items = {'TRAPEZOID', 'RECTANGULAR'};
            app.FinShapeDropDown.ValueChangedFcn = createCallbackFcn(app, @FinShapeDropDownValueChanged, true);
            app.FinShapeDropDown.Position = [67 194 125 22];
            app.FinShapeDropDown.Value = 'TRAPEZOID';

            % Create RootChordEditFieldLabel
            app.RootChordEditFieldLabel = uilabel(app.FinsPanel);
            app.RootChordEditFieldLabel.HorizontalAlignment = 'right';
            app.RootChordEditFieldLabel.Position = [227 194 67 22];
            app.RootChordEditFieldLabel.Text = 'Root Chord';

            % Create MaxChordEditField
            app.MaxChordEditField = uieditfield(app.FinsPanel, 'numeric');
            app.MaxChordEditField.Limits = [0 Inf];
            app.MaxChordEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.MaxChordEditField.Tag = 'MaxChord';
            app.MaxChordEditField.Position = [300 194 100 22];
            app.MaxChordEditField.Value = 17;

            % Create TipChordEditFieldLabel
            app.TipChordEditFieldLabel = uilabel(app.FinsPanel);
            app.TipChordEditFieldLabel.HorizontalAlignment = 'right';
            app.TipChordEditFieldLabel.Position = [234 162 58 22];
            app.TipChordEditFieldLabel.Text = 'Tip Chord';

            % Create MinChordEditField
            app.MinChordEditField = uieditfield(app.FinsPanel, 'numeric');
            app.MinChordEditField.Limits = [0 Inf];
            app.MinChordEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.MinChordEditField.Tag = 'MinChord';
            app.MinChordEditField.Position = [300 162 100 22];
            app.MinChordEditField.Value = 8;

            % Create XLE1EditFieldLabel
            app.XLE1EditFieldLabel = uilabel(app.FinsPanel);
            app.XLE1EditFieldLabel.HorizontalAlignment = 'right';
            app.XLE1EditFieldLabel.Position = [33 141 43 22];
            app.XLE1EditFieldLabel.Text = 'XLE(1)';

            % Create XLE1EditField
            app.XLE1EditField = uieditfield(app.FinsPanel, 'numeric');
            app.XLE1EditField.Limits = [0 Inf];
            app.XLE1EditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.XLE1EditField.Tag = 'XLE1';
            app.XLE1EditField.Position = [92 141 100 22];
            app.XLE1EditField.Value = 185;

            % Create XLE2EditFieldLabel
            app.XLE2EditFieldLabel = uilabel(app.FinsPanel);
            app.XLE2EditFieldLabel.HorizontalAlignment = 'right';
            app.XLE2EditFieldLabel.Position = [33 110 43 22];
            app.XLE2EditFieldLabel.Text = 'XLE(2)';

            % Create XLE2EditField
            app.XLE2EditField = uieditfield(app.FinsPanel, 'numeric');
            app.XLE2EditField.Limits = [0 Inf];
            app.XLE2EditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.XLE2EditField.Tag = 'XLE2';
            app.XLE2EditField.Position = [92 109 100 22];
            app.XLE2EditField.Value = 187;

            % Create HeightEditFieldLabel
            app.HeightEditFieldLabel = uilabel(app.FinsPanel);
            app.HeightEditFieldLabel.HorizontalAlignment = 'right';
            app.HeightEditFieldLabel.Position = [36 79 40 22];
            app.HeightEditFieldLabel.Text = 'Height';

            % Create FinHeightEditField
            app.FinHeightEditField = uieditfield(app.FinsPanel, 'numeric');
            app.FinHeightEditField.Limits = [0 Inf];
            app.FinHeightEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.FinHeightEditField.Tag = 'FinHeight';
            app.FinHeightEditField.Position = [92 79 100 22];
            app.FinHeightEditField.Value = 6;

            % Create ThicknessEditFieldLabel
            app.ThicknessEditFieldLabel = uilabel(app.FinsPanel);
            app.ThicknessEditFieldLabel.HorizontalAlignment = 'right';
            app.ThicknessEditFieldLabel.Position = [17 48 60 22];
            app.ThicknessEditFieldLabel.Text = 'Thickness';

            % Create FinThicknessEditField
            app.FinThicknessEditField = uieditfield(app.FinsPanel, 'numeric');
            app.FinThicknessEditField.Limits = [0 Inf];
            app.FinThicknessEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.FinThicknessEditField.Tag = 'FinThickness';
            app.FinThicknessEditField.Position = [92 48 100 22];
            app.FinThicknessEditField.Value = 0.5;

            % Create LMaxEditFieldLabel
            app.LMaxEditFieldLabel = uilabel(app.FinsPanel);
            app.LMaxEditFieldLabel.HorizontalAlignment = 'right';
            app.LMaxEditFieldLabel.Position = [41 16 35 22];
            app.LMaxEditFieldLabel.Text = 'LMax';

            % Create LMaxEditField
            app.LMaxEditField = uieditfield(app.FinsPanel, 'numeric');
            app.LMaxEditField.Limits = [0 Inf];
            app.LMaxEditField.ValueChangedFcn = createCallbackFcn(app, @TextBoxEdit, true);
            app.LMaxEditField.Tag = 'LMax';
            app.LMaxEditField.Position = [92 16 100 22];
            app.LMaxEditField.Value = 3;

            % Create FlightconditionsTab
            app.FlightconditionsTab = uitab(app.TabGroup);
            app.FlightconditionsTab.Title = 'Flight conditions';

            % Create AddAlphaButton
            app.AddAlphaButton = uibutton(app.FlightconditionsTab, 'push');
            app.AddAlphaButton.ButtonPushedFcn = createCallbackFcn(app, @AddFLTCOND, true);
            app.AddAlphaButton.Position = [33 125 100 22];
            app.AddAlphaButton.Text = 'Add Alpha';

            % Create UITableAlpha
            app.UITableAlpha = uitable(app.FlightconditionsTab);
            app.UITableAlpha.ColumnName = {'ALPHA'};
            app.UITableAlpha.RowName = {};
            app.UITableAlpha.ColumnEditable = true;
            app.UITableAlpha.RowStriping = 'off';
            app.UITableAlpha.CellEditCallback = createCallbackFcn(app, @UITableAlphaCellEdit, true);
            app.UITableAlpha.CellSelectionCallback = createCallbackFcn(app, @UITableAlphaCellSelection, true);
            app.UITableAlpha.Position = [33 157 100 369];

            % Create UITableBeta
            app.UITableBeta = uitable(app.FlightconditionsTab);
            app.UITableBeta.ColumnName = {'BETA'};
            app.UITableBeta.RowName = {};
            app.UITableBeta.ColumnEditable = true;
            app.UITableBeta.RowStriping = 'off';
            app.UITableBeta.CellEditCallback = createCallbackFcn(app, @UITableBetaCellEdit, true);
            app.UITableBeta.CellSelectionCallback = createCallbackFcn(app, @UITableBetaCellSelection, true);
            app.UITableBeta.Position = [155 157 98 369];

            % Create UITableMach
            app.UITableMach = uitable(app.FlightconditionsTab);
            app.UITableMach.ColumnName = {'MACH'};
            app.UITableMach.RowName = {};
            app.UITableMach.ColumnEditable = true;
            app.UITableMach.RowStriping = 'off';
            app.UITableMach.CellEditCallback = createCallbackFcn(app, @UITableMachCellEdit, true);
            app.UITableMach.CellSelectionCallback = createCallbackFcn(app, @UITableMachCellSelection, true);
            app.UITableMach.Position = [270 157 100 369];

            % Create UITableAlt
            app.UITableAlt = uitable(app.FlightconditionsTab);
            app.UITableAlt.ColumnName = {'ALT'};
            app.UITableAlt.RowName = {};
            app.UITableAlt.ColumnEditable = true;
            app.UITableAlt.RowStriping = 'off';
            app.UITableAlt.CellEditCallback = createCallbackFcn(app, @UITableAltCellEdit, true);
            app.UITableAlt.CellSelectionCallback = createCallbackFcn(app, @UITableAltCellSelection, true);
            app.UITableAlt.Position = [393 157 100 369];

            % Create AddBetaButton
            app.AddBetaButton = uibutton(app.FlightconditionsTab, 'push');
            app.AddBetaButton.ButtonPushedFcn = createCallbackFcn(app, @AddFLTCOND, true);
            app.AddBetaButton.Position = [153 125 100 22];
            app.AddBetaButton.Text = 'Add Beta';

            % Create AddMachButton
            app.AddMachButton = uibutton(app.FlightconditionsTab, 'push');
            app.AddMachButton.ButtonPushedFcn = createCallbackFcn(app, @AddFLTCOND, true);
            app.AddMachButton.Position = [270 125 100 22];
            app.AddMachButton.Text = 'Add Mach';

            % Create AddAltitudeButton
            app.AddAltitudeButton = uibutton(app.FlightconditionsTab, 'push');
            app.AddAltitudeButton.ButtonPushedFcn = createCallbackFcn(app, @AddFLTCOND, true);
            app.AddAltitudeButton.Position = [393 125 100 22];
            app.AddAltitudeButton.Text = 'Add Altitude';

            % Create RemoveAlphaButton
            app.RemoveAlphaButton = uibutton(app.FlightconditionsTab, 'push');
            app.RemoveAlphaButton.ButtonPushedFcn = createCallbackFcn(app, @RemoveFLTCOND, true);
            app.RemoveAlphaButton.Position = [34 88 100 22];
            app.RemoveAlphaButton.Text = 'Remove Alpha';

            % Create RemoveBetaButton
            app.RemoveBetaButton = uibutton(app.FlightconditionsTab, 'push');
            app.RemoveBetaButton.ButtonPushedFcn = createCallbackFcn(app, @RemoveFLTCOND, true);
            app.RemoveBetaButton.Position = [153 88 100 22];
            app.RemoveBetaButton.Text = 'Remove Beta';

            % Create RemoveMachButton
            app.RemoveMachButton = uibutton(app.FlightconditionsTab, 'push');
            app.RemoveMachButton.ButtonPushedFcn = createCallbackFcn(app, @RemoveFLTCOND, true);
            app.RemoveMachButton.Position = [270 88 100 22];
            app.RemoveMachButton.Text = 'Remove Mach';

            % Create RemoveAltitudeButton
            app.RemoveAltitudeButton = uibutton(app.FlightconditionsTab, 'push');
            app.RemoveAltitudeButton.ButtonPushedFcn = createCallbackFcn(app, @RemoveFLTCOND, true);
            app.RemoveAltitudeButton.Position = [391 88 104 22];
            app.RemoveAltitudeButton.Text = 'Remove Altitude';

            % Create LocalaltitudemEditFieldLabel
            app.LocalaltitudemEditFieldLabel = uilabel(app.FlightconditionsTab);
            app.LocalaltitudemEditFieldLabel.HorizontalAlignment = 'right';
            app.LocalaltitudemEditFieldLabel.Position = [37 34 96 22];
            app.LocalaltitudemEditFieldLabel.Text = 'Local altitude [m]';

            % Create ZLocEditField
            app.ZLocEditField = uieditfield(app.FlightconditionsTab, 'numeric');
            app.ZLocEditField.Limits = [0 Inf];
            app.ZLocEditField.Position = [148 34 100 22];

            % Create ExecDATCOMParseMatricesButton
            app.ExecDATCOMParseMatricesButton = uibutton(app.UIFigure, 'push');
            app.ExecDATCOMParseMatricesButton.ButtonPushedFcn = createCallbackFcn(app, @PrintDATCOMtextButtonPushed, true);
            app.ExecDATCOMParseMatricesButton.Tag = 'save2for005';
            app.ExecDATCOMParseMatricesButton.FontWeight = 'bold';
            app.ExecDATCOMParseMatricesButton.Position = [638 42 200 22];
            app.ExecDATCOMParseMatricesButton.Text = 'Exec DATCOM & Parse Matrices';

            % Create Panel
            app.Panel = uipanel(app.UIFigure);
            app.Panel.AutoResizeChildren = 'off';
            app.Panel.Position = [14 12 477 675];

            % Create RocketUIAxes
            app.RocketUIAxes = uiaxes(app.Panel);
            title(app.RocketUIAxes, 'Title')
            xlabel(app.RocketUIAxes, 'X')
            ylabel(app.RocketUIAxes, 'Y')
            app.RocketUIAxes.ClippingStyle = 'rectangle';
            app.RocketUIAxes.Position = [8 345 447 320];

            % Create FinsUIAxes
            app.FinsUIAxes = uiaxes(app.Panel);
            title(app.FinsUIAxes, 'Title')
            xlabel(app.FinsUIAxes, 'X')
            ylabel(app.FinsUIAxes, 'Y')
            app.FinsUIAxes.ClippingStyle = 'rectangle';
            app.FinsUIAxes.Position = [15 30 448 307];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = RocketGUI_V2_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end