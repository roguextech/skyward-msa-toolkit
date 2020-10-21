%%% Calcolo interferenza ogiva payload %%%

close all
clc

%% DATA

l = 300;                % length mm
r = 75;                 % radius mm 
PayloadRad = (100 * sqrt(2)) / 2;   % diagonal del payload mm  

% Tangent ogive

rho = (r^2 + l^2) / (2*r);
y_ogt = @(x)  sqrt( rho^2 - (l-x).^2 ) + r - rho ;

% % Secant ogive
% 
% rho_s = rho - rho * (20/100);
% alpha = acos( sqrt(l^2 + r^2) / (2*rho_s) ) - atan(r/l); 
% y_ogs = @(x) sqrt( rho_s^2 - (rho_s*cos(alpha) - x ).^2 ) - rho_s*sin(alpha);

% % Parabolic 
% 
% k=1;
% y_par = @(x) r * ( (2*(x./l) - k*(x./l).^2 ) / ( 2 - k) );

% Power series 

n = 1/3;
y_pow1 = @(x) r * (x./l).^n;
n = 1/2;
y_pow2 = @(x) r * (x./l).^n;
n = 3/4;
y_pow3 = @(x) r * (x./l).^n;

% haack series 

C = 1/3;
y_hak = @(x) ( r/sqrt(pi) ) * sqrt( acos( 1- ( (2*x)./l ) ) - ( sin(2*acos( 1- ( (2*x)./l ) )) ./ 2 ) + C*sin(acos( 1- ( (2*x)./l ) )).^3 );

% von Karman 

C = 0;
y_kar = @(x) ( r/sqrt(pi) ) * sqrt( acos( 1- ( (2*x)./l ) ) - ( sin(2*acos( 1- ( (2*x)./l ) )) ./ 2 ) + C*sin(acos( 1- ( (2*x)./l ) )).^3 );

%% Evaluation  

x = 0:0.1:l;

figure()
hold on 

plot(x,y_ogt(x));
% plot(x,y_ogs(x));
% plot(x,y_par(x));
plot(x,y_pow1(x));
plot(x,y_pow2(x));
plot(x,y_pow3(x));
plot(x,y_hak(x));
plot(x,y_kar(x));
plot(x,ones(1,length(x))*PayloadRad,'--r','LineWidth',2);
s = {'tangent ogive'; 'power 1/3'; 'power 1/2';'power 3/4' ;'haack'; 'Von Karman'};
legend(s); title('Ogive profile'); xlabel('x [mm]'); ylabel('y [mm]')

