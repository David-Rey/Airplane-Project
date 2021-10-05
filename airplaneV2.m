%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David Reynolds
% Email: reynod13@my.erau.edu
% Start Date: September 13, 2021
% EGR 101 - Section 12
% Filename: airplaneV2.m
% NOTE: file requires aerospace toolbox
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear; clc;

%% we get to chose these variabls %%

%range = [3800:1:4200];
V_LANDING = 140; % landing speed [must be between 135-240] (ft/s)
cruseMach = 0.79; % mach number at cruse
cruiseAlt = 47000; % ft
w_f = 12; % fuselage diameter % ft
A = 9; % aspect ratio: value must be between 6-9  
W_E_type = 4; % type of aircraft is a integer from 1 to 4 corresponding to table
flap_type = 4; % type of flap is an integer from 1 to 4 corresponding to table
lambda = 0.7; % tip cord / root chord (0 < lambda < 1)

%% Constants %%
satellites = 4; % number of satellites
satWeight = 942; % weight of satelite in (lbs)
NumPassenger = 6; % must be between 2-6
passengerWeight = 170; % weight of one persion (lbs)
range = 2240; % range in Nautical Miles

%% Calculations %%
W_payload = (passengerWeight * NumPassenger) + (satWeight * satellites); % (lb)

% section 2: estimate gross weight
[W_G, W_E, W_F] = weightCal(W_E_type, W_payload, range);

% section 3: wing area and landing speed
[S, V_STALL, L] = wingCal(flap_type, V_LANDING, W_G);

% section 4: drag at cruise
[D_CRUISE, V_CRUISE, C_L_CRUISE] = dragCal(cruseMach, cruiseAlt, S, W_G, w_f, A);

% section 5: select and engine
[hr_CRUISE, totalFuelWeight, trustMargin] = engineData(D_CRUISE, V_CRUISE, range, cruiseAlt);

% section 6: shape of the wing
[C_TIP, C_ROOT] = wingShape(A, S, lambda);

% section 7: horizontal and vertical tail size
tailSize(S);

% section 8: cost
cost = W_E .* 600; % ($)

ratio = abs((totalFuelWeight - W_F) ./ abs(W_F)) .* 100;



avaiable = [];

for n=1:length(cruiseAlt)
	if isnan(ratio(n)) == 0 && isnan(C_L_CRUISE(n)) == 0 && isnan(trustMargin(n)) == 0
		avaiable(end+1) = cruiseAlt(n);
	end
end

%% functions %%

% Function calculates weight characteristics of aircraft
% IN:
% type = type of aircraft is a integer from 1 to 4 corresponding to table
% W_P = weight of payload (lb)
% range = range of aircraft (miles)
% OUT:
% W_G = gross (total) weight (lb)
% W_E = empty weight (lb)
% W_F = fuel weight (lb)
function [W_G, W_E, W_F] = weightCal(type, W_P, range)
	range = range * 1.15078; % unit conversion from nm to miles
	W_E_arr = [0.57, 0.55, 0.55, 0.5]; % (W_G)
	W_E_ratio = W_E_arr(type); % (W_G)
	W_F_ratio = (0.15 + 3.33 .* 10^-5 .* (range - 1000)); % (W_G)
	W_G_ratio = 1 ./ (1 - W_E_ratio - W_F_ratio); % (W_P)
	W_G = W_G_ratio .* W_P; % (lb)
	W_E = W_E_ratio .* W_G; % (lb)
	W_F = W_F_ratio .* W_G; % (lb)
end

% Function calculates wing characteristics such as stall speed, area, and runway length
% IN:
% type = type of flap is an integer from 1 to 4 corresponding to table
% V_LANDING = landing speed (ft/s)
% W_G = gross (total) weight (lb)
% OUT:
% S = wing planform area (ft^2)
% V_STALL = stall speed (ft/s)
% L = minimum runway length (ft)
function [S, V_STALL, L] = wingCal(type, V_LANDING, W_G)
	C_L_MAX_arr = [1.4, 1.8, 2.4, 2.7]; % (X)
	C_L_MAX = C_L_MAX_arr(type); % (X)
	[~, ~, ~, rho] = getAtm(0); % (lb s^2/ft^4)
	S = (2.42 .* W_G) ./ (C_L_MAX .* rho .* V_LANDING.^2); % (ft^2)
    %S = 3345;
	V_STALL = V_LANDING ./ 1.1; % (ft/s)
	L = 0.5664 .* (0.592484 .* V_STALL).^2; % (ft)
	C_L = W_G ./ (0.5 .* rho .* V_LANDING.^2 .* S); % not used
end

% Function calculates drag at cruising altitude
% IN:
% cruseMach = mach number at cruising altitude (Mach)
% cruiseAlt = cruising altitude (ft)
% S = wing planform area (ft^2)
% W_G = gross (total) weight (lb)
% w_f = fuselage diameter (ft)
% A = aspect ratio (X)
% OUT:
% D_CRUISE = drag at cruising altitude (lb)
% V_CRUISE = speed at cruising altitude (ft/s)
function [D_CRUISE, V_CRUISE, C_L_CRUISE] = dragCal(cruseMach, cruiseAlt, S, W_G, w_f, A)
	[~, Sos, ~, rho] = getAtm(cruiseAlt); % Sos and rho are at cruising altitude
	V_CRUISE = cruseMach .* Sos; % (ft/s)
	C_L_CRUISE = W_G ./ (0.5 .* rho .* V_CRUISE.^2 .* S); % (X)
	C_D_i = C_L_CRUISE.^2 ./ (pi .* A); % (X)
	C_D_o = 0.012 + 0.000667 .* w_f; % (X)
	C_D = C_D_i + C_D_o; % (X)
	D_CRUISE = 0.5 .* rho .* C_D .* V_CRUISE.^2 .* S;
end

% Function calculates fule and time of flight
% IN:
% D_CRUISE = drag at cruising altitude (lb)
% V_CRUISE = speed at cruising altitude (ft/s)
% range = range of aircraft (miles)
% cruiseAlt = cruising altitude (ft)
% OUT:
% hr_CRUISE = time at cruising altitude (hr)
% totalFuelWeight = total weight of fuel (lb)
%
% sea level static (SLS) thrust [changes with number of engines]
% specific fuel consumption (SFC)
function [hr_CRUISE, totalFuelWeight, trustMargin] = engineData(D_CRUISE, V_CRUISE, range, cruiseAlt)
	T_CRUISE = D_CRUISE; % (lb)
	SLST = 6575;
	SFC = 0.573;
	[~, ~, ~, rho_SLS] = getAtm(0); % (lb s^2/ft^4)
	[~, ~, ~, rho_CRUISE] = getAtm(cruiseAlt); % (lb s^2/ft^4)
	sigma = rho_CRUISE ./ rho_SLS; % (X)
	T_MAX_SLS = T_CRUISE ./ (0.9 .* sigma); % (lb)
	T_RATED = T_MAX_SLS; % (lb)
	%fuelFlow = SFC .* SLST; % (lb/hr) not used
	range = range * 1.15078; % unit conversion from nm to miles
	V_CRUISE = V_CRUISE * 0.681818; % unit conversion from ft/s to mph
	hr_CRUISE = range ./ V_CRUISE; % (hr)
	reserveFuelWeight = (45/60) .* SFC .* T_CRUISE; % (lb)
	cruiseFuelWeight = SFC .* T_CRUISE .* hr_CRUISE; % (lb)
	totalFuelWeight = cruiseFuelWeight + reserveFuelWeight; % (lb)
	trustMargin = SLST - T_RATED;
end

% Function calculates the shape of the wing given the aspect ratio, surface area, and taper ratio
% IN:
% A = aspect ratio (X)
% S = surface area (ft^2)
% lambda = taper ratio is defined as tip chord / root chord (x)
% OUT:
% C_TIP = tip chord (ft)
% C_ROOT = root chord (ft)
function [C_TIP, C_ROOT] = wingShape(A, S, lambda)
	b = sqrt(A .* S); % (ft)
	C_ROOT = (2 .* S) ./ (b .* (1 + lambda)); % (X)
	C_TIP = lambda .* C_ROOT; % (X)
	plotWing(C_ROOT, C_TIP, b, 'Wing Shape');
end

% Function calculates characteristics of horizontal and vertical stablizer
% IN:
% S = surface area (ft^2)
function tailSize(S)
	S_VT = 0.15 * S;
	S_HT = 0.25 * S;
	A_VT = 1.5;
	A_HT = 3;
	lambda_VT = 0.667;
	lambda_HT = 0.5;
	b_VT = sqrt(S_VT * A_VT);
	b_HT = sqrt(S_HT * A_HT);
	C_ROOT_VT = (2 * S_VT) / (b_VT * (1 + lambda_VT));
	C_ROOT_HT = (2 * S_HT) / (b_HT * (1 + lambda_HT));
	C_TIP_VT = lambda_VT * C_ROOT_VT;
	C_TIP_HT = lambda_HT * C_ROOT_HT;
	plotWing(C_ROOT_VT, C_TIP_VT, b_VT, 'Vertical Tail');
	plotWing(C_ROOT_HT, C_TIP_HT, b_HT, 'Horizontal Tail');
end


%% helper functions %%

% Function gets SLST and SFC from table
% IN:
% index = row from table
function [SLST, SFC] = engineSelect(index)
	engine_table = [3000, .70;
					5000, .80;
					6576, .573;
					7500, 0.845;
					9275, 0.605;
					15400, 0.69;
					17000, 0.65];
	engine = engine_table(index, :);
	SLST = engine(1);
	SFC = engine(2);
end

% Function calculates U.S. 1976 standard atmosphere
% NOTE: file requires aerospace toolbox
% IN:
% h = altitude (ft)
% OUT:
% T = temperature (rankine)
% Sos = speed of sound (ft/s)
% P = pressure (lb/ft^2)
% rho = density (lb s^2/ft^4)
function [T, Sos, P, rho] = getAtm(h)
	h_SI = h / 3.2808;
	[T_SI, Sos_SI, P_SI, rho_SI] = atmosisa(h_SI); % units in SI
	T = T_SI * 9 / 5;
	Sos = Sos_SI * 3.2808;
	P = P_SI * 0.020885;
	rho = rho_SI / 515.38;
end

% Function plots wing given the tip chord, root chord and span. It also has an option to change the title
% IN:
% C_ROOT = root chord (ft)
% C_TIP = tip chord (ft)
% b = span (ft)
% plotTitle = plot title (x)
function plotWing(C_ROOT, C_TIP, b, plotTitle)
	figure;
	%xlim tight % only available on 2021a
	%ylim tight % only available on 2021a
	axis tight
	hold on
	b = b/2;
	% set equalScale to if you want equal scaling else leave it to zero
	equalScale = 0;
	
	% outline
	plot([0,0], [0,C_TIP], 'k');
	plot([b,b], [C_TIP, C_TIP + C_ROOT], 'k');
	plot([0,b], [C_TIP, C_TIP + C_ROOT], 'k');
	plot([0,b], [0, C_TIP], 'k');
	
	% 0.5 and 0.75 Chord and edge
	plot([0,b], [C_TIP * .5, C_TIP + (C_ROOT * 0.5)], 'k--');
	plot([0,b], [C_TIP * .75, C_TIP + (C_ROOT * 0.75)], 'k--');
	plot([0,b], [C_TIP + C_ROOT, 0], 'k--');
	
	% calculates MAC and nominal CG
	m2 = -(C_TIP + C_ROOT) / b;
	b2 = C_TIP + C_ROOT;
	[~, m1, b1] = wingChord(0, 0.5, C_ROOT, C_TIP, b);
	x_int = (b2 - b1) / (m1 - m2);
	y1 = wingChord(x_int, 0.5, C_ROOT, C_TIP, b);
	y2 = wingChord(x_int, 0.75, C_ROOT, C_TIP, b);
	y_trail = wingChord(x_int, 0, C_ROOT, C_TIP, b);
	y_lead = wingChord(x_int, 1, C_ROOT, C_TIP, b);
	MAC = y_lead - y_trail;
	
	% plots MAC lines for nominal CG
	plot([x_int, x_int], [y_lead, y_trail], 'k');
	plot([x_int, x_int], [y1, y2], 'r--');
	plot([x_int, b], [y2, y2], 'r--');
	
	% plot nominal CG points
	plot(x_int, y2, 'rx', 'MarkerSize',15);
	plot(b, y2, 'rx', 'MarkerSize',15);
	
	% extra
	title(plotTitle);
	xlabel('Feet');
	ylabel('Feet');
	
	% equal scaling switch
	switch equalScale
		case 0
			set(gcf, 'Position',  [200, 200, 800, 500])
		case 1
			axis equal
			% https://www.mathworks.com/matlabcentral/answers/402194-how-to-save-a-plot-without-margin-of-figure
			set(gcf, 'Position',  [200, 200, 1000, 400])
			tight = get(gca, 'TightInset');
			newPos = [tight(1), tight(2), 0.98-tight(1)-tight(3), 1-tight(2)-tight(4)];
			set(gca, 'Position', newPos);
	end
end

% Function calculates y interspet slope and y intercept at zero of a root chord.
function [y, m, b] = wingChord(x, q, C_ROOT, C_TIP, b)
	m = (C_TIP + (C_ROOT * q) - (C_TIP * q)) / b;
	b = C_TIP * q;
	y = m * x + b;
end

%plot(range,ratio,'k');

%ratio(ratio > .1) = NaN;
%C_L_CRUISE(C_L_CRUISE > 0.45) = NaN;
%trustMargin(trustMargin > 7000) = NaN;
%trustMargin(trustMargin < 0) = NaN;
