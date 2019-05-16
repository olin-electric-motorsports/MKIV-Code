%% Plot the datasheet data

T = (-55:5:155);
RTR25 = [96.3, 67.01, 47.17, 33.65, 24.26, 17.7, 13.04, 9.707, 7.293, 5.533, 4.232, 3.265, 2.539, 1.99, 1.571, 1.249, 1, 0.8057, 0.6531, 0.5327, 0.4369, 0.3603, 0.2986, 0.2488, 0.2083, 0.1752, 0.1481, 0.1258, 0.1072, 0.09177, 0.07885, 0.068, 0.05886, 0.05112, 0.04454, 0.03893, 0.03417, 0.03009, 0.02654, 0.02348, 0.02083, 0.01853, 0.01653];
figure
%plot distinct points
plot(T, RTR25.*10000, 'bo'); % multiple RT/R25 by R25 to get R by itself
hold on
% plot line
plot(T, RTR25.*10000, 'b'); 
xlabel('Temperature');
ylabel('R_T / R_{25}');
title({'NTC B57861S0103F040 Thermistor', 'RT Characteristic from Datasheet'});

%% Calculating temperature from voltage
R25 = 10000; %resistance at 25 deg C
T25 = 25 + 273.15; %baseline temp 25 deg C to Kelvin
B = 3988; %beta value for this thermistor
R = RTR25 * 10000; %use range of resistances from datasheet for easy error calculations later
V = 3.0 .* R ./ (100000+R); %convert resistance to voltage

%approxiate temperature from voltage
for i = 1:43
    Tcalc(i) = 1 / ((log((100000*V(i)) / (3 - V(i)) / R25) / B) + (1 / T25)) - 273.15;
end

figure
% plot datasheet values
plot(T, R);
hold on
xlabel('Temperature (degrees C)');
ylabel('Resistance (ohms)');
title('NTC B57861S0103F040 Thermistor RT Characteristic');
% plot approximated values
plot(Tcalc, R);
legend('Datasheet values', 'Approximated values');

%% Error calculation

% calculate error between datasheet values and approximations
absError = T - Tcalc;
percentError = abs((absError./T) .* 100);


%% Make table

Tin = linspace(0+273.15,80+273.15,81);
for k = 1:81
    R(k) = R25 * exp(B * ((1/Tin(k)) - (1/T25)));
    V(k) = (3 * (R25 * exp(B * ((1/Tin(k)) - (1/T25))))) / (100000 + (R25 * exp(B * ((1/Tin(k)) - (1/T25)))));
end

figure
plot(Tin - 273.15, R);
figure
plot(Tin - 273.15, V);

%% References

datasheet: https://www.tdk-electronics.tdk.com/inf/50/db/ntc/NTC_Mini_sensors_S861.pdf
http://www.giangrandi.ch/electronics/ntc/ntc.shtml
https://www.electronics-tutorials.ws/io/thermistors.html
