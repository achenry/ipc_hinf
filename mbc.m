close all;

T1P = @(theta) (2/3) * [1/2, 1/2, 1/2;
                        cos(theta), cos(theta + (2/3)*pi), cos(theta + (4/3)*pi);
                        sin(theta), sin(theta + (2/3)*pi), sin(theta + (4/3)*pi)];
T1P_inv = @(theta) [1, cos(theta), sin(theta);
                    1, cos(theta + (2/3)*pi), sin(theta + (2/3)*pi);
                    1, cos(theta + (4/3)*pi), sin(theta + (4/3)*pi)];

T3P = @(theta) (2/3) * [1/2, 1/2, 1/2;
                        cos(3*theta), cos(3*theta + (2/3)*pi), cos(3*theta + (4/3)*pi);
                        sin(3*theta), sin(3*theta + (2/3)*pi), sin(3*theta + (4/3)*pi)];
T3P_inv = @(theta) [1, cos(3*theta), sin(3*theta);
                    1, cos(3*theta + (2/3)*pi), sin(3*theta + (2/3)*pi);
                    1, cos(3*theta + (4/3)*pi), sin(3*theta + (4/3)*pi)];

theta = linspace(0, 2*pi, 1000);

w_blade_1P = @(theta) [cos(theta)', cos(theta + (2/3)*pi)', cos(theta + (4/3)*pi)'];
w_blade_3P = @(theta) [cos(3*theta)', cos(3*theta + (2/3)*pi)', cos(3*theta + (4/3)*pi)'];

figure(1);
tiledlayout(2, 1);
nexttile;
plot(theta, w_blade_1P(theta));
legend('B1', 'B2', 'B3');
nexttile;
plot(theta, w_blade_3P(theta));
legend('B1', 'B2', 'B3');

w_cdq_1P = [];
for th = theta
    w_cdq_1P = [w_cdq_1P; (T1P(th) * w_blade_1P(th)')'];
end
w_cdq_3P = [];
for th = theta
    w_cdq_3P = [w_cdq_3P; (T3P(th) * w_blade_3P(th)')'];
end
figure(2);
tiledlayout(2, 1);
nexttile;
plot(theta, w_cdq_1P);
legend('C', 'D', 'Q')
nexttile;
plot(theta, w_cdq_3P);