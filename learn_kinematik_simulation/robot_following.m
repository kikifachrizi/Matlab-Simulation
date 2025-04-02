clc; close all; clear;

% Parameter Robot
r = 0.075;    % Jari-jari roda (dalam meter)
L = 0.3;      % Jarak antar roda (dalam meter)
tf = 20;      % Waktu akhir simulasi (dalam detik)

% Kondisi Awal
theta0_1 = 0;    % Orientasi awal robot 1
x0_1 = 0;        % Posisi awal robot 1 pada sumbu x
y0_1 = 0;        % Posisi awal robot 1 pada sumbu y

theta0_2 = 0;    % Orientasi awal robot 2
x0_2 = -1;       % Posisi awal robot 2 pada sumbu x
y0_2 = 0;        % Posisi awal robot 2 pada sumbu y

% Vektor Waktu dan Time Increment
t = linspace(0, tf, 200); % Vektor waktu dari 0 hingga tf dengan 200 titik
dt = t(2) - t(1);         % Increment waktu

% Inisialisasi Variabel Pose untuk Robot 1 dan Robot 2
x_1 = zeros(1, length(t));
y_1 = zeros(1, length(t));
theta_1 = zeros(1, length(t));
x_2 = zeros(1, length(t));
y_2 = zeros(1, length(t));
theta_2 = zeros(1, length(t));

x_1(1) = x0_1;
y_1(1) = y0_1;
theta_1(1) = theta0_1;

x_2(1) = x0_2;
y_2(1) = y0_2;
theta_2(1) = theta0_2;

% Pengaturan Kecepatan untuk Robot 1 (kecepatan tetap)
%vA_1 = 0.5 * ones(1, length(t)); % Kecepatan roda kiri konstan
%vB_1 = 0.5 * ones(1, length(t)); % Kecepatan roda kanan konstan

% Pembagian Segmen Waktu
num_segments = 7;
segment_length = floor(length(t) / num_segments);

% Inisialisasi Kecepatan untuk Robot 1 dan Robot 2
vA_1 = zeros(1, length(t));
vB_1 = zeros(1, length(t));
vA_2 = zeros(1, length(t));
vB_2 = zeros(1, length(t));

% Pengaturan Kecepatan untuk Setiap Segmen
for j = 1:num_segments
    % Tentukan indeks awal dan akhir untuk segmen ini
    start_idx = (j - 1) * segment_length + 1;
    end_idx = min(j * segment_length, length(t));
    
    % Tentukan pola gerakan untuk setiap segmen
    switch j
        case 1 
            vA_1(start_idx:end_idx) = 0.8;
            vB_1(start_idx:end_idx) = 0.8;
        case 2
            vA_1(start_idx:end_idx) = -0.085;
            vB_1(start_idx:end_idx) = 0.085;
        case 3
            vA_1(start_idx:end_idx) = 0.8;
            vB_1(start_idx:end_idx) = 0.8;
        case 4
            vA_1(start_idx:end_idx) = -0.025;
            vB_1(start_idx:end_idx) = 0.025;
        case 5
            vA_1(start_idx:end_idx) = 0.8;
            vB_1(start_idx:end_idx) = 0.8;
        case 6
            vA_1(start_idx:end_idx) = 0.5;
            vB_1(start_idx:end_idx) = 0.2;
        case 7
            vA_1(start_idx:end_idx) = 0.8;
            vB_1(start_idx:end_idx) = 0.8;
    end
end

% Loop Update Pose untuk Robot 1
for ii = 2:length(t)
    % Update posisi dan orientasi untuk robot 1
    x_1(ii) = x_1(ii-1) + cos(theta_1(ii-1)) * (vA_1(ii-1) + vB_1(ii-1))/2 * dt;
    y_1(ii) = y_1(ii-1) + sin(theta_1(ii-1)) * (vA_1(ii-1) + vB_1(ii-1))/2 * dt;
    theta_1(ii) = theta_1(ii-1) + ((vA_1(ii-1) - vB_1(ii-1)) / L) * dt;
end

% Kontrol untuk Robot 2 agar mengikuti jalur Robot 1
kp = 0.75678; % Gain untuk kontrol posisi
ktheta = 1.9; % Gain untuk kontrol orientasi

% Loop Update Pose dengan Kinematik untuk Robot 2 agar mengikuti Robot 1
for ii = 2:length(t)
    % Hitung error posisi dan orientasi untuk robot 2 terhadap robot 1
    ex = x_1(ii-1) - x_2(ii-1);
    ey = y_1(ii-1) - y_2(ii-1);
    e_distance = sqrt(ex^2 + ey^2);
    e_theta = atan2(ey, ex) - theta_2(ii-1);
    
    % Pengendalian kecepatan linier dan rotasi untuk robot 2
    v = kp * e_distance; % Kecepatan linier berdasarkan jarak error
    omega = ktheta * e_theta; % Kecepatan rotasi berdasarkan error orientasi
    
    % Hitung kecepatan roda untuk robot 2
    vA_2 = v - (omega * L) / 2;
    vB_2 = v + (omega * L) / 2;
    
    % Update posisi dan orientasi untuk robot 2
    x_2(ii) = x_2(ii-1) + cos(theta_2(ii-1)) * (vA_2 + vB_2) / 2 * dt;
    y_2(ii) = y_2(ii-1) + sin(theta_2(ii-1)) * (vA_2 + vB_2) / 2 * dt;
    theta_2(ii) = theta_2(ii-1) + ((vB_2 - vA_2) / L) * dt;
end

% Plot dan Animasi
figure(1);
plot(x_1, y_1, 'k-', 'DisplayName', 'Robot 1 (Leader)');
hold on;
plot(x_2, y_2, 'r--', 'DisplayName', 'Robot 2 (Follower)');
title('Workspace Robot');
xlabel('x (m)');
ylabel('y (m)');
legend;

% Menyiapkan Grafik Roda dan Pusat Robot
wls = [0 0; -0.5 * L 0.5 * L]; % Posisi relatif roda dalam koordinat lokal

% Robot 1
h1_1 = plot(wls(1,1) + x_1(1), wls(2,1) + y_1(1), 'ro', 'LineWidth', 2, 'MarkerFaceColor', 'r'); % Roda kiri
h2_1 = plot(wls(1,2) + x_1(1), wls(2,2) + y_1(1), 'ro', 'LineWidth', 2, 'MarkerFaceColor', 'r'); % Roda kanan
h3_1 = plot(x_1(1), y_1(1), 'bo', 'MarkerSize', 20); % Pusat robot

% Robot 2
h1_2 = plot(wls(1,1) + x_2(1), wls(2,1) + y_2(1), 'go', 'LineWidth', 2, 'MarkerFaceColor', 'g'); % Roda kiri
h2_2 = plot(wls(1,2) + x_2(1), wls(2,2) + y_2(1), 'go', 'LineWidth', 2, 'MarkerFaceColor', 'g'); % Roda kanan
h3_2 = plot(x_2(1), y_2(1), 'bo', 'MarkerSize', 20); % Pusat robot

axis equal;
axis([-5 5 -5 5]);

% Animasi Pergerakan
for ii = 2:length(t)
    % Rotasi posisi roda sesuai orientasi robot saat ini
    wlsrot_1 = [cos(theta_1(ii)) -sin(theta_1(ii)); sin(theta_1(ii)) cos(theta_1(ii))] * wls;
    wlsrot_2 = [cos(theta_2(ii)) -sin(theta_2(ii)); sin(theta_2(ii)) cos(theta_2(ii))] * wls;
    
    % Update posisi roda dan pusat robot 1 dalam animasi
    set(h1_1, 'XData', wlsrot_1(1,1) + x_1(ii));
    set(h1_1, 'YData', wlsrot_1(2,1) + y_1(ii));
    set(h2_1, 'XData', wlsrot_1(1,2) + x_1(ii));
    set(h2_1, 'YData', wlsrot_1(2,2) + y_1(ii));
    set(h3_1, 'XData', x_1(ii));
    set(h3_1, 'YData', y_1(ii));
    
    % Update posisi roda dan pusat robot 2 dalam animasi
    set(h1_2, 'XData', wlsrot_2(1,1) + x_2(ii));
    set(h1_2, 'YData', wlsrot_2(2,1) + y_2(ii));
    set(h2_2, 'XData', wlsrot_2(1,2) + x_2(ii));
    set(h2_2, 'YData', wlsrot_2(2,2) + y_2(ii));
    set(h3_2, 'XData', x_2(ii));
    set(h3_2, 'YData', y_2(ii));
    
    drawnow;
    pause(0.05);
end
