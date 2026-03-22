%% ==============================
% Comparação: Protótipo x Teórico x Referência
% Circuito RLC Série
% ===============================
clc; clear; close all;

%% -------- Parâmetros do circuito (modelo teórico ideal) --------
R = 000;
L = 220e-6;
C = 47e-9;

%% -------- Dados do protótipo (TCC) --------
arquivo = ...
"C:\Users\Wellytom\Documents\GitHub\TCC\matlabs\ComparaResultados\leitura_009_220uH_47nF.csv";
dados = readmatrix(arquivo);
freq = dados(:,1); % Hz
mag_p = dados(:,2); % Ohms
fase_p = dados(:,3); % graus
fase_p = fase_p * 180/pi; % graus

%% -------- Modelo teórico --------
omega = 2*pi*freq;
Z_teo = R + 1j*(omega*L - 1./(omega*C));
mag_teo = abs(Z_teo);
fase_teo = angle(Z_teo) * 180/pi;

%% -------- Gráfico combinado com dois eixos Y --------
figure();
hold on;

% ----- Cores -----
cor_proto = [1 0 0];      % vermelho
cor_teo   = [1 0.8 0];    % amarelo
cor_ref   = [0 0 1];      % azul

set(gca,'XScale','log')
grid on;

%% ----- Magnitude (eixo Y esquerdo) -----
yyaxis left
set(gca,'YColor','k')     % <<< FORÇA EIXO PRETO

plot(freq, mag_p,   '-', 'Color',cor_proto, 'LineWidth',2); hold on;
plot(freq, mag_teo, '-', 'Color',cor_teo,   'LineWidth',2);

ylabel('|Z| (\Omega)');

%% ----- Fase (eixo Y direito) -----
yyaxis right
set(gca,'YColor','k')     % <<< FORÇA EIXO PRETO

plot(freq, fase_p,   ':', 'Color',cor_proto, 'LineWidth',2);
plot(freq, fase_teo, ':', 'Color',cor_teo,   'LineWidth',2);

ylabel('Fase (graus)');

%% ----- Ajustes finais -----
xlabel('Frequência (Hz)');
%title('Comparação Geral – Magnitude (linha contínua) e Fase (pontilhada)');

legend( ...
'|Z| – Protótipo', ...
'|Z| – Teórico', ...
'Fase – Protótipo', ...
'Fase – Teórico', ...
'Location','best');