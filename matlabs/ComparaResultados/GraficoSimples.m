%% ==============================
%  Protótipo – Magnitude e Fase
%  Circuito RLC Série
% ===============================
clc; clear; close all;

%% -------- Dados do protótipo (TCC) --------
arquivo = ...
"C:\Users\Wellytom\Documents\GitHub\TCC\matlabs\ColetaDados\leitura_002_2026-03-20_17-50-21";

dados = readmatrix(arquivo);

freq   = dados(:,1);   % Hz
mag_p  = dados(:,2);   % Ohms
fase_p = dados(:,3);   % rad

% Converte fase para graus
fase_p = fase_p * 180/pi;

%% -------- Gráfico combinado com dois eixos Y --------
figure('Name','Protótipo – Magnitude e Fase');
hold on;

% Cor do protótipo
cor_proto = [1 0 0];   % vermelho

set(gca,'XScale','log')
xlim([min(freq) max(freq)])
grid on;

%% ----- Magnitude (eixo Y esquerdo) -----
yyaxis left
set(gca,'YColor','k')

plot(freq, mag_p, '-', ...
    'Color', cor_proto, ...
    'LineWidth', 2);

ylabel('|Z| (\Omega)');

%% ----- Fase (eixo Y direito) -----
yyaxis right
set(gca,'YColor','k')

plot(freq, fase_p, ':', ...
    'Color', cor_proto, ...
    'LineWidth', 2);

ylabel('Fase (graus)');

%% ----- Ajustes finais -----
xlabel('Frequência (Hz)');
% title('Resposta em Frequência do Protótipo');

legend( ...
'|Z| – Protótipo', ...
'Fase – Protótipo', ...
'Location','best');