%% Configuração da serial
clear s
porta = "COM3";
baudrate = 115200;

s = serialport(porta, baudrate);
configureTerminator(s,"LF");

setDTR(s,false);
setRTS(s,false);

flush(s);

disp("✅ Coletor em loop iniciado.");
disp("Aguardando BEGIN_DATA...");

contador = 0;

while true   % LOOP PRINCIPAL
    %% Espera BEGIN_DATA
    while true
        if s.NumBytesAvailable == 0
            pause(0.01);
            continue;
        end

        linha = strtrim(readline(s));

        if strcmp(linha,"BEGIN_DATA")
            contador = contador + 1;
            fprintf("\n📥 Nova leitura #%d detectada\n", contador);
            break;
        end
    end

    %% Coleta os dados
    dados = [];

    while true
        if s.NumBytesAvailable == 0
            pause(0.001);
            continue;
        end

        linha = strtrim(readline(s));

        if strcmp(linha,"END_DATA")
            disp("📤 Fim da leitura");
            break;
        end

        valores = sscanf(linha,'%f,%f,%f');
        if numel(valores) == 3
            dados(end+1,:) = valores.'; %#ok<SAGROW>
        end
    end

    %% Salva automaticamente
    if ~isempty(dados)
        freq = dados(:,1);
        mag  = dados(:,2);
        fase = dados(:,3);

        timestamp = datestr(now,'yyyy-mm-dd_HH-MM-SS');

        nomeBase = sprintf("leitura_%03d_%s", contador, timestamp);

        writematrix(dados, nomeBase + ".csv");
        save(nomeBase + ".mat", "freq", "mag", "fase");

        fprintf("✅ Leitura #%d salva (%d pontos)\n", contador, size(dados,1));
    else
        warning("Leitura vazia ignorada.");
    end

    disp("Aguardando próxima leitura...");
end