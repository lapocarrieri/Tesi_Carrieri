function [F, m] = calcolaWrenchConAttrito(triangles, P, J, tau)
    % Calcolo della normale al triangolo contenente P
    [normale, trovato] = trovaNormale(triangles, P);
    if ~trovato
        error('Punto P non trovato in nessun triangolo');
    end

    % Coefficiente di attrito per il cono di attrito
    mu = tan(deg2rad(30));

    % Funzione obiettivo: minimizzare l'errore nel calcolo del wrench
    funzioneObiettivo = @(x) norm(tau - J*x)^2;

    % Vincoli per il cono di attrito
    vincoli = @(x) deal([], [norm(x(1:2)) - mu*dot(x(1:3), normale); -dot(x(1:3), normale)]);

    % Opzioni per l'ottimizzatore
    opzioni = optimoptions('fmincon', 'Algorithm', 'sqp');

    % Risoluzione del problema di ottimizzazione
    x0 = pinv(J) * tau; % Punto di partenza
    soluzione = fmincon(funzioneObiettivo, x0, [], [], [], [], [], [], vincoli, opzioni);

    % Estrazione di F e m dalla soluzione
    F = soluzione(1:3);
    m = soluzione(4:end);
end