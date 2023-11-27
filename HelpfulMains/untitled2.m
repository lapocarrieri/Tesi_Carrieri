
% Esempio di utilizzo
punto = [1; 2; 3]; % Punto di origine del cono
direzione = [1; 0; 0]; % Direzione della semiretta
lunghezza = 5; % Lunghezza della semiretta
angolo = 30; % Angolo del cono in gradi

figure;
disegnaCono(punto, direzione, angolo, lunghezza);

function disegnaCono(punto, direzione, angolo, lunghezza)
    % Calcola il raggio del cono
    angoloRad = deg2rad(angolo);
    raggio = lunghezza * tan(angoloRad);

    % Calcola i punti della base del cono
    [X, Y, Z] = cylinder([0 raggio], 100);
    Z = Z * lunghezza; % Scala l'altezza del cono

    % Ruota il cono per allinearlo con la direzione
    direzione = direzione / norm(direzione); % Normalizza la direzione
    asseZ = [0; 0; 1]; % Asse Z standard
    asseRotazione = cross(asseZ, direzione');
    angoloRotazione = acos(dot(asseZ, direzione'));

    % Crea la matrice di rotazione
    if norm(asseRotazione) ~= 0
        asseRotazione = asseRotazione / norm(asseRotazione);
        rotVec = [asseRotazione', angoloRotazione];
        R = vrrotvec2mat(rotVec);
    else
        R = eye(3);
    end

    % Applica la rotazione ai punti del cono
    for i = 1:size(X, 2)
        puntoRotato = R * [X(2, i); Y(2, i); Z(2, i)];
        X(2, i) = puntoRotato(1);
        Y(2, i) = puntoRotato(2);
        Z(2, i) = puntoRotato(3);
    end

    % Traslazione del cono per posizionarlo nel punto
    X = X + punto(1);
    Y = Y + punto(2);
    Z = Z + punto(3) - lunghezza; % Sottrai la lunghezza per posizionare il vertice in 'punto'

    % Disegna il cono
    surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Cono in 3D');
end

