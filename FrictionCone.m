clc;
clear all;
close all;
indixes=1;
           num_part=50;
           Niterations=10;
           load('initialization5.mat','Meshes','triangles','point','link')
           
plottaTriangoliConAttrito(triangles, point');
function plottaTriangoliConAttrito(triangles, P)
    % Numero di triangoli
    numTriangles = size(triangles, 3);

    figure;
    hold on;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Triangoli con Normale e Cono di Attrito');

    % Plot di tutti i triangoli
  for i = 1:numTriangles
        vertici = triangles(:,:,i);
        plot3(vertici(1,:), vertici(2,:), vertici(3,:), 'bo');
    end
    % Calcolo della normale per il triangolo contenente P
    [closestpoint, normale] =  closest_point_to_triangle(triangles, P);


    quiver3(closestpoint(1), closestpoint(2), closestpoint(3), normale(1), normale(2), normale(3), 'r', 'LineWidth', 2);

    disegnaCono(closestpoint', normale, 30, 0.1)
   
   
end

function [F, m] = calcolaWrenchConAttrito(triangles, P, J, tau)
    % Calcolo della normale al triangolo contenente P
    [closestpoint, normale] =  closest_point_to_triangle(triangles, P);


    % Coefficiente di attrito per il cono di attrito
    mu = tan(deg2rad(60));

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

function [normale, trovato] = trovaNormale(triangles, P)
    % Numero di triangoli
    numTriangles = size(triangles, 3);
    trovato = false;
    normale = [0; 0; 0];

    for i = 1:numTriangles
        vertici = triangles(:,:,i);
        % Calcolo della normale per il triangolo
        v1 = vertici(:,2) - vertici(:,1);
        v2 = vertici(:,3) - vertici(:,1);
        normaleCorrente = cross(v1, v2);
        normaleCorrente = normaleCorrente / norm(normaleCorrente);

        % Verifica se P appartiene al triangolo (si può usare un metodo di proiezione o altro)
        if appartieneAlTriangolo(P, vertici)
            normale = normaleCorrente;
            trovato = true;
            break;
        end
    end
end

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
        R = vrrotvec2mat([asseRotazione; angoloRotazione]);
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