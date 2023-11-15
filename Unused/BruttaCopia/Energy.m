            masses = load("models\masses.txt");
            CoMs = load("models\center_of_masses.txt");
            qq = [0 0 0 0 0 0 0];
            g=9.81;
            energy=0;
            for i=1:7
                T=QtoP(qq,i);
                c(i,:) = T*[CoMs(i,:) 1]';
                energy = energy + masses(i)*g*c(i,3);

            end
            vpa(c,2)
            energy