clear all;

%Nb individus
N=50;  

%Parametres de chaque individu
Mass=zeros(1,N); %masse
Relax=zeros(1,N); %temps de relaxation
Size=zeros(1,N);  %taille (rayon ri ds le cours)

% valeurs parametres des robots, tous égaux 
Mass(1,:)=0.5;    %en kg
Relax(1,:)=0.1;   %en sec
Size(1,:)=0.05;   %rayon en m
Vrobot=0.3;       %vitesse max d'un robot


%Parametres géométrie de la simulation
%domaine rectangulaire [0,a]x[0,b] 
a=15;
b=15;
c=15;
x=[0:0.1:a];
y=[0:0.1:b];
z=[0:0.1:c];


% Paramètres temps de simulation
Tfinal=30;
dt=0.08;


%Vecteurs et matrices utilisés (dimension 3 eventuellement)
Vn=zeros(3,N); %vitesses au temps tn
Vn1=zeros(3,N); %vitesses au temps tn+1
Vexpec=zeros(3,N); %vitesses désirées au temps tn

Xn=zeros(3,N); %positions au temps tn
Xn1=zeros(3,N); %positions au temps tn+1

Force_others=zeros(3,N); %vecteur des forces pour les interactions avec le PFD
Force_individuel=zeros(3,N); %vecteur des forces pour la force locomotrice


% on stock le nombre d'instants dt et de moyennes que l'on veut
nb_data_temps  = floor(Tfinal/dt);
moyenne =  zeros(1,nb_data_temps);
temps = zeros(1,nb_data_temps);


%Initialisation
Xn(1,:)= a*rand(1,N); 
Xn(2,:)=b*rand(1,N);
Xn(3,:)=c*rand(1,N);

t=0;
cont=1;

%parametres pour la force attraction repulsion : alpha pour le coeff
%d'anisometrie
alpha = 1;

%parametres pour la force locomotrice 
k=2;
Vm=0.5;


 %%%%%%%%%% Boucle en temps %%%%%%%%%%%%%%%
while (t<Tfinal)

   %centre de masse
   G=sum(Xn')'/N; 
   

  %%%%%%%%%% Calcul forces interaction avec les autres %%%%%%%%%%
  for i=1:N
    %calcul des forces s'execant sur le poisson i : interaction avec les autres
   

    Force_others(:,i) = Interactions_poissons(i,Xn,Vn,G,Size,N, alpha);
    Force_individuel(:,i) = Force_Locomotrice(i, Vn, k, Vm);
    
  end

 %Itération vitesses, schéma d'Euler explicite
 
   Vn1=Vn + dt * Force_others./Mass +dt*Force_individuel./Mass ; % application de la fonction calcul de vitesse sur la vitesse : PFD
  
%encadrement de la vitesse, juste pour des questions de stabilité à
%l'initalisation
   Vn1=min(2*Vrobot,Vn1); Vn1=max(-2*Vrobot,Vn1);

   %calcul des nouvelles positions

   Xn1=Xn + dt*Vn1;

   %%%%%%%   Tracé de la position des robots au temps t %%%%%
    %%%%%%%   toutes les nbiter itérations   %%%%%%
  nbiter=2; %on trace toutes les deux iterations
  
  if mod(cont,nbiter)==0 % division entieres : si c'est pair on trace
    figure(1);
    clf();
    hold on;

% Verify the size of Xn1
        if size(Xn1, 1) ~= 3
            error('Xn1 should have 3 rows for x, y, and z coordinates.');
        end

       % Plot the data in 3D
    plot3(Xn1(1,:), Xn1(2,:), Xn1(3,:), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');

    % Set the axes to be equal
    axis equal;

    % Set the domain limits
    xlim([0 a]);
    ylim([0 b]);
    zlim([0 c]);

    % Adjust the view to make sure the 3D aspect is visible
    view(3);  % Set the default 3D view
    grid on;  % Add grid for better visualization

    hold off;
    drawnow limitrate nocallbacks;

  end %fin end du if du tracé
% fin du tracé

   %%%%% mise a jour des variables pour itération suivante %%%%
Vn=Vn1;
Xn=Xn1;
t=t+dt;
temps(cont)=t;
cont=cont+1;
end
