% Modelisation interactions de robots. 
%Programme principal à completer pour le TP 
%G. Chiavassa 2021

clear all;

%Nb individus
N=100;  

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
b=5;
x=[0:0.1:a];
y=[0:0.1:b];

% Paramètres temps de simulation
Tfinal=60;
dt=0.08;


%Vecteurs et matrices utilisés (dimension 3 eventuellement)
Vn=zeros(3,N); %vitesses au temps tn
Vn1=zeros(3,N); %vitesses au temps tn+1
Vexpec=zeros(3,N); %vitesses désirées au temps tn

Xn=zeros(3,N); %positions au temps tn
Xn1=zeros(3,N); %positions au temps tn+1

Force_others=zeros(3,N);


source=zeros(3,1);

%Initialisation
Xn(1,:)= a*rand(1,N)/6; 
Xn(2,:)=b*rand(1,N);

t=0;
cont=1;

          %%%%%%%%%% Boucle en temps %%%%%%%%%%%%%%%
while (t<Tfinal)

   %centre de masse
   G=sum(Xn')'/N; 
    
  %%%%%%%%%% Calcul forces interaction avec les autres %%%%%%%%%%
  for i=1:N
    %calcul des forces s'execant sur robot i
    Force_others(:,i)=Interaction_robots(i,Xn,Vn,G,Size,N);
    
    %Vitesse expected du robot i
    Vexpec(:,i)=Vexpected_robots(i,Xn,Vn,source,Vrobot);

  end

 %Itération vitesses, schéma d'Euler explicite
 
   Vn1=Vn + dt*(Vexpec-Vn)./Relax + dt * Force_others./Mass;
  
%encadrement de la vitesse, juste pour des questions de stabilité à
%l'initalisation
   Vn1=min(2*Vrobot,Vn1); Vn1=max(-2*Vrobot,Vn1);
  
%calcul des nouvelles positions

   Xn1=Xn + dt*Vn1;

%%%%%   calcul de differentes caractéristiques du mouvement

%2) Norme vitesse moyenne:  A FAIRE
m=0;
for i=1:N
    m=m+norm(Vn1(:,i));
end
moyenne(cont)=m/N;


%3) Modification de la position de la source au cours du temps:  A FAIRE
source(1,1)=a-5;
source(2,1)=b*t/50;
 
%5) Distance moyenne entre les robots:  A FAIRE




    %%%%%%%   Tracé de la position des robots au temps t %%%%%
    %%%%%%%   toutes les nbiter itérations   %%%%%%
  nbiter=2;
  
  if mod(cont,nbiter)==0 
    figure(1)
    clf();
    hold on
   %tout les robots de la meme couleur
    plot(Xn1(1,:),Xn1(2,:),'bo','MarkerSize',5,'MarkerFaceColor','b');

   %robot 1 en rouge si leader
   %plot(Xn1(1,1),Xn1(2,1),'ro','MarkerSize',5,'MarkerFaceColor','r');

   %tracé de la source si elle existe
   plot(source(1,1),source(2,1),'gd','MarkerSize',8,'MarkerFaceColor','g');

   axis equal

   %Soit dans le domaine [0,a]x[0,b]
   xlim([0 a]);
   ylim([0 b]);
   plot([0,a],[0,0],'k','LineWidth',2);
   plot([0,a],[b,b],'k','LineWidth',2);
   txt=['t = ',num2str(t)];
   text(a/2,b+b/20,txt,'Fontsize',12);

   %%soit autour du centre de masse
   % ddx=max(Xn1(1,:))-min(Xn1(1,:));
   % ddy=max(Xn1(2,:))-min(Xn1(2,:));
   % xlim([G(1,1)-ddx G(1,1)+ddx]);
   % ylim([G(2,1)-ddy G(2,1)+ddy]);
   % txt=['t = ',num2str(t)];
   % text(G(1,1),G(2,1)+ddy,txt,'Fontsize',12);

   hold off
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

%%figure variation vitesse moyenne en fct du temps
  % A FAIRE
  figure(2)
  plot(temps, moyenne);
  xlabel('time sec');
  ylabel('vitesse moyenne m/sec');
    
  
  
%%figure variation distance minimale moyenne en fct du temps
  % A FAIRE
xdisque=[a/2;b/2;0];
Rdisque=b/6;
th=[0:0.04:2*pi];
xd=xdisque(1)+Rdisque*cos(th);
yd=xdisque(2)+Rdisque*sin(th);
plot(xd,yd,'k','linewidth',8);
