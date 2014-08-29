
%******************************************
%Parametros del modelo físico
%******************************************

%Profundidad del barco
prof_barco=20;%[m]
%Altura del Container
hContainer = 5;
%Ancho del container
wContainer = 5;

%Limites de Velocidad/Aceleracio 
v_lh_max = 1.5;
v_lh_min = -1.5;
a_lh_min = -1;
a_lh_max = 1;

v_xt_max = 1.5;
v_xt_min = -1.5;
a_xt_min = -1;
a_xt_max = 1;

%Condiciones Iniciales
yl0 = hContainer + 1;


%v_yl0 = 0;

%lh0 = hypot((xl0 - xt0),(yt0 - yl0)) - ((mL*g)/Kw);
%v_lh0 = 0;




%******************************************
%Izage
%******************************************

%Altura de la columna 
yt0=45;%[m]
g=9.8066;%[m/s²]        Aceleracion de gravedad

m0=15000;%              Masa de guinche
ml=50000;%              Masa de la carga
mL=m0+ml;%              Masa total de carga

Fh0 = mL*g;%            Fuerza de precarga

Rd=0.75;%[m]            Radio Tambor

Jm=30;%[kg*m²]          Momento de Inercia del motor + freno
Jd=8.0;%[kg*m²]         Momento de Inercia del tambor
ni=30;%                 Relacion de transmicion

beqi= 18; %Nm/rad/s Perdidas equivalentes para el eje rapido

Kw = 180e3;% N/m Constante de elasticidad del cable
bw = 3e3;% N/m/s Constante de friccion producida en el estiramiento del cable

Mh = (Jd + Jm*ni^2)/Rd^2;
bh = (beqi*ni^2)/Rd^2;


%*****************************************
%Traslacion
%*****************************************
Mc=50000;%[kg]          Masa del carro + sistema de izaje

%Sin la carga!!
Mto=Mc + ml;%[kg]          Mc + carga

Rw=0.5;%[m]             Radio de la rueda

Jw=2;%[kg*m²]           Momento de inercia de la rueda
Jmotort=10;%[kg*m²]     Momento de inercia del motor + freno del sistema de traslacion
nt=15;%                 Relacion de transmición

beqt = 30; %[Nm/rad/s]  Perdidas del sistema equivalentes expresadas en el eje rapido

Mt = (Mc + (Jw + Jmotort*nt^2)/Rw^2) ; %Masa total equivalente del sistema en traslacion
bt = (beqt*nt^2)/Rw^2;


%******************************************
%Parametros del sistema de control
%******************************************

%traslacion

E= 0.5;
wn= 1;%rad/s
p1x= -1;%polo 1 rad/s

%Ganancia derivativa
kdx=(2*E*wn-p1x)*Mt-bt;
%Ganancia proporcional
kpx= (wn^2-2*E*wn*p1x)*Mt;
%Ganancia Integral
kix=(-p1x*wn^2*Mt);


%izaje

E= -0.499;
wn = 50;%1500;%rad/s
p1y= -50;%-1499;%polo 1 rad/s 

%Ganancia derivativa
kdy=(2*E*wn-p1y)*Mh-bh + 9e4;%2300
%Ganancia proporcional
kpy=(wn^2-2*E*wn*p1y)*Mh;%240000
%Ganancia Integral
kiy=(-p1y*wn^2*Mh)/1e5;%32000


%Parametros de PID discreto

Ts=(1/wn)/10;

%******************************************
%Parametros de la maquina de estado
%******************************************


%Numero Columna
numColumna = 5;
%Numero de Containers en 5 columnas
numContainer = [0 0 0 0 0];

%Altura de la columna 
yt0 = 45;%[m]
%Altura maxima de la carga 
yl_max = 40;%[m]
%Altura minima de la carga
yl_min = prof_barco - 1.3;

% HOME
xt_h = -15.0;
lh_h = yt0-yl0-mL*g/Kw;  %Largo del cable desenrrollado
lh_min = yt0 - yl_max-mL*g/Kw;

%GOAL
xt_g= (numColumna + 0.5) * wContainer;
lh_g= yt0 + yl_min -  (hContainer*numContainer(numColumna)) -mL*g/Kw;

%Margenes de Error
marg_xt=5;
marg_xt_g=10;

marg_lh=5;
marg_lh_g=10;




%******************************************
%Parametros del sistema selector
%******************************************

%Consignas del motor
Cons_Av = 1;        %Velocidad de Avance
Cons_Re = -1;       %Velocidad de Retroceso
Cons_De = 0;

l = 40 + (mL*g)/Kw;
