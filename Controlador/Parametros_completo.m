%******************************************
%Parametros del modelo físico
%******************************************

%Izaje

%Altura de la columna 
yt0=40;%[m]

m0=15000;%              Masa de guinche
ml=50000;%              Masa de la carga
mL=m0+ml;%              Masa total de carga
rd=0.75;%[m]            Radio Tambor
bh=1e-5;%               Amortiguamiento del tambor
g=9.8066;%[m/s²]        Aceleracion de gravedad
ni=30;%                 Relacion de transmicion
Jmotor=30;%[kg*m²]      Inercia del motor + freno
Jtambor=8.0;%[kg*m²]    Inercia del tambor
%Mh=Jtambor/(rd^2);%[kg]  Masa del tambor
Kw=mL*g/yt0;%5e3;%5e6;%[N/m]           Constante de elasticidad del cable
bw=0;%2750;%                Constante de amortiguamiento de la elasticidad

%******************************************
%Parametros derivados sistema de izaje
%******************************************

%Inercia de la masa total
Jlt=mL*rd^2;%            Inercia en el eje de la polea
%Inercia sistema carga polea despues del reductor
Jlp=(Jtambor+Jlt)/(ni^2);

%******************************************
%Parametros referidos al eje del motor
%******************************************
%Inercia del motor + sistema
Jmi=Jlp+Jmotor;
%Amortiguamiento del motor
bmi=1.5e-5;
Mh=Jmi/rd^2;

%Traslacion

Mc=50000;%[kg]          Masa del carro + sistema de izaje
Mt=Mc+ml;%[kg]          Mc + carga
rw=0.5;%[m]             Radio de la rueda
Jw=2;%[kg*m²]           Momento de inercia de la rueda
bt=1e-5;%
nt=15;%                 Relacion de transmición
Jmotort=10;%[kg*m²]     Momento de inercia del motor + freno

%******************************************
%Parametros referidos al eje del motor
%******************************************

%Inercia del motor + sistema
Jmt=(Jw/(nt^2))+Jmotort;
%Amortiguamiento del motor
bmt=1.5e-5;

%******************************************
%Parametros del sistema de control
%******************************************

%traslacion

E=0.5;
wn=15;%rad/s
p1x=-15;%polo 1 rad/s

%Polo a lazo abierto
p2x=-bmt/Jmt;

%Ganancia derivativa
kdx=(2*E*wn-p1x)*Mt-bmt;
%Ganancia proporcional
kpx= (wn^2-2*E*wn*p1x)*Mt;
%Ganancia Integral
kix=(-p1x*wn^2*Mt);


%izaje

E= 5;
wn=15;%rad/s
p1y=-10;%polo 1 rad/s 

%Polo a lazo abierto
p2y=-bmi/Jmi;

%Ganancia derivativa
kdy=(2*E*wn-p1y)*Mh-bmi;
%Ganancia proporcional
kpy= (wn^2-2*E*wn*p1y)*Mh;
%Ganancia Integral
kiy= (-p1y*wn^2*Mh);


%Parametros de PID discreto

Ts=(1/wn)/10;

%******************************************
%Parametros de la maquina de estado
%******************************************

%Altura de la columna 
yt0=40;%[m]

%Posición inicial
p0x = 0.0;
p0y = 0.0;

%Máxima altura
phy = 39;

%Posición final
pfx= 30.0;
pfy= 0.0;

%Multi port switch
Avanzar = 1;
Retroceder = 2;
Detener = 3;

%Condiciones iniciales

xt0 = 0;
v_xt0 = 0;

xl0 = 0;
v_xl0 = 0;

yl0 = 0;
v_yl0 = 0;

lh0 = hypot((xl0 - xt0),(yt0 - yl0)) - ((mL*g)/Kw);
v_lh0 = 0;

Fh0 = mL*g;

%******************************************
%Parametros del sistema selector
%******************************************

%Consignas del motor
Cons_Av = 1;        %Velocidad de Avance
Cons_Re = -1;       %Velocidad de Retroceso
Cons_De = 0;