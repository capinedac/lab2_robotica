%%
%rosinit; %Conexión con nodo maestro
%%
velServ = rossvcclient("/turtle1/teleport_absolute","turtlesim/TeleportAbsolute"); %Creación servicio
velMsg = rosmessage(velServ); %Creación de mensaje
%%
%Valores
velMsg.X = 10;
velMsg.Y = 10;
velMsg.Theta = 3.14/3;

call(velServ,velMsg); %Envio con el uso de call
pause(1)

%%
rosshutdown;  %Finalizar el nodo maestro desde Matlab