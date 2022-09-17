%%
%rosinit; %Conexión con nodo maestro
%%
velSub = rossubscriber("/turtle1/pose","turtlesim/Pose"); %Creacion suscriptor
velMsg = velSub.LatestMessage; %lectura del mensaje
%%
%Lectura de atributos de velMsg
X = velMsg.X
Y = velMsg.Y
thetha = velMsg.Theta
velLin = velMsg.LinearVelocity
velAng = velMsg.AngularVelocity
pause(1)