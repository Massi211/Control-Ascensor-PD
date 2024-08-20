// Ejemplo de control integral

// Función de transferencia y espacio de simulación
s=%s
t=0:0.01:10
G= 5.93/s/(1+0.221*s)
G=syslin('c',G)
close()
close()
close()
// 
figure(1)
evans(G,2)
plot([0,-5], [0,5],"--")


// Pruebas variando el kp
figure(2)
for k=[0.2,0.25,0.3,0.38,0.5]
    T=(k*G)/(1+k*G)
    Y=csim('step',t,T);
    plot(t,Y);
end

//Respuesta frente a una entrada rampa

rampa=t*5;
for i= 201:1:1001
    rampa(i)=10;
end

figure(3)
plot(t,rampa,"r");
for k=[0.2,0.25,0.3,0.38,0.5]
    T=(k*G)/(1+k*G)
    Y=csim(rampa,t,T);
    plot(t,Y);
end



// Ploteo en rojo el caso sin sobrevalor zeta=1
//I=1/((80/25)*s);
//T=(I*G)/(1+I*G);
//plot(t,csim('step',t,T),'-r');
//denominador=coeff(T.den);
//wn=sqrt(denominador(1));
//zeta=denominador(2)/(2*wn);
//disp(ti, wn, zeta)
