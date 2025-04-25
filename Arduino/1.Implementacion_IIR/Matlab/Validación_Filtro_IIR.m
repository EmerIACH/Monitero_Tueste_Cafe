% -----------------------------------------------------
% ** Código MATLAB para validar el filtro IIR **
% -----------------------------------------------------

% Coeficientes del filtro (SOS y ganancia)
SOS = [1.0000, 0, -1.0000 , 1.0000, -0.1998,-0.4142];
G = [0.7071, 1.0000];

% Convertir SOS a coeficientes [b,a] con ganancia
[b, a] = sos2tf(SOS, G); % Resultado esperado: b = [0.5658, 0, -0.5658], a = [1, 0, -0.1317]

% Parámetros de simulación
fs = 48000;          % Frecuencia de muestreo (igual que en el ESP32)
t = 0:1/fs:0.01;     % 10 ms de señal

% -----------------------------------------------------
% ** 1. Generar señal de prueba **
% -----------------------------------------------------
f1 = 1000;   % Frecuencia en banda de rechazo (1 kHz)
f2 = 16000;   % Frecuencia en banda de paso (8 kHz)
x = 0.5*sin(2*pi*f1*t) + 0.5*sin(2*pi*f2*t) + 0.2*randn(size(t)); % Mezcla + ruido

% -----------------------------------------------------
% ** 2. Aplicar filtro **
% -----------------------------------------------------
y = filter(b, a, x);

% -----------------------------------------------------
% ** 3. Gráficas para análisis **
% -----------------------------------------------------
figure;

% Señal original vs filtrada (dominio del tiempo)
subplot(2,1,1);
plot(t, x, 'b', t, y, 'r');
xlabel('Tiempo (s)');
ylabel('Amplitud');
legend('Entrada', 'Salida');
title('Comparación temporal');
grid on;

% Respuesta en frecuencia (FFT)
subplot(2,1,2);
N = 128;
f = linspace(0, fs/2, N/2+1);
X = abs(fft(x, N));
Y = abs(fft(y, N));
plot(f, X(1:N/2+1), 'b', f, Y(1:N/2+1), 'r');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
legend('Entrada', 'Salida');
title('Espectro frecuencial');
grid on;

% -----------------------------------------------------
% ** 4. Respuesta en frecuencia del filtro **
% -----------------------------------------------------
figure;
freqz(b, a, 1024, fs);
title('Respuesta del filtro diseñado');