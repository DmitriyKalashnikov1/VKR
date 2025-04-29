% программа моделирования движения робота при ручном управлении движением в
% плоскости
% параметры: mode -- режим движения, 0 -- прямолинейное, 1 -- движение с
% поворотом, 2 -- поворот на месте
function robotInHandControlMode(mode)
startTime = 0;
stopTime = 30;
dt = 0.01;
time = startTime:dt:stopTime;

%массивы под углы джойстика
alpha = zeros(length(time),1);
beta = zeros(length(time),1);

 
%генерация данных с джойстика при прямолинейном движении 
if (mode == 0)
    for tf = 1:length(time)
        t = tf*dt;
        alphaInc = pi/2000;
        
        if (t < 5)
            alpha(tf,1) = tf*alphaInc;
        end
        if ((t >= 5)&&(t<10))
            alpha(tf,1) = pi/2;
        end
        if ((t >= 10)&&(t <= 30))
            alpha(tf,1) = (length(time) - tf)*alphaInc;
        end
        if ((t > 30))
            alpha(tf,1) = 0;
        end
    end
end

%генерация данных с джойстика при движении с поворотом 
if (mode == 1)
    for tf = 1:length(time)
        t = tf*dt;
        alphaInc = pi/2000;
        betaDec = -pi/1000;
        
        if (t < 5)
            alpha(tf,1) = tf*alphaInc;
            beta(tf,1) = tf*betaDec;
        end
        if ((t >= 5)&&(t<10))
            alpha(tf,1) = pi/4;
            beta(tf,1) = -pi/2;
        end
        if ((t >= 10)&&(t <= 15))
            alpha(tf,1) = (length(time) - tf)*alphaInc;
            beta(tf,1) = (length(time) - tf)*betaDec;
        end
        if ((t > 15))
            alpha(tf,1) = 0;
            beta(tf,1) = 0;
        end
    end
end

%генерация данных с джойстика на повороте 
if (mode == 2)
    for tf = 1:length(time)
        t = tf*dt;

        betaDec = -pi/1000;
        
        if (t < 5)
            beta(tf,1) = tf*betaDec;
        end
        if ((t >= 5)&&(t<10))
            
            beta(tf,1) = -pi/2;
        end
        if (t >= 10)&&(t <= 20)
            beta(tf,1) = (length(time) - tf)*betaDec;
        end
        if ((t > 20))
            beta(tf,1) = 0;
        end
    end
end


% параметры робота и симуляции
 r = 0.158; % радиус колеса
 lw = 0.0705; % ширина колеса
 mk = 2;       % масса колеса
 J0 = (mk*r^2)/2; %инерция колеса

 m = 135.5; % масса робота 
 lx = (0.787 - lw);  % длина оси робота
 ly = 0.5*1.3;
 % Матрица положения центров колес
J = (1/4) * [1, 1, 1, 1; 
             -1, 1, 1, -1; 
             -1/(lx + ly), 1/(lx + ly), -1/(lx + ly), 1/(lx + ly)];

 alphaGround = 0;  % угол наклона поверхности
 deltaTr = 0.011; % коеффициент трения качения резины по сухому асфальту
 g = 9.8; %константа свободного падения
 Pmax = 763; % номинальная мощность приводов
 h = 0.05; % высота рукоятки джойстика
 joy_c = 0.1; % коеффициент увеличения для джойстика
 % Угловые скорости колес (инициализация)
 omega_start = [0; 0; 0; 0]; % Начальные угловые скорости колес
 omega_end = [2; 5; 2; 5]; % Конечные угловые скорости колес
 t_accel = 4; % Время разгона до конечной скорости (с)

 % Генерация коэффициентов проскальзывания (от 64% до 100%)
 slip_factors = 0.82 + (1 - 0.82) * rand(length(time), 2); % Для каждого времени и стороны робота
 
 % Масивы для хранения результатов симуляции
 x_target = zeros(length(time),1); 
 y_target = zeros(length(time),1); 
 teta_target = zeros(length(time),1); 
 x_real = zeros(length(time),1); 
 y_real = zeros(length(time),1); 
 teta_real = zeros(length(time),1); 
 
 wl_history = zeros(length(time),1); 
 wr_history = zeros(length(time),1);
 
 vl_history = zeros(length(time),1); 
 vr_history = zeros(length(time),1);
 Mr_history = zeros(length(time),1); 
 Ml_history = zeros(length(time),1); 

 omega_history = zeros(length(time), 4); % История угловых скоростей
 slip_history = zeros(length(time), 4); % История коэффициентов проскальзывания



% Дополнительные переменные
 slip_x = zeros(4, 1); % Скорость скольжения по оси X
 slip_y = zeros(4, 1); % Скорость скольжения по оси Y
 V_wheels_contact = zeros(4, 1); % Скорости в точках контакта

 
 joystick_pos_a = zeros(length(time),1);
 joystick_pos_b = zeros(length(time),1);
 
 maxW = 20;
 
 
 % симуляция
 %teta_target(1,1) = 2*pi;
 for tf = 2:length(time)
     slip_r = slip_factors(tf,2);
     slip_l = slip_factors(tf,1);
     % положение джойстика
     joystick_pos_a(tf, 1) = pi/2 + alpha(tf, 1);
     joystick_pos_b(tf, 1) = pi/2 + beta(tf, 1);
     
     % расчет желаемой траектории
     posVect = joystick(alpha(tf,1), beta(tf,1), h, x_target(tf-1, 1), y_target(tf-1, 1), teta_target(tf-1, 1), joy_c);
     
     x_target(tf, 1) = posVect.xN;
     y_target(tf, 1) = posVect.yN;
     teta_target(tf, 1) = posVect.tetaN;

     vx = 0.01*(x_target(tf, 1) - x_target(tf - 1, 1))/dt;
     vy = 0.01*(y_target(tf, 1) - y_target(tf - 1, 1))/dt; 
     
     v = sqrt(vx^2 + vy^2);
     omega = (teta_target(tf, 1) - teta_target(tf - 1, 1))/dt;
     
     wl_k = (1/r)*(v - (omega*lx)/2);
     wr_k = (1/r)*(v + (omega*lx)/2);
     
     if (abs(wl_k) > maxW)
         wl_history(tf, 1) = maxW*sign(wl_k);
     else
         wl_history(tf, 1) = wl_k;
     end
     
     if (abs(wr_k) > maxW)
         wr_history(tf, 1) = maxW*sign(wr_k);
     else
         wr_history(tf, 1) = wr_k;
     end
     
     vr_history(tf, 1) = r*wr_k*slip_r;
     vl_history(tf, 1) = r*wl_k*slip_l;
     
     Mr_history(tf, 1) = 0.5*m*g*cos(alphaGround)*deltaTr + 0.5*m*((vr_history(tf,1) - vr_history(tf -1,1))/dt + g*sin(alphaGround))*r - (J0/dt) - ((vr_history(tf,1)/r) - (vr_history(tf - 1,1)/r));
      
     Ml_history(tf, 1) = 0.5*m*g*cos(alphaGround)*deltaTr + 0.5*m*((vl_history(tf,1) - vl_history(tf -1,1))/dt + g*sin(alphaGround))*r - (J0/dt) - ((vl_history(tf,1)/r) - (vl_history(tf - 1,1)/r));
      
      
     
     x_real(tf, 1) = x_real(tf - 1, 1) + (r/2)*(wl_history(tf, 1) + wr_history(tf, 1))*cos(omega);
     y_real(tf, 1) = y_real(tf - 1, 1) + (r/2)*(wl_history(tf, 1) + wr_history(tf, 1))*sin(omega);
     teta_real(tf, 1) = teta_real(tf - 1, 1) + omega;
%      t = tf*dt;
%      
%        % Вычисление текущих угловых скоростей колес
%      omega = gradual_acceleration(t, omega_start, omega_end, t_accel);
% 
%      
%          % 1. Вычисление необходимого направления
%     delta_x = x_target(tf, 1) - x_real(tf, 1);
%     delta_y = y_target(tf, 1) - y_real(tf, 1);
%     distance = sqrt(delta_x^2 + delta_y^2); % Расстояние до цели
%     angle_to_target = atan2(delta_y, delta_x); % Угол к целевой точке
%     
%     % 2. Вычисление необходимого угла поворота
%     delta_angle = teta_target(tf, 1) - angle_to_target; % Разница углов
%     if abs(delta_angle) > pi
%         delta_angle = delta_angle - 2*pi*sign(delta_angle); % Оператор для перехода через 0
%     end
%     
%     % 3. Линейная и угловая скорости
%     linear_velocity = min(1, distance); % Линейная скорость (ограничиваем величину)
%     angular_velocity = min(1, delta_angle); % Угловая скорость (ограничиваем)
% 
%     % 4. Определение угловых скоростей колес через кинематическую модель
%     % Для получения угловых скоростей используем линейную и угловую скорости
%     % Для этого будем использовать следующее преобразование:
%     % omega_1 = (Vx + omega_psi * Lx) / r
%     % omega_2 = (Vx - omega_psi * Lx) / r
%     % omega_3 = (Vy + omega_psi * Ly) / r
%     % omega_4 = (Vy - omega_psi * Ly) / r
% 
%     % 4. Определение угловых скоростей колес через кинематическую модель
% % Расчет угловых скоростей для колес
% omega_1 = (linear_velocity + angular_velocity * lx) / r;
% omega_2 = (linear_velocity - angular_velocity * lx) / r;
% omega_3 = (linear_velocity + angular_velocity * ly) / r;
% omega_4 = (linear_velocity - angular_velocity * ly) / r;
% 
% % Учет проскальзывания
% omega_slip_1 = omega_1 * slip_factors(tf, 1);
% omega_slip_2 = omega_2 * slip_factors(tf, 2);
% omega_slip_3 = omega_3 * slip_factors(tf, 3);
% omega_slip_4 = omega_4 * slip_factors(tf, 4);
% 
% % Сохранение угловых скоростей для графиков
% omega_history(tf, :) = [omega_slip_1, omega_slip_2, omega_slip_3, omega_slip_4];
% 
% % Учет скоростей в точке контакта с землей
% V_wheel_1 = r * omega_slip_1;
% V_wheel_2 = r * omega_slip_2;
% V_wheel_3 = r * omega_slip_3;
% V_wheel_4 = r * omega_slip_4;
% 
% % Скорость в точке контакта колес
% V_wheels_contact = [V_wheel_1; V_wheel_2; V_wheel_3; V_wheel_4];
% 
% 
% V_robot = J * V_wheels_contact;
% Vx = V_robot(1);
% Vy = V_robot(2);
% omega_psi = V_robot(3);
% 
% % Коррекция скорости робота (аналогично предыдущему коду)
% slip_x = V_wheels_contact - Vx * cos(teta_real(tf,1)) - Vy * sin(teta_real(tf,1));
% slip_y = -Vx * sin(teta_real(tf,1)) + Vy * cos(teta_real(tf,1));
% correction_factor = 1 - (norm(slip_x) + norm(slip_y)) / 4;
% 
% Vx = Vx * correction_factor;
% Vy = Vy * correction_factor;
% 
% % Обновление координат и угла поворота (интеграция)
% x_real(tf,1) = x_real(tf - 1,1) + (Vx) * dt;
% y_real(tf,1) = y_real(tf - 1,1) + (Vy) * dt;
% teta_real(tf,1) = teta_real(tf - 1,1) + omega_psi * dt;

     
%      % 'обратная' модель робота
%      slip_r = slip_factors(tf,2);
%      slip_l = slip_factors(tf,1);
%      vr_history(tf, 1) = ((teta_target(tf,1) - teta_target(tf - 1,1))/dt) * (lx/2 - ((x_target(tf, 1) - (x_target(tf-1, 1))/(cos(teta_target(tf,1) - teta_target(tf - 1,1))*sin(teta_target(tf - 1,1))+sin(teta_target(tf - 1,1)) + cos(teta_target(tf,1) - teta_target(tf - 1,1))*sin(teta_target(tf - 1,1))))));
%      vl_history(tf, 1) = ((teta_target(tf,1) - teta_target(tf - 1,1))/dt) * (((x_target(tf, 1) - (x_target(tf-1, 1))/(-cos(teta_target(tf,1) - teta_target(tf - 1,1))*sin(teta_target(tf - 1,1)) - sin(teta_target(tf - 1,1)) - cos(teta_target(tf,1) - teta_target(tf - 1,1))*sin(teta_target(tf - 1,1))))) - lx/2);
%      
%      Mr_history(tf, 1) = 0.5*m*g*cos(alphaGround)*deltaTr + 0.5*m*((vr_history(tf,1) - vr_history(tf -1,1))/dt + g*sin(alphaGround))*r - (J0/dt) - ((vr_history(tf,1)/r) - (vr_history(tf - 1,1)/r));
%      Ml_history(tf, 1) = 0.5*m*g*cos(alphaGround)*deltaTr + 0.5*m*((vl_history(tf,1) - vl_history(tf -1,1))/dt + g*sin(alphaGround))*r - (J0/dt) - ((vl_history(tf,1)/r) - (vl_history(tf - 1,1)/r));
%      
%      wr_history(tf, 1) = Pmax / (0.5*Mr_history(tf, 1));
%      wl_history(tf, 1) = Pmax / (0.5*Ml_history(tf, 1));
%      
%      % 'прямая' модель робота
%      R_t = (lx/2) * ((vl_history(tf, 1) + vr_history(tf, 1))/((vr_history(tf, 1) - vl_history(tf, 1))));
%      omega = (vr_history(tf, 1) - vl_history(tf, 1))*lx;
%      
%      teta_real(tf,1) = teta_real(tf - 1,1) + omega*dt;
%      x_real(tf, 1) = x_real(tf - 1, 1) -  cos(omega*dt)*R_t*sin(teta_real(tf - 1,1)) - sin(omega*dt)*R_t*cos(teta_real(tf - 1,1)) - R_t*sin(teta_real(tf - 1,1));
%      y_real(tf, 1) = y_real(tf - 1, 1) - sin(omega*dt)*R_t*sin(teta_real(tf - 1,1)) + cos(omega*dt)*R_t*cos(teta_real(tf - 1,1)) + R_t*cos(teta_real(tf - 1,1));

 
 end

 %отображение результатов симуляции
figure(1);
subplot(2, 1,1);
plot(time, alpha(:,1), 'b');
title("Отклонение рукоятки джойстика по x");
xlabel("Время (с)");
ylabel("Угол (рад)");

subplot(2, 1,2);
plot(time, beta(:,1), 'r');
title("Отклонение рукоятки джойстика по y");
xlabel("Время (с)");
ylabel("Угол (рад)");

figure(2);
subplot(2, 1,1);
plot(time, joystick_pos_a(:,1), 'b');
title("Угол наклона джойстика по x");
xlabel("Время (с)");
ylabel("Угол (рад)");

subplot(2, 1,2);
plot(time, joystick_pos_b(:,1), 'r');
title("Угол наклона джойстика по y");
xlabel("Время (с)");
ylabel("Угол (рад)");


figure(3);
subplot(2, 1,1);
plot(time, slip_factors(:,1), 'b');
title("Коеффициент проскальзывания левых колес (%)");
xlabel("Время (с)");
ylabel("(%)");

subplot(2, 1,2);
plot(time, slip_factors(:,2), 'r');
title("Коеффициент проскальзывания правых колес (%)");
xlabel("Время (с)");
ylabel("(%)");

figure(4);
subplot(4, 1,1);
plot(x_target(:,1), y_target(:,1), 'b');
title("Желаемая траектория (м/м)");
xlabel("X (м)");
ylabel("У (м)");
subplot(4, 1,2);
plot(time, x_target(:,1), 'r');
title("Координата х");
xlabel("Время (с)");
ylabel("(м)")
subplot(4, 1,3);
plot(time, y_target(:,1), 'r');
title("Координата y");
xlabel("Время (с)");
ylabel("(м)")
subplot(4, 1,4);
plot(time, teta_target(:,1), 'r');
title("Желаемый курс робота");
xlabel("Время (с)");
ylabel("Курс (рад)");

figure(5);
subplot(4, 1,1);
plot(x_real(:,1), y_real(:,1), 'b');
title("Реальная траектория (м/м)");
xlabel("X (м)");
ylabel("У (м)");
subplot(4, 1,2);
plot(time, x_real(:,1), 'r');
title("Координата х");
xlabel("Время (с)");
ylabel("(м)")
subplot(4, 1,3);
plot(time, y_real(:,1), 'r');
title("Координата y");
xlabel("Время (с)");
ylabel("(м)")
subplot(4, 1,4);
plot(time, teta_real(:,1), 'r');
title("Реальный курс робота");
xlabel("Время (с)");
ylabel("Курс (рад)");

figure(6);
subplot(4, 1,1);
plot(time, wl_history(:,1), 'b');
title("Угловая скорость левых колес");
xlabel("(с)");
ylabel("(рад/c)");
subplot(4, 1,2);
plot(time, wr_history(:,1), 'r');
title("Угловая скорость правых колес");
xlabel("(с)");
ylabel("(рад/c)");
subplot(4, 1,3);
plot(time, vl_history(:,1), 'r');
title("Линейная скорость левых колес");
xlabel("(с)");
ylabel("(м/c)");
subplot(4, 1,4);
plot(time, vr_history(:,1), 'r');
title("Линейная скорость правых колес");
xlabel("(с)");
ylabel("(м/c)");

figure(7);
subplot(2, 1,1)
plot(time, Ml_history(:,1), 'r');
title("Суммарный момент левых колес");
xlabel("(с)");
ylabel("(Н/м)");
subplot(2, 1,2);
plot(time, Mr_history(:,1), 'r');
title("Суммарный момент правых колес");
xlabel("(с)");
ylabel("(Н/м)");
end

% Функция для плавного разгона угловых скоростей
function omega = gradual_acceleration(t, omega_start, omega_end, t_accel)
    if t <= t_accel
        omega = omega_start + (omega_end - omega_start) * (t / t_accel);
    else
        omega = omega_end; % После разгона угловая скорость постоянная
    end
end

%функция моделирования джойстика
%параметры: alpha -- угол отклонения джойстика по оси х
%           beta -- угол отклонения джойстика по оси y
%           h -- высота рукоятки джойстика (м)
%           x -- прежняя х-координата
%           y -- прежняя y-координата
%           teta -- прежнее направление движения
%           jc -- коеффициент пропорциональнасти для джойстика
%возвращает: вектор нового желаемого положения робота [xN, yN, tetaN]
function posVect = joystick(alpha, beta, h, x, y, teta, jc)

if ((alpha ~= 0)&&(beta == 0))
    k2 = h*cos(pi/2 - alpha);
    xN = x + jc*k2;
    yN = y;
    tetaN = teta;
elseif ((alpha == 0)&&(beta ~= 0))
    xN = x;
    yN = y;
    tetaN = teta + jc*beta;
elseif ((alpha == 0)&&(beta == 0))
    xN = x;
    yN = y;
    tetaN = teta;
else
    k2 = h*cos(pi/2 + alpha);
    k1 = h*cos(pi/2 - beta);
    V = sqrt(k1^2 + k2^2);
    gamma = acos(k1/V);

    xN = x + jc*k2;
    yN = y + jc*k1;
    tetaN = teta + jc*gamma;
end

posVect.xN = xN;
posVect.yN = yN;
posVect.tetaN = tetaN;
end
