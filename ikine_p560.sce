// Rotina base para obtenção de ângulos via cálculo numérico

// Habilita modo de execução passo a passo
m = mode();
//mode(7)

// Usando o modelo Puma 560

// Cria os links do manipulador a partir da TAbela  de Denavic-Hatemberg
L1 = Link('d', 0,       'a', 0,      'alpha', %pi/2,  'modified')
L2 = Link('d', 0,       'a', 0.4318, 'alpha', 0,      'modified')
L3 = Link('d', 0.15005, 'a', 0.0203, 'alpha', -%pi/2, 'modified')
L4 = Link('d', 0.4318,  'a', 0,      'alpha', %pi/2,  'modified')
L5 = Link('d', 0,       'a', 0,      'alpha', -%pi/2, 'modified')
L6 = Link('d', 0,       'a', 0,      'alpha', 0,      'modified')

// Cria uma lista sequencial dos links
L = list(L1,L2, L3, L4, L5, L6)

// Define o manipulador como um link serial
p560 = SerialLink(L, 'name', 'my robot')

// Utilizando os limites de ângulos apresentados no software RoKiSim
q_rokisim_minus = [-165, -110, -90, -160, -120, -400]
q_rokisim_plus = [+165, +110, +70, +160, +120, +400]

// Considerando q_int_minus intervalo menor de deslocamento e q_int_plus intervalo maior de deslocamento
// https://www.parallemic.org/RoKiSim.html
q_int_minus = %pi/360*q_rokisim_minus
q_int_plus  = %pi/360*q_rokisim_plus

// Considerando determinado ponto para ser posicionado
p = [0.1744, -0.4318, 0.3815]
p_x = p(1); p_y = p(2); p_z = p(3);

// Veja que o resultado de theta_1 = %pi/4 e theta_2 = %pi/8  coloca o manipulador no ponto p
T = fkine(p560, [%pi/4,%pi/8,0,0,0,0])

// Apresenta o resultado da cinemática direta de determinado conjunto de ângulos iniciais
s_ang = [%pi/2,0,0,0,0,0]
T_s   = fkine(p560, s_ang)

//Salva valores nas variáveis 
r_11 = T_s(1,1); r_12 = T_s(1,2); r_13 = T_s(1,3); p_x = T_s(1,4);
r_21 = T_s(2,1); r_22 = T_s(2,2); r_23 = T_s(2,3); p_y = T_s(2,4);
r_31 = T_s(3,1); r_32 = T_s(3,2); r_33 = T_s(3,3); p_z = T_s(3,4);
 
// Usando algumas equações da Cinemática Inversa do livro Craig (2006) disponível em:
// http://files.yuki-phantomhive.webnode.mx/200000031-25e8a26e29/Robotica.pdf

theta_1p = atan(p_y, p_x) - atan(L(3).d, +sqrt(p_x^2 + p_y^2-L(3).d^2))
theta_1n = atan(p_y, p_x) - atan(L(3).d, -sqrt(p_x^2 + p_y^2-L(3).d^2))
 
K = p_x^2 + p_y^2 + p_y^2 - L(2).a^2 - L(3).a^2 - L(3).d^2 - L(4).d^4
theta_3p = atan(L(3).a, L(4).d) - atan(K, +sqrt(L(3).a^2+L(4).d^2-K^2)) 
theta_3n = atan(L(3).a, L(4).d) - atan(K, -sqrt(L(3).a^2+L(4).d^2-K^2))

// Gera um vetor de posições intermediárias
t = [0:.01:2];
[a,samples] = size(t)

// Armazena em ang o valor dos ângulos mais próximos do ponto p
q_ang = [0,0,0,0,0,0]

// Calcula a trajetória das coordenadas
for axis = 1:6
    // Cria ponto inicial e final de análise
    q_p1 = [theta_1n,0,theta_3p,0,0,0]
    q_p2 = [theta_1n,0,theta_3p,0,0,0]
    // Busca ângulos já obtidos
    if axis > 1 then
        for i=1:axis
            if(q_p1(axis) == 0 & q_p2(axis) == 0) then
                q_p1(i)=q_ang(i)
                q_p2(i)=q_ang(i)
            end
        end
    end

    q_p1
    q_p2
    // Avalia intervalo de eixo
    q_p1(axis)= q_int_minus(axis)
    q_p2(axis)= q_int_plus(axis)
    
    q = jtraj(q_p1, q_p2, t);
    mprintf("\n q [%s] : %dx%d",typeof(q),size(q,1),size(q,2));

    // Calcula a cinemática direta dessas coordenadas
    T = fkine(p560,q);
    mprintf("\n T [%s] : %dx%dx%d",typeof(T),size(T,1),size(T,2),size(T,3))
    // Calcula a distância euclidiana e armazena o índice da menor distância
    dist_eucl = 0
    dist= zeros(samples)
    for i=1:samples
        dist(i) = sqrt((T(1,4,i)-p_x)^2 + (T(2,4,i)-p_y)^2 + (T(3,4,i)-p_z)^2)
        if i == 1 | dist(i) < dist_eucl then
            dist_eucl = dist(i)
            dist_index = i
            if(dist_eucl < 0.005) then
                mprintf("\nParada por distância euclidiana pequena ( < 0.005)")
                mprintf("\naxis: %i sample: %i dist_eucl: %f angle: %f\n", axis, dist_index, dist_eucl, q(dist_index, axis))
                break;
            end
        end
    end
    // Registra em ang o valor do ângulo de deslocamento de menor distância euclidiana
    mprintf("\naxis: %i sample: %i dist_eucl: %f angle: %f\n", axis, dist_index, dist_eucl, q(dist_index, axis))
    
    // Se desvio padrão das amostras é pequeno então esse ângulo não movimenta, pega ângulo do inicio
    if(st_deviation(dist) < 0.0001)then
        mprintf("\nDesvio padrao pequeno no eixo %d [%f]\n", axis, st_deviation(dist))
        q_ang(axis)=s_ang(axis)
    else
        disp(T(:,:,dist_index))
        q_ang(axis)= q(dist_index, axis)
        disp(q_ang)

    end
end
mprintf("\nParada por fim de cálculo\n");
T   = fkine(p560, q_ang)
dist_eucl = sqrt((T(1,4)-p_x)^2 + (T(2,4)-p_y)^2 + (T(3,4)-p_z)^2)
disp(q_ang); 
disp(T);
disp(dist_eucl);

mode(m);
return;
