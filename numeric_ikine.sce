// Rotina base para obtenção de ângulos via cálculo numérico

// Habilita modo de execução passo a passo
m = mode();
mode(7)

// Usando o modelo Puma 560
mdl_puma560

// Considerando q_start ponto inicial e q_end ponto final

q_start = [0,0,0,0,0,0]

q_end   = [%pi/2, 0, 0, 0, 0, 0]

// Considerando q_int_minus intervalo menor de deslocamento e q_int_plus intervalo maior de deslocamento 
q_int_minus = [-%pi/2,-%pi/2,-%pi/2,-%pi/2,-%pi/2,-%pi/2]

q_int_plus  = [%pi/2,%pi/2,%pi/2,%pi/2,%pi/2,%pi/2]

// Apresenta o resultado da cinemática direta de determinados conjuntos de ângulos

fkine(p560,q_start)

fkine(p560, q_end)

// Veja que o resultado de %pi/4 no ângulo theta_1 coloca o manipulador no ponto p

T_p = fkine(p560, [%pi/4,0,0,0,0,0])

//Salva valores nas variáveis 
r_11 = T(1,1); r_12 = T(1,2); r_13 = T(1,3); p_x = T(1,4);
r_21 = T(2,1); r_22 = T(2,2); r_23 = T(2,3); p_y = T(2,4);
r_31 = T(3,1); r_32 = T(3,2); r_33 = T(3,3); p_z = T(3,4);

// Usando as equações da Cinemática Inversa do livro Craig (2006) disponível em:
// http://files.yuki-phantomhive.webnode.mx/200000031-25e8a26e29/Robotica.pdf
theta1a = atan(p_y, p_x) - atan(L(3).d, sqrt(p_x^2 + p_y^2-L(3).d^2))
theta1b = atan(p_y, p_x) - atan(L(3).d, sqrt(p_x^2 + p_y^2-L(3).d^2)) 


// Considerando determinado ponto para ser posicionado
p = [0.42, 0.22, 0.43]

// Gera um vetor de posições intermediárias
t = [0:.05:2];
[a,samples] = size(t)

// Armazena em ang o valor dos ângulos mais próximos do ponto p
q_ang = [0,0,0,0,0,0]

// Calcula a trajetória das coordenadas
for axis = 1:6
    // Cria ponto inicial e final de análise
    q_p1 = [0,0,0,0,0,0]
    q_p2 = [0,0,0,0,0,0]
    // Busca ângulos já obtidos
    if axis > 1 then
        for i=1:axis
            q_p1(i)=q_ang(i)
            q_p2(i)=q_ang(i)
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
    for i=1:samples
        dist = sqrt((T(1,4,i)-p(1))^2 + (T(2,4,i)-p(2))^2 * (T(3,4,i)-p(3))^2)
        if i == 1 | dist < dist_eucl then
            dist_eucl = dist
            dist_index = i
        end
    end
    // Registra em ang o valor do ângulo de deslocamento de menor distância euclidiana
    mprintf("axis: %i sample: %i dist_eucl: %f matriz:\n", axis, dist_index, dist_eucl)
    T(:,:,dist_index)
    q_ang(axis)=q(dist_index, axis)
    if(dist_eucl < 0.005) then
        mprintf("Parada por distância euclidiana pequena ( < 0.005)")
        return;
    end
end
mode(m);
return;
