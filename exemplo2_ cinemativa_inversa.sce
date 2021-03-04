// Usamos mod(7) para simulação passo a passo

m = mode();
mode(7);

// Exemplo Figura 3.10, livro Introdução a Robótica, Craig
// Manipulador Robótico RPR
// Define as constantes do manipulador
L1 = 0.0
L2 = 0.5
L3 = 0.1


// Cria os links do manipulador a partir da Tabela  de Denavic-Hatemberg
Link_0 = Link('alpha',      0, 'a', 0, 'd', 0,   'prismatic', 'modified')
Link_1 = Link('alpha',      0, 'a', 0, 'd', 0,   'revolute', 'modified')
Link_2 = Link('alpha', %pi/2, 'a', 0,  'd', 0,  'prismatic', 'modified')
Link_3 = Link('alpha',      0, 'a', 0, 'd', L2, 'revolute', 'modified')
Link_4 = Link('alpha',      0, 'a', 0, 'd', L3, 'revolute', 'modified')

// Cria uma lista sequencial dos links
L = list(Link_0, Link_1,Link_2, Link_3, Link_4)

// Define o manipulador como um link serial
bot = SerialLink(L, 'name', 'my robot')

// Apresenta a quantidade de juntas do manipulador
bot.n
// Define uma posição inicial do manipulador robótico
theta1_s = %pi/2
d2_s     = 0
theta3_s = 0
q_start = [L1, theta1_s,d2_s, theta3_s, 0]

// Apresenta a Cinemática Direta do manipulador dada a posição angular dos links
T_s = fkine(bot,q_start)

//Salva valores nas variáveis 
r_11 = T_s(1,1); r_12 = T_s(1,2); r_13 = T_s(1,3); p_x = T_s(1,4);
r_21 = T_s(2,1); r_22 = T_s(2,2); r_23 = T_s(2,3); p_y = T_s(2,4);
r_31 = T_s(3,1); r_32 = T_s(3,2); r_33 = T_s(3,3); p_z = T_s(3,4);

// Apresenta o modelo do manipulador dada a posição angular dos links
plot_robot(bot,q_start);

// Usaremosas posições atuais da ponta do manipulador nas equações de Cinemática Inversa
p_x
p_y
p_z
// Conforme as equações de Cinemática Inversa obtém-se possibilidades de valores

// Calculo de theta1
theta1_p = atan(-p_x+L1, p_y)
theta1_n = atan(p_x-L1, -p_y)

// Cálculo de d2
d2_p = sin(theta1_p)*p_x - cos(theta1_p)*p_y-sin(theta1_p)*L1 - L3 - L2
d2_n = sin(theta1_n)*p_x - cos(theta1_n)*p_y-sin(theta1_n)*L1 - L3 - L2

//Calculo de theta3
b_p = cos(theta1_p)*r_11+sin(theta1_p)*r_21
b_n = cos(theta1_n)*r_11+sin(theta1_n)*r_21
// Os valores de r_11 e r_21 deverias ser da posição anterior
// Como estamos calculando os ângulos a partir da posição atual
// pode ser por isso que  o resultado de theta3 ficou diferente
theta3_p1 = atan(b_p, sqrt(1-b_p*b_p))
theta3_n1 = atan(b_p, -sqrt(1-b_p*b_p))

theta3_p2 = atan(b_n, sqrt(1-b_n*b_n))
theta3_n2 = atan(b_n, -sqrt(1-b_n*b_n))

// Obtem uma posição de teste
q_test1 = [L1, theta1_p,d2_p, theta3_p2, 0]
// Apresenta o modelo do manipulador dada a posição angular dos links
plot_robot(bot,q_test1);

// Obtem outra posição de teste
q_test2= [L1, theta1_n,d2_n, theta3_n1, 0]
// Apresenta o modelo do manipulador dada a posição angular dos links
plot_robot(bot,q_test2);
mode(m);
