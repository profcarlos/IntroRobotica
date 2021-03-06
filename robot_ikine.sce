// Usamos mod(7) para simulação passo a passo

m = mode();
mode(7);

// BAseado no modelo do PUMA 560
// https://edisciplinas.usp.br/pluginfile.php/4080556/mod_resource/content/1/Aula%202%20-%20SEM0317%20-%202017.pdf

// Cria os links do manipulador a partir da TAbela  de Denavic-Hatemberg
L_1 = 100
L_2 = 60
L_3 = 40 

L1 = Link('d', 0, 'a', 0, 'alpha', %pi/2, 'modified')
L2 = Link('d', 0, 'a', L_1, 'alpha', 0,     'modified')
L3 = Link('d', 0, 'a', L_2, 'alpha', 0, 'modified')
L4 = Link('d', 0, 'a', L_3, 'alpha', 0, 'modified')


// Cria uma lista sequencial dos links
L = list(L1,L2, L3, L4)

// Define o manipulador como um link serial
bot = SerialLink(L, 'name', 'my robot')

// Apresenta a quantidade de juntas do manipulador
bot.n
// Define uma posição inicial do manipulador robótico
qstart = [%pi/2 %pi/2, 0.0, 0.0]

// Apresenta a Cinemática Direta do manipulador dada a posição angular dos links
T = fkine(bot,qstart)
// Copia valores de translação do manipulador px, py, pz
p_x = T(1,4)
p_y = T(2,4)
p_z = T(3,4)

// Equações da Cinemática Inversa do Manipulador
// Insira aqui suas equações
c_2 = (L_1^2 + (L_2 + L_3)^2 - p_x^2 - p_y^2)/(L_1^2 + (L_2 + L_3)^2)
tetha_2a = atan(+sqrt(1-c_2^2), c_2)
tetha_2b = atan(-sqrt(1-c_2^2), c_2)
tetha_1a = 0
tetha_1b = %pi/4
tetha_3a = 0
tetha_3b = 0

// Apresenta o modelo do manipulador dada a posição angular dos links
pos = [tetha_1a, tetha_2a, tetha_3a, 0]
plot_robot(bot,pos)

// Caso tenha mais de uma posição vamos plotar para ver qual a que representa a posição desejada
pos = [tetha_1b, tetha_2a, tetha_3a, 0]
plot_robot(bot,pos)
mode(m);
