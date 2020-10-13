// Usamos mod(7) para simulação passo a passo

m = mode();
mode(7);

// Baseado no modelo do PUMA 560 e parâmetros da Figura 3.21 do Livro Texto
// Introdução à Robótica, Craig.

// Cria os links do manipulador a partir da TAbela  de Denavic-Hatemberg
L1 = Link('alpha',      0, 'a', 0, 'd', 0,  'modified')
L2 = Link('alpha', -%pi/2, 'a', 0, 'd', 0,  'modified')
L3 = Link('alpha',      0, 'a', 1, 'd', 1,  'modified')
L4 = Link('alpha', -%pi/2, 'a', 1, 'd', 1,  'modified')
L5 = Link('alpha',  %pi/2, 'a', 0, 'd', 0,  'modified')
L6 = Link('alpha', -%pi/2, 'a', 0, 'd', 0,  'modified')

// Cria uma lista sequencial dos links
L = list(L1,L2, L3, L4, L5, L6)

// Define o manipulador como um link serial
bot = SerialLink(L, 'name', 'my robot')

// Apresenta a quantidade de juntas do manipulador
bot.n
// Define uma posição inicial do manipulador robótico
qstart = [%pi/2 %pi/2, 0.0, 0.0, 0.0, 0.0]

// Apresenta a Cinemática Direta do manipulador dada a posição angular dos links
fkine(bot,qstart)

// Apresenta o modelo do manipulador dada a posição angular dos links
plot_robot(bot,qstart);
//
mode(m);
