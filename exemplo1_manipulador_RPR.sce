// Usamos mod(7) para simulação passo a passo

m = mode();
//mode(7);

// Exemplo Figura 3.10, livro Introdução a Robótica, Craig
// Manipulador Robótico RPR
// Cria os links do manipulador a partir da Tabela  de Denavic-Hatemberg
L1 = Link('alpha',      0, 'a', 0, 'd', 0,   'revolute', 'modified')
L2 = Link('alpha', %pi/2, 'a', 0,  'd', 0,  'prismatic', 'modified')
L3 = Link('alpha',      0, 'a', 0, 'd', 0.5, 'revolute', 'modified')

// Cria uma lista sequencial dos links
L = list(L1,L2, L3)

// Define o manipulador como um link serial
bot = SerialLink(L, 'name', 'my robot')

// Apresenta a quantidade de juntas do manipulador
bot.n
// Define uma posição inicial do manipulador robótico
qstart = [%pi/2,0, %pi/2]

// Apresenta a Cinemática Direta do manipulador dada a posição angular dos links
fkine(bot,qstart)

// Apresenta o modelo do manipulador dada a posição angular dos links
plot_robot(bot,qstart);


mode(m);
