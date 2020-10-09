// Usamos mod(7) para simulação passo a passo

m = mode();
mode(7);

// BAseado no modelo do PUMA 560
// https://edisciplinas.usp.br/pluginfile.php/4080556/mod_resource/content/1/Aula%202%20-%20SEM0317%20-%202017.pdf

// Cria os links do manipulador a partir da TAbela  de Denavic-Hatemberg
L1 = Link('d', 0, 'a', 0, 'alpha', -%pi/2, 'modified')
L2 = Link('d', 1, 'a', 1, 'alpha', 0,     'modified')
L3 = Link('d', 0, 'a', 1, 'alpha', %pi/2, 'modified')
L4 = Link('d', 1, 'a', 1, 'alpha', 0,     'modified')
L5 = Link('d', 0, 'a', 1, 'alpha', %pi/2, 'modified')
L6 = Link('d', 0, 'a', 1, 'alpha', 0,     'modified')

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
