#include <iostream>
using namespace std;

int main() {
    // lê o número de linhas da grelha unitária
    int n;
    cin >> n;

    // lê o número de colunas da grelha unitária
    int m;
    cin >> m;

    // cria um vetor para armazenar os valores ci
    int c[n];

    // inicializa o número total de configurações
    int total_configuracoes = 0;
    // para cada linha da grelha
    for (int i = 0; i < n; i++) {
        // lê o valor ci que indica o índice da coluna pelo qual o caminho passa na linha i
        cin >> c[i];

        // calcula o número de colunas à esquerda do caminho em escada na linha i
        int colunas_esquerda = c[i];

        // cria um vetor para armazenar o número de configurações possíveis para cada tamanho de ladrilho
        int configuracoes_esquerda[colunas_esquerda + 1];
        // inicializa o número de configurações para cada ladrilho como 1
        for (int j = 1; j <= colunas_esquerda; j++) {
            configuracoes_esquerda[j] = 1;
        }

        // para cada tamanho de ladrilho, desde o menor até o maio
        for (int j = 2; j <= colunas_esquerda; j++) {
            // para cada coluna que pode ser coberta pelo ladrilho atual
            for (int k = j; k <= colunas_esquerda; k++) {
                // multiplica o número de configurações possíveis para o ladrilho anterior pelo número de configurações possíveis para o ladrilho atual na coluna atual
                int novo_num_configuracoes = configuracoes_esquerda[j - 1] * configuracoes_esquerda[k];
                // adiciona o resultado obtido ao número de configurações para o ladrilho atual na coluna atual
                configuracoes_esquerda[k] += novo_num_configuracoes;
            }
        }
        // se esta é a primeira linha da grelha, armazena o número de configurações para a linha atual no total de configurações
        if (i == 0) {
            total_configuracoes = configuracoes_esquerda[colunas_esquerda];
        }
        // caso contrário, multiplica o número de configurações para a linha anterior pelo número de configurações para a linha atual
        else {
            total_configuracoes *= configuracoes_esquerda[colunas_esquerda];
        }
        // se o número de configurações para o caminho em escada inteiro for definido por ci = 0 para todas as linhas, o número total de configurações é 0
        if (c[0] == 0) {
            total_configuracoes = 0;
        }

        // escreve o número total de configurações no output
        cout << total_configuracoes << endl;

        return 0;
        }