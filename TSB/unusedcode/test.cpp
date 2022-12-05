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

        // calcula o número de colunas à esquerda e à direita do caminho em escada na linha i
        int colunas_esquerda = c[i];
        int colunas_direita = m - c[i] - 1;

        // calcula o número de configurações possíveis para ladrilhar as colunas à esquerda e à direita do caminho na linha i
        int configuracoes_esquerda = 1;
        int configuracoes_direita = 1;
        for (int j = 0; j < colunas_esquerda; j++) {
            configuracoes_esquerda = configuracoes_esquerda * 2;
        }
        for (int j = 0; j < colunas_direita; j++) {
            configuracoes_direita = configuracoes_direita * 2;
        }

        // atualiza o número total de configurações
        total_configuracoes = total_configuracoes + configuracoes_esquerda * configuracoes_direita;
    }

    // imprime o número total de configurações
    cout << total_configuracoes << endl;

    return 0;
}
