# Cálculo da Matriz Jacobiana e Matrizes Ômega

Este projeto contém um script em Python (`jacobiano.py`) desenvolvido para calcular a **Matriz Jacobiana Analítica** e as **Matrizes Ômega** (matrizes antissimétricas de velocidade angular) de um robô manipulador. O cálculo é feito a partir dos parâmetros de **Denavit-Hartenberg (DH)**, que devem ser fornecidos em um arquivo JSON estruturado.

O projeto utiliza a biblioteca **SymPy** para realizar cálculos simbólicos, o que permite que os parâmetros DH sejam inseridos como variáveis (ex: `theta1`, `d3`, `L1`) ou expressões matemáticas (ex: `pi/2`), gerando matrizes genéricas e exatas.

## Pré-requisitos

Para rodar o script, você precisará ter o Python instalado e a biblioteca `sympy`.

Você pode instalar as dependências necessárias utilizando o `pip`:

```bash
pip install sympy
```

## Como Usar

O script é executado via linha de comando (terminal), passando o caminho para o arquivo JSON contendo a modelagem do robô.

```bash
python jacobiano.py <nome_do_arquivo>.json
```

**Exemplo:**
```bash
python jacobiano.py example.json
```
ou
```bash
python jacobiano.py IRB_910.json
```

## Estrutura do Arquivo JSON

Os parâmetros do robô são definidos em um arquivo JSON. O arquivo deve conter o nome do robô e uma lista de juntas (`joints`) em ordem, da base ao efetuador final.

Exemplo de estrutura:

```json
{
    "name": "Nome do Robo",
    "joints": [
        {
            "type": "revolute",
            "a": "L1",
            "alpha": 0,
            "d": "d1",
            "theta": "theta1"
        },
        {
            "type": "prismatic",
            "a": 0,
            "alpha": "-90",
            "d": "d2",
            "theta": 0
        }
    ]
}
```

### Campos Suportados em cada Junta:
- **`type`**: O tipo da junta. Pode ser `"revolute"` (rotacional) ou `"prismatic"` (prismática).
- **`a`**: Distância entre os eixos Z (comprimento do elo).
- **`alpha`**: Ângulo de torção do elo (em graus ou radianos, o script tenta realizar a conversão adequada; para exatidão use expressões como `pi/2` ou variáveis).
- **`d`**: Deslocamento ao longo do eixo Z anterior (offset do elo).
- **`theta`**: Ângulo de rotação em torno do eixo Z anterior (variável da junta rotacional).

**Nota sobre valores:** Os valores de `a`, `alpha`, `d` e `theta` podem ser números (ex: `0`, `175`), strings representando variáveis simbólicas (ex: `"theta1"`, `"d3"`, `"L1"`) ou expressões matemáticas suportadas pelo SymPy (ex: `"pi/2"`).

## Saídas do Programa

Ao rodar o script, ele realizará os seguintes passos e gerará as respectivas saídas:

1. **Saída no Console**:
   - Imprime a Matriz Jacobiana ($J$) analítica simplificada do manipulador.
   - Imprime as Matrizes Ômega ($\Omega$) simplificadas de cada junta.
2. **Geração de Arquivos CSV**:
   - `jacobiana_<nome_do_robo>.csv`: Arquivo contendo a matriz Jacobiana (6 linhas por $N$ colunas, onde $N$ é o número de juntas).
   - `omega_<nome_do_robo>.csv`: Arquivo contendo as Matrizes Ômega de 3x3 para cada junta organizadas de forma sequencial.