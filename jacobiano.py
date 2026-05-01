import json
import sympy as sp
import csv
import sys

def parse_value(val, is_angle=False):
    """
    Tenta converter o valor lido do JSON.
    Usa o Sympy para fazer conversões matemáticas seguras (ex: 'pi/2', '0', 'L1', 'q1').
    """
    if val is None or str(val).strip() == '':
        return 0
    try:
        parsed = sp.sympify(str(val))
    except Exception:
        parsed = sp.Symbol(str(val))
        
    if is_angle:
        if hasattr(parsed, 'free_symbols') and not parsed.free_symbols and not parsed.has(sp.pi):
            return parsed * sp.pi / 180
        if isinstance(parsed, (int, float)):
            return parsed * sp.pi / 180
            
    return parsed

def get_dh_matrix(theta, d, a, alpha):
    """
    Retorna a matriz de transformação homogênea padrão usando o modelo Denavit-Hartenberg.
    """
    A = sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d              ],
        [0,              0,                            0,                           1              ]
    ])
    return sp.simplify(A)

def get_omega_matrix(w):
    """
    Retorna a matriz antissimétrica (Matriz Ômega) a partir de um vetor 3x1.
    Usada para calcular o produto vetorial (w x v = Ômega * v).
    """
    return sp.Matrix([
        [    0, -w[2],  w[1]],
        [ w[2],     0, -w[0]],
        [-w[1],  w[0],     0]
    ])

def calculate_jacobian_from_json(json_file):
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            robot_data = json.load(f)
    except Exception as e:
        print(f"Erro ao ler o arquivo JSON: {e}")
        return None
        
    print(f"--- Calculando Jacobiana para o Robô: {robot_data.get('name', 'Desconhecido')} ---")
    
    joints = []
    
    # Extrair os parâmetros DH do JSON
    for joint_data in robot_data.get('joints', []):
        j_type = joint_data.get('type', 'revolute')
        
        # Chaves de acordo com o padrão DH (theta, d, a, alpha)
        theta_str = joint_data.get('theta', '0')
        d_str = joint_data.get('d', '0')
        a_str = joint_data.get('a', '0')
        alpha_str = joint_data.get('alpha', '0')
        
        joints.append({
            'type': j_type,
            'theta': parse_value(theta_str, is_angle=True),
            'd': parse_value(d_str),
            'a': parse_value(a_str),
            'alpha': parse_value(alpha_str, is_angle=True)
        })

    n_joints = len(joints)
    
    # A Matriz de Transformação da Base (Identidade no inicio)
    T = sp.eye(4)
    
    # Vetores z_i e posições p_i
    # frame da base
    z_list = [sp.Matrix([0, 0, 1])]
    p_list = [sp.Matrix([0, 0, 0])]
    
    # 1. Passo Direto da Cinemática - Calcula posições e eixos para todas as juntas
    print("Calculando as matrizes de transformações homogêneas...")
    for i in range(n_joints):
        dh = joints[i]
        A_i = get_dh_matrix(dh['theta'], dh['d'], dh['a'], dh['alpha'])
        T = T * A_i
        
        # Opcional: tentar simplificar no meio do caminho para acelerar o processo final
        T = sp.simplify(T) 
        
        # Extrair z_i (terceira coluna das submatriz de rotação, elementos 0 a 2)
        z_i = T[0:3, 2] 
        z_list.append(z_i)
        
        # Extrair p_i (vetor translação, elementos 0 a 2 da última coluna)
        p_i = T[0:3, 3]
        p_list.append(p_i)
        
    p_n = p_list[-1] # Posição final (Efetuador)
    
    # Criar uma matriz zerada para a Jacobiana (J tem 6 linhas de GDL x N Juntas)
    J = sp.zeros(6, n_joints)
    
    # 2. Computar a Matriz Jacobiana Analítica coluna por coluna
    print("Montando a Matriz Jacobiana...")
    omega_list = []
    for i in range(n_joints):
        j_type = joints[i]['type'].lower()
        
        z_prev = z_list[i]
        p_prev = p_list[i]
        
        if j_type == 'revolute' or j_type == 'rotational':
            # Junta Rotacional
            p_diff = p_n - p_prev
            Omega = get_omega_matrix(z_prev)
            Jv = Omega * p_diff
            Jw = z_prev
            omega_list.append(Omega)
        elif j_type == 'prismatic':
            # Junta Prismática
            Jv = z_prev
            Jw = sp.zeros(3, 1)
            omega_list.append(sp.zeros(3, 3))
        else:
            print(f"AVISO: O tipo de junta '{j_type}' é desconhecido. Assumindo que é revoluta.")
            p_diff = p_n - p_prev
            Omega = get_omega_matrix(z_prev)
            Jv = Omega * p_diff
            Jw = z_prev
            omega_list.append(Omega)
            
        J[0:3, i] = Jv
        J[3:6, i] = Jw

    # 3. Simplificar o máximo possível
    print("\nSimplificando a Matriz Jacobiana final (Isso pode levar alguns instantes para RBs complexos)...")
    J = sp.simplify(J)
    
    print("\n========= MATRIZ JACOBIANA =========")
    sp.pprint(J)
    
    # Salvar em CSV
    csv_filename = f"jacobiana_{robot_data.get('name', 'robo').replace(' ', '_')}.csv"
    try:
        with open(csv_filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # J.tolist() converte a matriz do SymPy em uma lista de listas
            writer.writerows(J.tolist())
        print(f"\n[+] Matriz salva com sucesso no arquivo: {csv_filename}")
    except Exception as e:
        print(f"\n[-] Erro ao salvar arquivo CSV: {e}")
        
    print("\n========= MATRIZES ÔMEGA =========")
    omega_csv_filename = f"matrizes_omega_{robot_data.get('name', 'robo').replace(' ', '_')}.csv"
    try:
        with open(omega_csv_filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            for i, Omega in enumerate(omega_list):
                print(f"Matriz Ômega da Junta {i+1}:")
                sp.pprint(sp.simplify(Omega))
                print()
                
                writer.writerow([f"Junta {i+1}"])
                writer.writerows(sp.simplify(Omega).tolist())
                writer.writerow([]) # Linha em branco
        print(f"[+] Matrizes Ômega salvas com sucesso no arquivo: {omega_csv_filename}")
    except Exception as e:
        print(f"[-] Erro ao salvar arquivo CSV das Matrizes Ômega: {e}")

    print("======================================")
    
    return J

if __name__ == "__main__":
    if len(sys.argv) > 1:
        json_filename = sys.argv[1]
    else:
        print("como rodar: python jacobiano.py <nome_do_arquivo>.json")
    calculate_jacobian_from_json(json_filename)
