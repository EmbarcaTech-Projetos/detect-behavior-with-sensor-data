import os
import sys
import pickle
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

from api.data_processing import ToFScaler

print("Iniciando a reconstrução do arquivo pickle...")

# Caminhos para os arquivos
artifacts_dir = os.path.join(project_root, 'artifacts')
old_pickle_path = os.path.join(artifacts_dir, 'train_scalers.pkl')
new_pickle_path = os.path.join(artifacts_dir, 'train_scalers_fixed.pkl')

try:
    print(f"Lendo o arquivo antigo: {old_pickle_path}")
    with open(old_pickle_path, 'rb') as f:
        non_tof_scaler, tof_scaler = pickle.load(f)

    print(f"Salvando o novo arquivo: {new_pickle_path}")
    with open(new_pickle_path, 'wb') as f:
        pickle.dump((non_tof_scaler, tof_scaler), f)

    print("\n✅ Reconstrução concluída com sucesso!")
    print("Lembre-se de atualizar 'api/ml_loader.py' para usar 'train_scalers_fixed.pkl'.")

except FileNotFoundError:
    print(f"ERRO: O arquivo '{old_pickle_path}' não foi encontrado.")
except Exception as e:
    print(f"Ocorreu um erro: {e}")