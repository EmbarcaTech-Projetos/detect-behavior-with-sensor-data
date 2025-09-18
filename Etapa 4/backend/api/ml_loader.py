import torch
import pickle
import os
import glob
from .model_def import IMUOnlyModel
from .data_processing import ToFScaler  
from django.conf import settings        

MODELS = []
NON_TOF_SCALER = None
TOF_SCALER = None
GESTURE_ENCODER = None

def load_artifacts():
    """Carrega todos os artefatos para as variáveis globais."""
    global MODELS, NON_TOF_SCALER, TOF_SCALER, GESTURE_ENCODER

    # Garante que os modelos não sejam carregados múltiplas vezes se a função for chamada novamente
    if MODELS:
        return

    print("Loading ML artifacts...")

    DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    MODELS_DIR = os.path.join(settings.BASE_DIR, 'models')
    ARTIFACTS_DIR = os.path.join(settings.BASE_DIR, 'artifacts')
    
    N_CLASSES = 18

    # Carrega os scalers
    scaler_path = os.path.join(ARTIFACTS_DIR, 'train_scalers_fixed.pkl')
    with open(scaler_path, 'rb') as f:
        NON_TOF_SCALER, TOF_SCALER = pickle.load(f)

    # Carrega o gesture encoder
    encoder_path = os.path.join(ARTIFACTS_DIR, 'gesture_encoder.pkl')
    with open(encoder_path, 'rb') as f:
        GESTURE_ENCODER = pickle.load(f)

    # Carrega todos os modelos treinados 
    model_paths = sorted(glob.glob(os.path.join(MODELS_DIR, "model_fold*_best.pth")))
    if not model_paths:
        raise FileNotFoundError(f"No models found in {MODELS_DIR}")

    for path in model_paths:
        model = IMUOnlyModel(imu_dim=20, tof_dim=325, n_classes=N_CLASSES).to(DEVICE)
        model.load_state_dict(torch.load(path, map_location=DEVICE))
        model.eval()
        MODELS.append(model)

    print(f"✅ ML artifacts loaded: {len(MODELS)} models.")