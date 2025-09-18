import torch
import torch.nn.functional as F
import numpy as np
import pandas as pd
import polars as pl
import pickle
import os
import glob
from copy import deepcopy

from model_def import IMUOnlyModel
from data_processing import GestureDataset, ToFScaler

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
MODELS_DIR = 'models'
ARTIFACTS_DIR = 'artifacts'
N_CLASSES = 18 

def load_artifacts():
    """Carrega todos os artifacts necessários: scalers, encoder, and models."""
    print("Carregando artifacts...")
    
    with open(os.path.join(ARTIFACTS_DIR, 'train_scalers.pkl'), 'rb') as f:
        non_tof_scaler, tof_scaler = pickle.load(f)

    with open(os.path.join(ARTIFACTS_DIR, 'gesture_encoder.pkl'), 'rb') as f:
        gesture_encoder = pickle.load(f)

    model_paths = sorted(glob.glob(os.path.join(MODELS_DIR, "model_fold*_best.pth")))
    if not model_paths:
        raise FileNotFoundError(f"No models found in {MODELS_DIR}")
        
    models = []
    for path in model_paths:
        model = IMUOnlyModel(imu_dim=20, tof_dim=325, n_classes=N_CLASSES).to(DEVICE)
        model.load_state_dict(torch.load(path, map_location=DEVICE))
        model.eval()
        models.append(model)
    
    print(f"✅ Artifacts loaded: {len(models)} models, 2 scalers, 1 gesture encoder.")
    return models, non_tof_scaler, tof_scaler, gesture_encoder

def make_prediction(sequence_df, demographics_df, loaded_artifacts):
    models, non_tof_scaler, tof_scaler, gesture_encoder = loaded_artifacts
    
    inference_dataset = GestureDataset(sequence_dir=None, label_dir=None, metadata_df=pd.DataFrame())
    inference_dataset.non_tof_scaler = non_tof_scaler
    inference_dataset.tof_scaler = tof_scaler
    inference_dataset.gesture_encoder = gesture_encoder

    IMU_FEATURES = ['acc_x', 'acc_y', 'acc_z', 'rot_w', 'rot_x', 'rot_y', 'rot_z']
    THM_FEATURES = ['thm_1', 'thm_2', 'thm_3', 'thm_4', 'thm_5']
    TOF_FEATURES = [f'tof_{sensor}_v{pixel}' for sensor in range(1, 6) for pixel in range(64)]
    STATIC_FEATURES = ['adult_child', 'age', 'sex', 'handedness', 'height_cm', 'shoulder_to_wrist_cm', 'elbow_to_wrist_cm']
    
    seq_np = sequence_df.select(IMU_FEATURES + THM_FEATURES + TOF_FEATURES).to_numpy().astype(np.float32)
    demo_np = demographics_df.select(STATIC_FEATURES).fill_null(0).to_numpy().astype(np.float32)[0]
    
    full_seq = np.concatenate([seq_np, np.tile(demo_np, (len(seq_np), 1))], axis=1)

    engineered_features = inference_dataset._feature_engineering(full_seq)
    norm_features = inference_dataset.normalize_features(engineered_features)

    tof_dim = inference_dataset.tof_dim
    tof_features = norm_features[:, -tof_dim:]
    imu_thm_demo_features = norm_features[:, :-tof_dim]
    demo_features = imu_thm_demo_features[0, -7:]
    thm_features = imu_thm_demo_features[:, :-7][:, -5:]
    imu_features = imu_thm_demo_features[:, :-7][:, :-5]
    thm_tof_features = np.concatenate([thm_features, tof_features], axis=1)

    imu_tensor = torch.tensor(imu_features, dtype=torch.float32).unsqueeze(0).to(DEVICE)
    thm_tof_tensor = torch.tensor(thm_tof_features, dtype=torch.float32).unsqueeze(0).to(DEVICE)
    demo_tensor = torch.tensor(demo_features, dtype=torch.float32).unsqueeze(0).to(DEVICE)
    
    with torch.no_grad():
        all_preds = []
        for model in models:
            outputs = model(imu_tensor, thm_tof_tensor, demo_tensor)
            probs = F.softmax(outputs, dim=1)
            all_preds.append(probs)
        
        avg_pred = torch.stack(all_preds).mean(dim=0)
        pred_idx = avg_pred.argmax(dim=1).item()
        pred_label = gesture_encoder.inverse_transform([pred_idx])[0]
        
    return pred_label

if __name__ == "__main__":
    artifacts = load_artifacts()

    print("\n--- Running Prediction Example ---")
    
    sequence_length = 150
    mock_seq_data = {
        'acc_x': np.random.randn(sequence_length), 'acc_y': np.random.randn(sequence_length), 'acc_z': np.random.randn(sequence_length),
        'rot_w': np.random.randn(sequence_length), 'rot_x': np.random.randn(sequence_length), 'rot_y': np.random.randn(sequence_length), 'rot_z': np.random.randn(sequence_length),
        'thm_1': np.random.randn(sequence_length), 'thm_2': np.random.randn(sequence_length), 'thm_3': np.random.randn(sequence_length), 'thm_4': np.random.randn(sequence_length), 'thm_5': np.random.randn(sequence_length),
        **{f'tof_{s}_v{p}': np.random.randint(-1, 255, sequence_length) for s in range(1, 6) for p in range(64)}
    }
    mock_sequence_df = pl.DataFrame(mock_seq_data)

    mock_demo_data = {
        'adult_child': [0], 'age': [30], 'sex': [1], 'handedness': [0],
        'height_cm': [175], 'shoulder_to_wrist_cm': [60], 'elbow_to_wrist_cm': [30]
    }
    mock_demographics_df = pl.DataFrame(mock_demo_data)

    predicted_gesture = make_prediction(mock_sequence_df, mock_demographics_df, artifacts)
    
    print(f"\nReceived mock sensor data...")
    print(f"==> Predicted Gesture: {predicted_gesture}")