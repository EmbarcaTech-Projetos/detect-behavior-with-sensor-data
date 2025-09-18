from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
import json
import torch
import torch.nn.functional as F
import numpy as np
import pandas as pd
import polars as pl
from django.shortcuts import render

# Importa os artefatos carregados e suas classes
from .ml_loader import MODELS, NON_TOF_SCALER, TOF_SCALER, GESTURE_ENCODER
from .data_processing import GestureDataset
from .models import PredictionLog

TARGET_GESTURES = [
    'Above ear - pull hair',
    'Cheek - pinch skin',
    'Eyebrow - pull hair',
    'Eyelash - pull hair',
    'Forehead - pull hairline',
    'Forehead - scratch',
    'Neck - pinch skin',
    'Neck - scratch',
]

@csrf_exempt
def predict(request):
    if request.method != 'POST':
        return JsonResponse({'error': 'Only POST requests are accepted'}, status=405)

    try:
        print(request.body)
        data = json.loads(request.body)
        sequence_data = data['sequence']
        sequence_df = pl.DataFrame(sequence_data)
        
        # --- ETAPA 1: Extrair os dados REAIS de IMU e Temperatura ---
        IMU_FEATURES = ['acc_x', 'acc_y', 'acc_z', 'rot_w', 'rot_x', 'rot_y', 'rot_z']
        THM_FEATURES = ['thm_1', 'thm_2', 'thm_3', 'thm_4', 'thm_5']
        
        imu_np = sequence_df.select(IMU_FEATURES).to_numpy().astype(np.float32)
        thm_np = sequence_df.select(THM_FEATURES).to_numpy().astype(np.float32)
        
        num_timesteps = imu_np.shape[0]

        # --- ETAPA 2: Criar dados "falsos" APENAS para o que está faltando ---
        tof_np = np.zeros((num_timesteps, 320)) # TOF está faltando
        demo_np = np.zeros(7)                   # Dados demográficos estão faltando

        # --- ETAPA 3: Montar o array completo na ordem correta do treinamento ---
        # Ordem original: IMU (7), THM (5), TOF (320), DEMO (7)
        full_seq = np.concatenate([
            imu_np,    # <-- Dados reais
            thm_np,    # <-- Dados reais
            tof_np,    # <-- Dados falsos (zeros)
            np.tile(demo_np, (num_timesteps, 1)) # <-- Dados falsos (zeros)
        ], axis=1)

        # --- ETAPA 4: O pipeline continua como antes ---
        inference_dataset = GestureDataset(sequence_dir=None, label_dir=None, metadata_df=pd.DataFrame())
        inference_dataset.non_tof_scaler = NON_TOF_SCALER
        inference_dataset.tof_scaler = TOF_SCALER
        
        engineered_features = inference_dataset._feature_engineering(full_seq)
        norm_features = inference_dataset.normalize_features(engineered_features)

        tof_dim = inference_dataset.tof_dim
        imu_features_processed = norm_features[:, :20]
        thm_features_processed = norm_features[:, 20:25]
        demo_features_processed = norm_features[0, 25:32]
        tof_features_processed = norm_features[:, 32:]
        thm_tof_features = np.concatenate([thm_features_processed, tof_features_processed], axis=1)

        DEVICE = next(MODELS[0].parameters()).device
        imu_tensor = torch.tensor(imu_features_processed, dtype=torch.float32).unsqueeze(0).to(DEVICE)
        thm_tof_tensor = torch.tensor(thm_tof_features, dtype=torch.float32).unsqueeze(0).to(DEVICE)
        demo_tensor = torch.tensor(demo_features_processed, dtype=torch.float32).unsqueeze(0).to(DEVICE)
        
        with torch.no_grad():
            all_preds = []
            for model in MODELS:
                outputs = model(imu_tensor, thm_tof_tensor, demo_tensor)
                probs = F.softmax(outputs, dim=1)
                all_preds.append(probs)
            
            avg_pred = torch.stack(all_preds).mean(dim=0)
            pred_idx = avg_pred.argmax(dim=1).item()
            pred_label = GESTURE_ENCODER.inverse_transform([pred_idx])[0]

            is_destructive = pred_label in TARGET_GESTURES

            response_data = {
                'predicted_gesture': pred_label,
                'is_destructive_behavior': is_destructive
            }

            log_entry = PredictionLog(gesture_name=pred_label, is_destructive=is_destructive)
            log_entry.save()

            return JsonResponse(response_data)

    except Exception as e:
        import traceback
        return JsonResponse({'error': str(e), 'traceback': traceback.format_exc()}, status=400)
    
def dashboard_view(request):
    """Apenas renderiza a página HTML do nosso dashboard."""
    return render(request, 'api/dashboard.html')

def get_latest_predictions(request):
    """Fornece as 10 últimas predições como JSON."""
    logs = PredictionLog.objects.all()[:10] # Pega os 10 mais recentes
    
    # Converte os objetos do Django para um formato JSON simples
    data = [
        {
            'gesture_name': log.gesture_name,
            'is_destructive': log.is_destructive,
            'predicted_at': log.predicted_at.strftime('%H:%M:%S') # Formata a hora
        } 
        for log in logs
    ]
    
    return JsonResponse(data, safe=False)