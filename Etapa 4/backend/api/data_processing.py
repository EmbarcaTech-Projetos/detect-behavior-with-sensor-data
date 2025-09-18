import os
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset, DataLoader
from torch.nn.utils.rnn import pad_sequence
from tqdm import tqdm
from pathlib import Path
import pickle
from scipy.spatial.transform import Rotation as R
import warnings
import torch.nn as nn
import torch.optim as optim
import time
import torch.nn.functional as F
from copy import deepcopy
import gc
import random, math
import warnings 
from torch.optim import Adam, AdamW, Adamax
from torch.optim.lr_scheduler import CosineAnnealingWarmRestarts
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.utils.class_weight import compute_class_weight
from sklearn.model_selection import StratifiedKFold
from timm.scheduler import CosineLRScheduler
from scipy.signal import firwin
import polars as pl
from sklearn.model_selection import StratifiedGroupKFold, GroupKFold
import logging
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import classification_report, confusion_matrix

class ToFScaler:
    def __init__(self):
        self.means_full = np.zeros(320)  
        self.stds_full = np.ones(320)    
        self.failure_counts = {s: 0 for s in range(1, 6)}
        self.col_indices = {}
        for sensor in range(1, 6):
            start = (sensor-1)*64
            end = sensor*64
            self.col_indices[sensor] = (start, end)
    
    def fit(self, X_tof):
        tof_data = X_tof.values if isinstance(X_tof, pd.DataFrame) else X_tof
        
        if tof_data.shape[1] != 320:
            raise ValueError(f"Os dados TOF deveriam ter 320 colunas, mas têm {tof_data.shape[1]}")
        
        tof_data = tof_data.reshape(-1, 5, 64)
        
        for sensor in range(5):
            sensor_idx = sensor + 1
            sensor_data = tof_data[:, sensor, :]
            failure_mask = (sensor_data == 0)
            self.failure_counts[sensor_idx] = failure_mask.sum()
            valid_mask = (sensor_data != -1) & (sensor_data != 0)
            valid_data = sensor_data[valid_mask]
            if len(valid_data) > 0:
                mean_val = valid_data.mean()
                std_val = valid_data.std()
            else:
                mean_val = 0
                std_val = 1
            start, end = self.col_indices[sensor_idx]
            self.means_full[start:end] = mean_val
            self.stds_full[start:end] = std_val if std_val > 1e-7 else 1.0
        return self

    def transform(self, X_tof):
        # Get data
        if isinstance(X_tof, pd.DataFrame):
            tof_cols = X_tof.columns
            tof_values = X_tof.values.copy()
        else:
            tof_values = X_tof.copy()
        # Create masks for the entire data at once
        valid_mask = (tof_values != -1) & (tof_values != 0)
        no_signal_mask = (tof_values == -1)
        failure_mask = (tof_values == 0)
        normalized = np.zeros_like(tof_values)
        normalized[valid_mask] = (tof_values[valid_mask] - self.means_full[np.where(valid_mask)[1]]) / self.stds_full[np.where(valid_mask)[1]]
        normalized[no_signal_mask] = 10     # -1 → 10
        normalized[failure_mask]   = -10    # 0 → -10
        if isinstance(X_tof, pd.DataFrame):
            return pd.DataFrame(normalized, columns=tof_cols, index=X_tof.index)
        return normalized



class GestureDataset(Dataset):
    def __init__(self, sequence_dir, label_dir, metadata_df, max_len=None, 
                 is_train=False, scale_path=None, use_augmentation=False,
                 drift_std=0.01, drift_max=0.05):
        self.sequence_dir = sequence_dir
        self.label_dir = label_dir
        self.metadata = metadata_df
        self.max_len = max_len
        self.is_train = is_train
        self.scale_path = scale_path
        self.use_augmentation = use_augmentation and is_train
        self.drift_std = drift_std
        self.drift_max = drift_max

        self.thm_tof_dim = 325
        self.tof_dim = 320
        self.demo_dim = 7
        
        self.gesture_encoder = LabelEncoder()
        self.non_tof_scaler = StandardScaler()
        self.tof_scaler = ToFScaler()

        if not self.metadata.empty:
            self.sequence_ids = self.metadata['sequence_id'].tolist()
            self.gesture_encoder.fit(self.metadata['gesture'])
            self.sequence_data = self._load_sequences()
            
            if is_train and scale_path is not None:
                self._fit_scalers()
                self._save_scalers(scale_path)
                self._normalize_and_cache_sequences()
            elif not is_train and scale_path is not None:
                # Carrega scalers pré-calculados para validação/teste
                if os.path.exists(os.path.join(scale_path, 'train_scalers.pkl')):
                    with open(os.path.join(scale_path, 'train_scalers.pkl'), 'rb') as f:
                        self.non_tof_scaler, self.tof_scaler = pickle.load(f)
                    self._normalize_and_cache_sequences()
                else:
                    warnings.warn("Scaler não encontrado.")
            else:
                warnings.warn("Caminho do scaler não fornecido.")
                self.cached_sequences = self.sequence_data.copy()
            
            print(f"Dataset inicializado com {len(self.sequence_ids)} sequências.")
        

    def _normalize_and_cache_sequences(self):
        self.cached_sequences = []
        for seq in tqdm(self.sequence_data, desc="Normalização de dados sequenciais", leave=False):
            non_tof_part = seq[:, :-self.tof_dim]
            tof_part = seq[:, -self.tof_dim:]
            norm_non_tof = self.non_tof_scaler.transform(non_tof_part)
            norm_tof = self.tof_scaler.transform(tof_part)
            normalized_seq = np.concatenate([norm_non_tof, norm_tof], axis=1)
            self.cached_sequences.append(normalized_seq)
        self.sequence_data = None


    def _load_sequences(self):
        sequence_data = []
        for seq_id in tqdm(self.sequence_ids, desc="Carregar a sequência de dados", leave=False):
            subject_id = self.metadata[self.metadata['sequence_id'] == seq_id]['subject'].iloc[0]
            seq_path = os.path.join(self.sequence_dir, str(subject_id), f"{seq_id}.npy")
            if os.path.exists(seq_path):
                seq = np.load(seq_path)  
                seq = self._feature_engineering(seq)
                sequence_data.append(seq)
        return sequence_data


    def _remove_gravity_from_acc(self, acc_values, quat_values):
        num_samples = acc_values.shape[0]
        linear_accel = np.zeros_like(acc_values)
        gravity_world = np.array([0, 0, 9.81])
        for i in range(num_samples):
            if np.all(np.isnan(quat_values[i])) or np.all(np.isclose(quat_values[i], 0)):
                linear_accel[i, :] = acc_values[i, :] 
                continue
            try:
                rotation = R.from_quat(quat_values[i])
                gravity_sensor_frame = rotation.apply(gravity_world, inverse=True)
                linear_accel[i, :] = acc_values[i, :] - gravity_sensor_frame
            except ValueError:
                linear_accel[i, :] = acc_values[i, :]  
        return linear_accel
    def _calculate_angular_velocity_from_quat(self, quat_values, time_delta=1/10):
        num_samples = quat_values.shape[0]
        angular_vel = np.zeros((num_samples, 3))
        for i in range(num_samples - 1):
            q_t = quat_values[i]
            q_t_plus_dt = quat_values[i+1]
            if np.all(np.isnan(q_t)) or np.all(np.isclose(q_t, 0)) or \
            np.all(np.isnan(q_t_plus_dt)) or np.all(np.isclose(q_t_plus_dt, 0)):
                continue
            try:
                rot_t = R.from_quat(q_t)
                rot_t_plus_dt = R.from_quat(q_t_plus_dt)
                delta_rot = rot_t.inv() * rot_t_plus_dt
                angular_vel[i, :] = delta_rot.as_rotvec() / time_delta
            except ValueError:
                pass
        return angular_vel
    def _calculate_angular_distance(self, quat_values):
        num_samples = quat_values.shape[0]
        angular_dist = np.zeros(num_samples)
        for i in range(num_samples - 1):
            q1 = quat_values[i]
            q2 = quat_values[i+1]
            if np.all(np.isnan(q1)) or np.all(np.isclose(q1, 0)) or \
            np.all(np.isnan(q2)) or np.all(np.isclose(q2, 0)):
                angular_dist[i] = 0
                continue
            try:
                r1 = R.from_quat(q1)
                r2 = R.from_quat(q2)
                relative_rotation = r1.inv() * r2
                angle = np.linalg.norm(relative_rotation.as_rotvec())
                angular_dist[i] = angle
            except ValueError:
                angular_dist[i] = 0
                pass
        return angular_dist



    def _feature_engineering(self, seq):
        imu = seq[:,:7]
        other_features = seq[:, 7:]  
        acc = imu[:,0:3]      # x, y, z
        rot = imu[:,3:7]      # w, x, y, z
        acc_mag = np.sqrt(acc[:,0]**2 + acc[:,1]**2 + acc[:,2]**2)                            
        rot_angle = 2 * np.arccos(rot[:,0].clip(-1, 1))                                       
        acc_mag_jerk = np.diff(acc_mag, prepend=acc_mag[0])                                   
        rot_angle_vel = np.diff(rot_angle, prepend=rot_angle[0])                              
        linear_acc = self._remove_gravity_from_acc(acc, rot[:, [1, 2, 3, 0]])                 
        linear_acc_mag = np.sqrt(linear_acc[:,0]**2 + linear_acc[:,1]**2 + linear_acc[:,2]**2) 
        linear_acc_mag_jerk = np.diff(linear_acc_mag, prepend=linear_acc_mag[0])               
        angular_vel = self._calculate_angular_velocity_from_quat(rot[:, [1, 2, 3, 0]])         
        angular_distance = self._calculate_angular_distance(rot[:, [1, 2, 3, 0]])              
        new_imu = np.concatenate([
            acc,                          # 3
            rot,                          # 4
            acc_mag[:, None],             # 1
            rot_angle[:, None],           # 1
            acc_mag_jerk[:, None],        # 1
            rot_angle_vel[:, None],       # 1
            linear_acc,                   # 3
            linear_acc_mag[:, None],      # 1
            linear_acc_mag_jerk[:, None], # 1
            angular_vel,                  # 3
            angular_distance[:, None]     # 1
        ], axis=1)
        thm = other_features[:, :5]  
        tof = other_features[:, 5:5+320] 
        demo = other_features[:, 5+320:] 
        features = np.concatenate([new_imu, thm,demo,tof], axis=1)
        return features

    def _fit_scalers(self):
        all_sequences = np.vstack(self.sequence_data)
        non_tof_features = all_sequences[:, :-self.tof_dim]
        tof_features = all_sequences[:, -self.tof_dim:]
        self.non_tof_scaler.fit(non_tof_features)
        self.tof_scaler.fit(tof_features) 
        print("Cálculo das estatísticas para normalização do treino finalizado.")

    def _save_scalers(self, scale_path):
        Path(scale_path).mkdir(parents=True, exist_ok=True)
        with open(os.path.join(scale_path, 'train_scalers.pkl'), 'wb') as f:
            pickle.dump((self.non_tof_scaler, self.tof_scaler), f)
        print(f"Salvando estatísticas de normalização do conjunto de treino em {scale_path}/train_scalers.pkl")
        
        encoder_path = os.path.join(scale_path, 'gesture_encoder.pkl')
        with open(encoder_path, 'wb') as f:
            pickle.dump(self.gesture_encoder, f)
        print(f"Encoder de gestos salvo em {encoder_path}")

    def normalize_features(self, sequence):
        non_tof_part = sequence[:, :-self.tof_dim]
        tof_part = sequence[:, -self.tof_dim:]  # TOF(320)
        norm_non_tof = self.non_tof_scaler.transform(non_tof_part)
        norm_tof = self.tof_scaler.transform(tof_part)  
        return np.concatenate([norm_non_tof, norm_tof], axis=1)


    def _jitter(self, sequence, sigma=0.1):
        return sequence + np.random.normal(loc=0., scale=sigma, size=sequence.shape)
    def _time_mask(self, sequence, max_mask_size=25):
        seq_len = sequence.shape[0]
        mask_size = np.random.randint(1, max_mask_size)
        start = np.random.randint(0, max(1, seq_len - mask_size))
        sequence[start : start + mask_size] = 0
        return sequence
    def _feature_mask(self, sequence, max_mask_size=10):
        num_features = sequence.shape[1]
        mask_size = np.random.randint(1, max_mask_size)
        masked_features = np.random.choice(num_features, mask_size, replace=False)
        sequence[:, masked_features] = 0
        return sequence
    def _motion_drift(self, imu_features: np.ndarray) -> np.ndarray:
        T = imu_features.shape[0]
        drift = np.cumsum(np.random.normal(scale=self.drift_std, size=(T, 1)),axis=0)
        drift = np.clip(drift, -self.drift_max, self.drift_max)   
        imu_features[:, 0:3] += drift
        imu_features[:, 10:13] += drift
        imu_features[:, 15:18] += drift
        return imu_features
    def _apply_augmentations(self, sequence):
        if np.random.rand() < 0.7:
            sequence = self._jitter(sequence, sigma=0.05)
        if np.random.rand() < 0.5:
            sequence = self._time_mask(sequence, max_mask_size=20)
        if np.random.rand() < 0.5:
            sequence = self._feature_mask(sequence, max_mask_size=15)
        if np.random.rand() < 0.5:
            imu_features = sequence[:, :20]
            other_features = sequence[:, 20:]
            augmented_imu = self._motion_drift(imu_features)
            sequence = np.concatenate([augmented_imu, other_features], axis=1)
        return sequence


    def __len__(self):
        return len(self.sequence_ids)

    def __getitem__(self, idx):
        sequence = self.cached_sequences[idx].copy()
        if self.use_augmentation:
            sequence = self._apply_augmentations(sequence)
        seq_id = self.sequence_ids[idx]
        subject_id = self.metadata[self.metadata['sequence_id'] == seq_id]['subject'].iloc[0]
        label_path = os.path.join(self.label_dir, str(subject_id), f"{seq_id}.npy")
        phase_labels = np.load(label_path).astype(np.float32)
        if self.max_len and len(sequence) > self.max_len:
            sequence = sequence[-self.max_len:]
            phase_labels = phase_labels[-self.max_len:]
        tof_features = sequence[:, -self.tof_dim:]  
        imu_thm_demo_features = sequence[:, :-self.tof_dim]
        demo_features = imu_thm_demo_features[0, -7:]  
        thm_features = imu_thm_demo_features[:, :-7][:, -5:]  
        imu_features = imu_thm_demo_features[:, :-7][:, :-5]
        thm_tof_features = np.concatenate([thm_features, tof_features], axis=1)
        meta = self.metadata[self.metadata['sequence_id'] == seq_id].iloc[0]
        gesture_label = self.gesture_encoder.transform([meta['gesture']])[0]
        imu_features = torch.tensor(imu_features, dtype=torch.float32)
        thm_tof_features = torch.tensor(thm_tof_features, dtype=torch.float32)
        demo_features = torch.tensor(demo_features, dtype=torch.float32)
        phase_labels = torch.tensor(phase_labels, dtype=torch.long)
        
        return {
            'imu': imu_features,
            'thm_tof': thm_tof_features,
            'demo': demo_features,
            'phase_labels': phase_labels,
            'length': len(imu_features),
            'sequence_id': seq_id,
            'gesture_label': gesture_label
        }


def collate_fn(batch):
    batch.sort(key=lambda x: x['length'], reverse=True)
    imu_features = [item['imu'] for item in batch]
    thm_tof_features = [item['thm_tof'] for item in batch]
    demo_features = torch.stack([item['demo'] for item in batch])
    phase_labels = [item['phase_labels'] for item in batch]
    lengths = torch.tensor([item['length'] for item in batch], dtype=torch.long)
    sequence_ids = [item['sequence_id'] for item in batch]
    gesture_labels = torch.tensor([item['gesture_label'] for item in batch], dtype=torch.long)
    imu_padded = pad_sequence(imu_features, batch_first=True, padding_value=0)
    thm_tof_padded = pad_sequence(thm_tof_features, batch_first=True, padding_value=0)
    phase_padded = pad_sequence(phase_labels, batch_first=True, padding_value=3)
    max_len = imu_padded.size(1)
    mask = torch.arange(max_len)[None, :] < lengths[:, None]
    mask = mask.float()
    
    return {
        'imu': imu_padded,
        'thm_tof': thm_tof_padded,
        'demo': demo_features,
        'phase_labels': phase_padded,
        'mask': mask,
        'lengths': lengths,
        'gesture_labels': gesture_labels,
        'sequence_ids': sequence_ids
    }