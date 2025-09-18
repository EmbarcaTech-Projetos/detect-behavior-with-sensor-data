import torch
import torch.nn as nn
import torch.nn.functional as F

class SEBlock(nn.Module):
    def __init__(self, channels, reduction=8):
        super().__init__()
        self.squeeze = nn.AdaptiveAvgPool1d(1)
        self.excitation = nn.Sequential(
            nn.Linear(channels, channels // reduction, bias=False),
            nn.ReLU(inplace=True),
            nn.Linear(channels // reduction, channels, bias=False),
            nn.Sigmoid()
        )
    def forward(self, x):
        b, c, _ = x.size()
        y = self.squeeze(x).view(b, c)
        y = self.excitation(y).view(b, c, 1)
        return x * y.expand_as(x)
    

class CoordAttention(nn.Module):
    def __init__(self, channels, reduction=8):
        super(CoordAttention, self).__init__()
        self.mid_channels = max(8, channels // reduction)

        self.compression = nn.Sequential(
            nn.Conv1d(channels, self.mid_channels, kernel_size=1, bias=False),
            nn.BatchNorm1d(self.mid_channels),
            nn.SiLU(inplace=True)
        )
        self.time_conv = nn.Conv1d(1, 1, kernel_size=5, padding=2, bias=False)  
        self.channel_conv = nn.Conv1d(self.mid_channels, channels, kernel_size=1, bias=False)
        self.sigmoid = nn.Sigmoid()
    def forward(self, x):
        x = x.permute(0, 2, 1)
        # x: (B, T, C)
        x_p = x.permute(0, 2, 1)  # (B, C, T)
        f   = self.compression(x_p)  # (B, rC, T)
        f_t = f.mean(dim=1, keepdim=True)      
        time_attn = self.sigmoid(self.time_conv(f_t))  
        f_c = f.mean(dim=2, keepdim=True)      
        channel_attn = self.sigmoid(self.channel_conv(f_c)) 
        ## (B, T, C)
        out = (x_p * time_attn * channel_attn).permute(0,2,1)
        return out.permute(0, 2, 1)


class ResidualCNNBlock(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, Model, reduction=8, pool_size=2, dropout=0.3, weight_decay=1e-4):
        super().__init__()
        self.conv1 = nn.Conv1d(in_channels, out_channels, kernel_size, padding=kernel_size//2, bias=False)
        self.bn1 = nn.BatchNorm1d(out_channels)
        self.conv2 = nn.Conv1d(out_channels, out_channels, kernel_size, padding=kernel_size//2, bias=False)
        self.bn2 = nn.BatchNorm1d(out_channels)
        self.attention = Model(out_channels, reduction)
        self.shortcut = nn.Sequential()
        if in_channels != out_channels:
            self.shortcut = nn.Sequential(
                nn.Conv1d(in_channels, out_channels, 1, bias=False),
                nn.BatchNorm1d(out_channels)
            )
        self.pool = nn.MaxPool1d(pool_size)
        self.dropout = nn.Dropout(dropout)
        
    def forward(self, x):
        shortcut = self.shortcut(x)
        out = F.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        out = self.attention(out)
        out += shortcut
        out = F.relu(out)
        out = self.pool(out)
        out = self.dropout(out)
        
        return out

class AttentionLayer(nn.Module):
    def __init__(self, hidden_dim):
        super().__init__()
        self.attention = nn.Linear(hidden_dim, 1)
    def forward(self, x):
        scores = torch.tanh(self.attention(x))  
        weights = F.softmax(scores.squeeze(-1), dim=1)  
        context = torch.sum(x * weights.unsqueeze(-1), dim=1) 
        return context

class MLPAttention(nn.Module):
    def __init__(self, feature_dim):
        super(MLPAttention, self).__init__()
        self.attn = nn.Sequential(
            nn.Linear(feature_dim, feature_dim//8),
            nn.SiLU(inplace=True),
            nn.Linear(feature_dim//8, 1)
        )
    def forward(self, x):
        # inputs: (B, T, C)
        weights = self.attn(x)  # (B, T, 1)
        weights = F.softmax(weights, dim=1)  # (B, T, 1)
        context = (x * weights).sum(dim=1)  # (B, C)
        return context


class IMUOnlyModel(nn.Module):
    def __init__(self, imu_dim, tof_dim, n_classes, weight_decay=1e-4):
        super().__init__()
        self.imu_dim = imu_dim
        self.tof_dim = tof_dim
        self.n_classes = n_classes
        self.weight_decay = weight_decay
        self.imu_block1 = ResidualCNNBlock(imu_dim, 64, 3, dropout=0.3, Model=CoordAttention,  weight_decay=weight_decay)
        self.imu_block2 = ResidualCNNBlock(64, 128, 5, dropout=0.3, Model=CoordAttention, weight_decay=weight_decay)
    
        self.bigru = nn.GRU(128, 128, bidirectional=True, batch_first=True)
        self.gru_dropout = nn.Dropout(0.4)
        
        self.attention = AttentionLayer(256)  
        self.mlp_attention = MLPAttention(256) 
        
        self.dense1 = nn.Linear(256, 256, bias=False)
        self.bn_dense1 = nn.BatchNorm1d(256)
        self.drop1 = nn.Dropout(0.5)
        self.dense2 = nn.Linear(256, 128, bias=False)
        self.bn_dense2 = nn.BatchNorm1d(128)
        self.drop2 = nn.Dropout(0.3)
        self.classifier = nn.Linear(128, n_classes)
        
    def forward(self, imu, thm_tof, demo):
        imu = imu.transpose(1, 2)  
        x1 = self.imu_block1(imu)
        x1 = self.imu_block2(x1)
        merged = x1.transpose(1, 2)  
        gru_out, _ = self.bigru(merged)
        gru_out = self.gru_dropout(gru_out)
        attended = self.mlp_attention(gru_out)
        x = F.relu(self.bn_dense1(self.dense1(attended)))
        x = self.drop1(x)
        x = F.relu(self.bn_dense2(self.dense2(x)))
        x = self.drop2(x)
        logits = self.classifier(x)
        return logits