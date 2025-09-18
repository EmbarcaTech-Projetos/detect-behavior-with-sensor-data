from django.db import models

# Modelo para guardar o histórico de predições para o dashboard
class PredictionLog(models.Model):
    gesture_name = models.CharField(max_length=100)
    is_destructive = models.BooleanField()
    predicted_at = models.DateTimeField(auto_now_add=True)

    # Ordena os resultados por data, do mais novo para o mais antigo
    class Meta:
        ordering = ['-predicted_at'] 

    def __str__(self):
        return f"{self.gesture_name} (Destructive: {self.is_destructive}) at {self.predicted_at}"