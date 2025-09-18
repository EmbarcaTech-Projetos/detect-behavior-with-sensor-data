from django.apps import AppConfig
import sys

class ApiConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'api'

    def ready(self):
        if 'runserver' in sys.argv:
            from . import ml_loader
            ml_loader.load_artifacts()