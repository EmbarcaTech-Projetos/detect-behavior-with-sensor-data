from django.urls import path
from . import views

urlpatterns = [
    path('predict/', views.predict, name='predict'),
    path('dashboard/', views.dashboard_view, name='dashboard'),
    path('latest_predictions/', views.get_latest_predictions, name='get_latest_predictions'),
]