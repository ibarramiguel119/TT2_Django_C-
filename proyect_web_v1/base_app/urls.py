"""proyect_web_v1 URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/4.1/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""
from django.contrib import admin
from django.urls import path
from . import views

from django.conf import settings
from django.conf.urls.static import static

urlpatterns = [
    path('', views.home_system, name='mi_vista'),

    path('home_send_data/',views.send_data,name='home_send_data'),

    path('home_recibe_data/',views.recibe_data,name='home_recibe_data'),

    path('suma/', views.suma_view, name='suma'),
    
    path('calcular-grados/', views.calcular_grados_view, name='calcular_grados'),

    path('enviar_datos_serial/',views.send_data,name='Enviar_datos ')
    
]+ static(settings.MEDIA_URL,document_root=settings.MEDIA_ROOT)

