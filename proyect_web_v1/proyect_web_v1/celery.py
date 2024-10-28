import os
from celery import Celery

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'proyect_web_v1.settings')  # Cambia myproject por tu nombre de proyecto

app = Celery('proyect_web_v1')
app.config_from_object('django.conf:settings', namespace='CELERY')
app.autodiscover_tasks()