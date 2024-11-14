from celery import shared_task
from channels.layers import get_channel_layer
from asgiref.sync import async_to_sync
import json
import logging
import algoritmos_robot
import time
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
import base64
import os
import json

from channels.layers import get_channel_layer

logger = logging.getLogger(__name__)

@shared_task
def procesar_datos(slider1, slider2, slider3, slider4):
    # Define la función que envía la señal de captura de imagen
    def captura_imagen(q1, numerototal):
       


        channel_layer = get_channel_layer()
        logger.info("El canal el siguiente: %s", channel_layer)
        try:
            logger.info("Preparando para enviar el mensaje a capture_group")
            message = {
                "type": "captura.imagen",  # Asegúrate de que este tipo sea manejado en tu consumidor
                "message": "capturar_imagen",
                "q1": q1,
                "numerototal": numerototal
            }
            logger.info("Enviando mensaje: %s", message)
            async_to_sync(channel_layer.group_send)("capture_group", message)


            logger.info("Solicitud de captura de imagen enviada con q1: %s, numerototal: %s", q1, numerototal)
        except Exception as e:
            logger.error("Error al enviar solicitud de captura de imagen: %s", e)

    # Procesa los datos y llama a la función de captura de imagen
    result = algoritmos_robot.procesarDatos(slider1, slider2, slider3, slider4, 
                                            lambda q1, numerototal: captura_imagen(q1, numerototal))

    return result

@shared_task
def save_image(image_data, session_folder):
  
    try:
        # Extraer el formato y contenido de la imagen
        format, imgstr = image_data.split(';base64,')  # Imagen en base64
        ext = format.split('/')[-1]  # Obtenemos la extensión de la imagen
        image_content = ContentFile(base64.b64decode(imgstr), name=f'capture.{ext}')

        # Guardamos la imagen en el servidor en una carpeta específica de la sesión
        file_path = _save_image(image_content, ext, session_folder)
        print(f'Image saved at: {file_path}')

        # Obtener la URL de la imagen guardada
        image_url = default_storage.url(file_path)  # Generar la URL pública

        # Enviar la URL de la imagen al cliente a través de WebSocket usando Channels
        # Enviar el mensaje al grupo WebSocket con un 'type' adecuado
        # Después de guardar la imagen, enviar un mensaje con el 'type' al grupo WebSocket
        channel_layer = get_channel_layer()
        logger.info("El canal el siguiente: %s", channel_layer)
        try:
            logger.info("Preparando para enviar el mensaje a capture_group")
            message = {
                "type": "send.imagen_clint",  # Asegúrate de que este tipo sea manejado en tu consumidor
                "message": "capturar_imagen",
                'image_url': image_url,  # Datos asociados al mensaje
                'session_folder': session_folder,
                
            }
            logger.info("Enviando mensaje: %s", message)
            async_to_sync(channel_layer.group_send)("capture_group", message)

        except Exception as e:
            logger.error("Error al enviar la direccion de la carpeta donde se guarda la informacion: %s", e)
    except Exception as e:
        print(f"Error al guardar la imagen: {e}")

def _save_image(image_content, ext, session_folder):
    """
    Método sincronizado para guardar la imagen en una carpeta de sesión
    """
    session_path = os.path.join('captured_images', session_folder)

    # Asegurarse de que la carpeta de sesión exista
    if not os.path.exists(session_path):
        os.makedirs(session_path)

    # Guardar la imagen dentro de la carpeta de sesión
    file_path = default_storage.save(os.path.join(session_path, f'capture.{ext}'), image_content)
    return file_path