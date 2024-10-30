from celery import shared_task
from channels.layers import get_channel_layer
from asgiref.sync import async_to_sync
import json
import logging
import algoritmos_robot

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