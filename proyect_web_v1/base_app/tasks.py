# tasks.py en la misma carpeta que tu archivo de configuración
from celery import shared_task
import algoritmos_robot


@shared_task
def procesar_datos(slider1, slider2, slider3, slider4):
    # Aquí se define la función que captura la imagen
    def captura_imagen(q1, numerototal):
        # Aquí puedes manejar la lógica de captura de imagen
        print(f'Ejecutando captura de imagen con q1: {q1}, numerototal: {numerototal}')

    # Llama a la función de procesamiento de datos, pasando la función de captura de imagen como argumento
    result = algoritmos_robot.procesarDatos(slider1, slider2, slider3, slider4,lambda q1, numerototal:captura_imagen(q1,numerototal))
    return result