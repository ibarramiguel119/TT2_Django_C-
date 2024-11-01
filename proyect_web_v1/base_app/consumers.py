
import json
import base64
from channels.generic.websocket import AsyncWebsocketConsumer
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
from.tasks import procesar_datos
from channels.layers import get_channel_layer
from asgiref.sync import async_to_sync
import logging
import os
from datetime import datetime

logger = logging.getLogger(__name__)


class CaptureConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        # Connect the consumer to the WebSocket group
        await self.channel_layer.group_add("capture_group", self.channel_name)
        await self.accept()
        self.capture_folder = None  # Variable para guardar el nombre de la carpeta de sesión actual

    async def disconnect(self, close_code):
        # Remove the consumer from the WebSocket group
        await self.channel_layer.group_discard("capture_group", self.channel_name)

    async def receive(self, text_data):
        # Receive messages from the WebSocket
        data = json.loads(text_data)
        logger.info(f"Received data: {data}")  # Logging the received data for debugging
        action = data.get('action')
        

        # Handle start_capture action
        if action == 'start_capture':
            # Crear una carpeta para la nueva sesión de captura usando timestamp
            self.capture_folder = f"captured_images/session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            capture_folder_path = default_storage.path(self.capture_folder)
            os.makedirs(capture_folder_path, exist_ok=True)  # Asegúrate de que la carpeta se crea


            radio_option = data.get('radio_option', None)  # Obtiene el valor del radio button

            if radio_option=='1':
            
                sliders = data.get('sliders', {})
                slider1 = int(sliders.get('slider1', 0))
                slider2 = int(sliders.get('slider2', 0))
                slider3 = int(sliders.get('slider3', 0))
                slider4 = int(sliders.get('slider4', 0))

                # Send confirmation of sliders received
                await self.send(text_data=json.dumps({
                    'action': 'sliders_received',
                    'sliders': {
                        'slider1': slider1,
                        'slider2': slider2,
                        'slider3': slider3,
                        'slider4': slider4
                    },
                    'radio_buton':radio_option,
                }))

                # Send capture request to Celery
                procesar_datos.delay(slider1, slider2, slider3, slider4)
            elif radio_option=='2':
                select_option = data.get('select_option') 
                
                await self.send(text_data=json.dumps({
                    'action': 'data_recibe',
                    'select_option':select_option,
                }))
                if select_option=="opcion_27":
                    Naltitud=3
                    Nzimut=3
                    NRoll=3
                    Radio=150
                    await self.send(text_data=json.dumps({
                        'action': 'data_select_27',
                        'select_option':select_option,
                    }))
                    procesar_datos.delay(Naltitud, Nzimut, NRoll, Radio)


                elif select_option=="opcion_50":
                    Naltitud=3
                    Nzimut=3
                    NRoll=3
                    Radio=150
                    await self.send(text_data=json.dumps({
                        'action': 'data_select_50',
                        'select_option':select_option,
                    }))
                    procesar_datos.delay(Naltitud, Nzimut, NRoll, Radio)

                elif select_option == "opcion_70":
                    Naltitud=3
                    Nzimut=3
                    NRoll=3
                    Radio=150
                    await self.send(text_data=json.dumps({
                        'action': 'data_select_70',
                        'select_option':select_option,
                    }))
                    procesar_datos.delay(Naltitud, Nzimut, NRoll, Radio)


        # Handle captura.imagen action
        elif action == 'captura.imagen':
            # Manejar el mensaje de captura de imagen
            message = data.get('message')
            q1 = data.get('q1', 0)
            numerototal = data.get('numerototal', 0)

            # Envía el mensaje al cliente
            await self.send(text_data=json.dumps({
                'action': message,
                'q1': q1,
                'numerototal': numerototal
            }))
        # Handle save_image action
        elif action == 'save_image':
            image_data = data['image']
            # Save image on the server
            self.save_image(image_data)
            await self.send(text_data=json.dumps({'action': 'image_saved'}))    

    async def receive_celery(self, text_data):
        # This method can be designed to handle messages sent from Celery to the WebSocket
        data = json.loads(text_data)
        logger.info(f"Received from Celery: {data}")
        
        # Here you can send data back to the client if necessary
        await self.send(text_data=json.dumps(data))

    async def captura_imagen(self, event):
        # Manejar el mensaje de captura de imagen
        message = event['message']
        q1 = event.get('q1', 0)
        numerototal = event.get('numerototal', 0)

        # Envía el mensaje al cliente
        await self.send(text_data=json.dumps({
            'action': message,
            'q1': q1,
            'numerototal': numerototal
        }))

    
    def save_image(self, image_data):
        if not self.capture_folder:
            logger.error("Capture folder not set. Cannot save image.")
            return

        # Verificar si la carpeta de captura existe y crearla si no existe
        capture_folder_path = default_storage.path(self.capture_folder)
        if not os.path.exists(capture_folder_path):
            os.makedirs(capture_folder_path, exist_ok=True)

        # Extraer el formato y el contenido de la imagen
        try:
            format, imgstr = image_data.split(';base64,')
            ext = format.split('/')[-1]
            image_data = ContentFile(base64.b64decode(imgstr), name='capture.' + ext)

            # Generar un nombre de archivo único usando un timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S%f')  # Genera un timestamp único
            file_name = f'{self.capture_folder}/capture_{timestamp}.{ext}'  # Nombre de archivo único

            # Guarda la imagen usando el almacenamiento por defecto de Django
            file_path = default_storage.save(file_name, image_data)
            logger.info(f'Imagen guardada en: {file_path}')  # Cambiado de print a logger
        except Exception as e:
            logger.error(f'Error al guardar la imagen: {e}')        