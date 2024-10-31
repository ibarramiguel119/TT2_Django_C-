
import json
import base64
from channels.generic.websocket import AsyncWebsocketConsumer
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
from.tasks import procesar_datos
from channels.layers import get_channel_layer
from asgiref.sync import async_to_sync
import logging

logger = logging.getLogger(__name__)


class CaptureConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        # Connect the consumer to the WebSocket group
        await self.channel_layer.group_add("capture_group", self.channel_name)
        await self.accept()

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
                }
            }))

            # Send capture request to Celery
            procesar_datos.delay(slider1, slider2, slider3, slider4)

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
        # Extraer el formato y el contenido de la imagen
        format, imgstr = image_data.split(';base64,')
        ext = format.split('/')[-1]
        image_data = ContentFile(base64.b64decode(imgstr), name='capture.' + ext)
        
        # Guarda la imagen usando el almacenamiento por defecto de Django
        file_path = default_storage.save(f'captured_images/capture.{ext}', image_data)
        print(f'Imagen guardada en: {file_path}')    