import json
import base64
from channels.generic.websocket import AsyncWebsocketConsumer
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage

class CaptureConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.channel_layer.group_add("capture_group", self.channel_name)
        await self.accept()

    async def disconnect(self, close_code):
        await self.channel_layer.group_discard("capture_group", self.channel_name)

    async def receive(self, text_data):
        data = json.loads(text_data)
        action = data['action']

        if action == 'save_image':
            image_data = data['image']
            # Guardar imagen en el servidor
            self.save_image(image_data)
            await self.send(text_data=json.dumps({'action': 'image_saved'}))
        else:
            # Enviar mensaje a todos los miembros del grupo
            await self.channel_layer.group_send(
                "capture_group",
                {
                    'type': 'capture_message',
                    'action': action
                }
            )

    async def capture_message(self, event):
        action = event['action']

        # Enviar el mensaje a WebSocket
        await self.send(text_data=json.dumps({'action': action}))

    def save_image(self, image_data):
        # Extraer el formato y el contenido de la imagen
        format, imgstr = image_data.split(';base64,')
        ext = format.split('/')[-1]
        image_data = ContentFile(base64.b64decode(imgstr), name='capture.' + ext)
        
        # Guarda la imagen usando el almacenamiento por defecto de Django
        file_path = default_storage.save(f'captured_images/capture.{ext}', image_data)
        print(f'Imagen guardada en: {file_path}')
