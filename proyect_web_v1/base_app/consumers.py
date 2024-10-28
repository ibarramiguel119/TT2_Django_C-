import json
import base64
from channels.generic.websocket import AsyncWebsocketConsumer
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
from .tasks import procesar_datos  # Asegúrate de importar la tarea
import asyncio

class CaptureConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.channel_layer.group_add("capture_group", self.channel_name)
        await self.accept()

    async def disconnect(self, close_code):
        await self.channel_layer.group_discard("capture_group", self.channel_name)

    async def receive(self, text_data):
        data = json.loads(text_data)
        action = data.get('action')

        if action == 'start_capture':
            sliders = data.get('sliders', {})
            slider1 = int(sliders.get('slider1', 0))  
            slider2 = int(sliders.get('slider2', 0))  
            slider3 = int(sliders.get('slider3', 0))  
            slider4 = int(sliders.get('slider4', 0))  

            await self.send(text_data=json.dumps({
                'action': 'sliders_received',
                'sliders': {
                    'slider1': slider1,
                    'slider2': slider2,
                    'slider3': slider3,
                    'slider4': slider4
                }
            }))

            # Llama a la tarea de Celery en lugar de ejecutar la función directamente
            procesar_datos.delay(slider1, slider2, slider3, slider4)

        elif action == 'save_image':
            image_data = data.get('image')
            if image_data:
                self.save_image(image_data)
                await self.send(text_data=json.dumps({'action': 'image_saved'}))

        else:
            await self.channel_layer.group_send(
                "capture_group",
                {
                    'type': 'capture_message',
                    'action': action
                }
            )

    async def capture_message(self, event):
        action = event['action']
        await self.send(text_data=json.dumps({'action': action}))

    def save_image(self, image_data):
        format, imgstr = image_data.split(';base64,')
        ext = format.split('/')[-1]
        image_data = ContentFile(base64.b64decode(imgstr), name='capture.' + ext)
        
        file_path = default_storage.save(f'captured_images/capture.{ext}', image_data)
        print(f'Imagen guardada en: {file_path}')

