import json
import base64
from channels.generic.websocket import AsyncWebsocketConsumer
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
from.tasks import procesar_datos
from channels.layers import get_channel_layer
from asgiref.sync import async_to_sync
import logging

from asgiref.sync import sync_to_async


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

            radio_option = data.get('radio_option', None)  # Obtiene el valor del radio button

            if radio_option == '1':
            
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
                    'radio_button': radio_option,
                }))

                # Send capture request to Celery
                procesar_datos.delay(slider1, slider2, slider3, slider4)
            elif radio_option == '2':
                select_option = data.get('select_option') 
                
                await self.send(text_data=json.dumps({
                    'action': 'data_received',
                    'select_option': select_option,
                }))
                if select_option == "opcion_27":
                    Naltitud = 3
                    Nzimut = 3
                    NRoll = 3
                    Radio = 150
                    await self.send(text_data=json.dumps({
                        'action': 'data_select_27',
                        'select_option': select_option,
                    }))
                    procesar_datos.delay(Naltitud, Nzimut, Radio, NRoll)

                elif select_option == "opcion_50":
                    Naltitud = 3
                    Nzimut = 3
                    NRoll = 3
                    Radio = 150
                    await self.send(text_data=json.dumps({
                        'action': 'data_select_50',
                        'select_option': select_option,
                    }))
                    procesar_datos.delay(Naltitud, Nzimut, Radio, NRoll)

                elif select_option == "opcion_70":
                    Naltitud = 3
                    Nzimut = 3
                    NRoll = 3
                    Radio = 150
                    await self.send(text_data=json.dumps({
                        'action': 'data_select_70',
                        'select_option': select_option,
                    }))
                    procesar_datos.delay(Naltitud, Nzimut, Radio, NRoll)

        # Handle capture.image action
        elif action == 'captura.imagen':
            # Handle the capture image message
            message = data.get('message')
            q1 = data.get('q1', 0)
            numerototal = data.get('numerototal', 0)

            # Send the message to the client
            await self.send(text_data=json.dumps({
                'action': message,
                'q1': q1,
                'numerototal': numerototal
            }))
        
        # Handle save_image action
        elif action == 'save_image':
            image_data = data['image']
            # Save the image on the server asynchronously
            await self.save_image(image_data)
            await self.send(text_data=json.dumps({'action': 'image_saved'}))    

    async def receive_celery(self, text_data):
        # This method can be designed to handle messages sent from Celery to the WebSocket
        data = json.loads(text_data)
        logger.info(f"Received from Celery: {data}")
        
        # Here you can send data back to the client if necessary
        await self.send(text_data=json.dumps(data))

    async def captura_imagen(self, event):
        # Handle the capture image message
        message = event['message']
        q1 = event.get('q1', 0)
        numerototal = event.get('numerototal', 0)

        # Send the message to the client
        await self.send(text_data=json.dumps({
            'action': message,
            'q1': q1,
            'numerototal': numerototal
        }))

    async def save_image(self, image_data):
        # Extract the format and content of the image
        format, imgstr = image_data.split(';base64,')
        ext = format.split('/')[-1]
        image_content = ContentFile(base64.b64decode(imgstr), name='capture.' + ext)

        # Save the image asynchronously (with sync_to_async to run in a thread)
        file_path = await sync_to_async(self._save_image)(image_content, ext)
        print(f'Image saved at: {file_path}')  

    def _save_image(self, image_content, ext):
        # This method will save the image in Django storage
        file_path = default_storage.save(f'captured_images/capture.{ext}', image_content)
        return file_path