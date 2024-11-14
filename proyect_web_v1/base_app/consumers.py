import json
import base64
import uuid
import os
from datetime import datetime
from datetime import datetime
from channels.generic.websocket import AsyncWebsocketConsumer
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
from.tasks import procesar_datos,save_image
from channels.layers import get_channel_layer
from asgiref.sync import async_to_sync
import logging
from asgiref.sync import sync_to_async
logger = logging.getLogger(__name__)



class CaptureConsumer(AsyncWebsocketConsumer):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.session_folder = None


    async def connect(self):
        # Connect the consumer to the WebSocket group
        await self.channel_layer.group_add("capture_group", self.channel_name)
        await self.accept()
        now = datetime.now()
        self.session_folder = now.strftime('%Y-%m-%d_%H-%M-%S')

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
                procesar_datos.delay(slider1, slider2, slider4, slider3)
                

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
         
        if action == 'send.imagen_clint':
            # Maneja el mensaje enviado desde Celery
            image_url = data.get('image_url')
            session_folder = data.get('session_folder')

            # Aquí puedes enviar un mensaje de vuelta al cliente WebSocket
            await self.send(text_data=json.dumps({
                'action': 'capturar_imagen',
                'image_url': image_url,
                'session_folder': session_folder,
            }))


        
        # Handle save_image action
        elif action == 'save_image':
            image_data = data['image']
            save_image.delay(image_data,self.session_folder,)
            
 

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




    
