import json
import base64
from channels.generic.websocket import AsyncWebsocketConsumer
from django.core.files.base import ContentFile
from django.core.files.storage import default_storage
import algoritmos_robot 
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
            # Obtener los valores de los sliders enviados por el cliente
            sliders = data.get('sliders', {})
            slider1 = int(sliders.get('slider1', 0))  
            slider2 = int(sliders.get('slider2', 0))  
            slider3 = int(sliders.get('slider3', 0))  
            slider4 = int(sliders.get('slider4', 0))  

            # Mostrar los valores de los sliders en la consola (para propósitos de depuración)
            print(f"Valores de sliders recibidos: Slider 1: {slider1}, Slider 2: {slider2}, Slider 3: {slider3}, Slider 4: {slider4}")


            # Enviar una respuesta al cliente indicando que se recibieron los valores de los sliders
            await self.send(text_data=json.dumps({
                'action': 'sliders_received',
                'sliders': {
                    'slider1': slider1,
                    'slider2': slider2,
                    'slider3': slider3,
                    'slider4': slider4
                }
            }))


            #En esta seccion se hace la conecccion con el sistema que se comunica con el periferico del Robot 
            algoritmos_robot.procesarDatos(slider1, slider2, slider3, slider4, lambda q1, numerototal: self.captura_imagen(q1, numerototal))
            
        elif action == 'save_image':
            image_data = data.get('image')
            if image_data:
                # Guardar imagen en el servidor
                self.save_image(image_data)
                await self.send(text_data=json.dumps({'action': 'image_saved'}))

        else:
            # Enviar mensaje a todos los miembros del grupo para otras acciones
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
        
        # Guardar la imagen usando el almacenamiento por defecto de Django
        file_path = default_storage.save(f'captured_images/capture.{ext}', image_data)
        print(f'Imagen guardada en: {file_path}')

    def captura_imagen(self, q1, numerototal):
        print('Ejecución de la función de captura de imagen de manera correcta')
        print(q1, numerototal)
