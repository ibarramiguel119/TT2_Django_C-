import json
from channels.generic.websocket import AsyncWebsocketConsumer

class CaptureConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.channel_layer.group_add("capture_group", self.channel_name)
        await self.accept()

    async def disconnect(self, close_code):
        await self.channel_layer.group_discard("capture_group", self.channel_name)

    async def receive(self, text_data):
        data = json.loads(text_data)
        action = data['action']

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
        await self.send(text_data=json.dumps({
            'action': action
        }))