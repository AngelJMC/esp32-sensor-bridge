

import random
import time
import json
from paho.mqtt import client as mqtt_client
from faker import Faker
import time
import asyncio


sensors_dict = {
    'nh3': { 'unit' : "ppb", 'range': [100, 600] },
    'ch4': { 'unit' : "ppb", 'range': [200, 1000] },
    'h2s': { 'unit' : "ppb", 'range': [10, 100] },
    'temperature': { 'unit' : "C", 'range': [20, 30] },
    'humidity': { 'unit' : "%", 'range': [0, 100]}
}


fake = Faker()

class device:
    def __init__(self, device_id, token, measures):
        self.broker = "industrial.api.ubidots.com"
        self.username =token 
        self.password = ""
        self.port = 1883
        self.client_id = fake.name()
        self.device_id = device_id
        self.measures = measures
        self.gps = { "lat" : fake.latitude(), "lng" : fake.longitude() }
        self.client = mqtt_client.Client(self.client_id)

    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        
        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = on_connect
        self.client.connect(self.broker, self.port)
    
    def publish_data( self, dictframe ):
        json_msg = json.dumps( dictframe )
        topic = '/v2.0/devices/' + self.device_id
        result = self.client.publish(topic, json_msg)
        
        if result[0] == 0:
            print(f"Send `{json_msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")

    def get_sensor(self, name ):
        measure =  {}
        try: 
            scale = 10
            value = random.randint( sensors_dict[name]["range"][0]*scale, 
                                    sensors_dict[name]["range"][1]*scale )
            value = value/scale
            measure =  {
                "value": value, 
                "context": { 
                        "unit": sensors_dict[name]["unit"], 
                        "status": "ok"  
                    },
            }
        except:
            print('Error: measurement name not found in the dictionary')
        return measure

    async def publish_device_measurements(self, period ):
        while True:
            dictframe = {}
            for measure in self.measures:
                dictframe[measure] = self.get_sensor(measure)
            dictframe["timestamp"] =  int(time.time()*1000)
            self.publish_data( dictframe )
            await asyncio.sleep(period)
            
    async def publish_device_info(self, period ):
        while True:
            dictframe = {}
            dictframe["location"] = { "lat": float(self.gps["lat"]), "lng": float(self.gps["lng"]) }
            dictframe["timestamp"] =  int(time.time()*1000)
            self.publish_data( dictframe )
            await asyncio.sleep(period)

    async def publish_device_status(self,period):
        while True:
            dictframe = {}
            dictframe["batt"] = float(3.7),
            dictframe["relay_1"] = "on"
            dictframe["relay_2"] = "off"
            dictframe["timestamp"] =  int(time.time()*1000)
            self.publish_data( dictframe )
            await asyncio.sleep(period)

    async def run( self ):
        self.connect_mqtt()
        self.client.loop_start()
        await asyncio.gather( 
            self.publish_device_measurements( period = 60 ),
            self.publish_device_info( period = 300 ),
            self.publish_device_status( period = 120 )
        )


async def main():
    dev1 = device( "a16c41bc003e", "BBFF-rJrJeXzHGdW8QnRiW8Wcii4DDzjjRd", ["ch4","temperature"])
    dev2 = device( "55c2f491a374", "BBFF-0MLrgWp1tUGu5QF7WFXsnZkfSdyDqn", ["nh3","temperature"])
    dev3 = device( "af9247e90c34", "BBFF-cU4SsABD037CosY7YLzFZ6VNqYEbBQ", ["h2s","temperature"])
    await asyncio.gather(dev1.run(), dev2.run(), dev3.run())
    #await asyncio.gather(dev1.run() )






if __name__ == "__main__":
    asyncio.run(main())