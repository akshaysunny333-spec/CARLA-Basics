# Connect to a CARLA simulator instance and retrieve the world object
import carla

def connect_to_carla(host='localhost', port=2000, timeout=10.0):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    return client, world
