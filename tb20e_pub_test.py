import roslibpy
import time


def main():
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    # publisher = roslibpy.Topic(client, '/tb20e/boom/cmd', 'std_msgs/msg/Float64')
    publisher = roslibpy.Topic(client, '/zx200/tracks/cmd_vel', 'geometry_msgs/Twist')

    counter = 0
    while client.is_connected:
        # message = roslibpy.Message({'data': -counter * 0.1})
        message = roslibpy.Message({'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}})
        publisher.publish(message)
        # print(f'Published message: {message["data"]}')
        time.sleep(1)
        counter += 1
    
    publisher.unadvertise()
    client.terminate()


if __name__ == "__main__":
    main()