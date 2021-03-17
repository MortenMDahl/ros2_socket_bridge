# ros2_domain_bridge

**README and the project is still under development. Any feedback is appreciated! Contact me at mortemd@stud.ntnu.no.**

The ros2_domain_bridge package is made to enable communication across different domains set with 

```export ROS_DOMAIN_ID=*```. 

## Installation

Install from source as with any other ROS2 package - download and run ```colcon build```, then source the setup.bash file.

### Requirements
* Python 3.6+
* ROS2 'Foxy Fitzroy'
* 'Cryptography' Python package (pip install cryptography)


## Usage

The use case for this package is only set by imagination, however it does require a little bit of configuration to work with any topic.

### Generating an encryption key
Due to security risks using the pickle package, all messages are encrypted when sent. To encrypt messages, an encryption key is needed. This can be any 32 url-safe base64-encoded bytes object, and you can generate one by running the ```generate_key.py``` script located in the main folder of ros2_domain_bridge.

Navigate to the ros2_domain_bridge folder and run
```
python3 generate_key.py
```
A file named 'key.txt' containing your key should appear in the main folder. It is important that both the server and the client has the same key to be able to decrypt messages. Where to set the encryption key will be explained in the setup sections.

### Server setup
The server requires that you set ```server_ip``` in the 'src/rdb_server/config/bringup.yaml' file to the IP of your computer which is running the server node. 

After this, open the launch file 'src/rdb_server/launch/server.launch.py'. This is where you select the robot name (which must match with the client), the port to be used as main communication between server and client, and finally your encryption key.

This is all the setup required to run the server. Topics to be transmitted or received, protocols and QoS is all set at the client.

### Client setup
The main client setup is done in the 'src/rdb_client/config/bringup.yaml' file. In the file you will find a detailed description of all variables required. 

First, set the ```server_ip``` and ```server_port``` variables. These must match with whatever you set up in the server.

Everything starting with ```receive_``` will be directed to topics that are going to be sent from the server to the client. The topic name, message type, port, protocol and QoS are required in order to make the node work. 

All settings starting with ```transmit_``` will be sent from the client to the server. The same settings are required as with the receive topics.

Configure the launch file by navigating to 'src/rdb_client/launch/client.launch.py'. Insert the robot_name and encryption key to match with the server.


After configuring both server and client, remember to build and source the setup file.

### Example
In its default form, the server and client is set up to work with the Turtlebot3 example belonging to nav2_bringup. If you have this install, simply open four different terminals. Two of these are your server-side terminals, while the other two are client-side.

In two of the terminals, run

```export ROS_DOMAIN_ID=42```

And in the other two, run

```export ROS_DOMAIN_ID=10```

You should now have two independent ROS domains running. Topics on one domain can not be seen by the other.

On one of the server-side terminals, run

```
ros2 launch rdb_server server.launch.py
```
You should now see that the server is waiting for a connection from the client.

In one of the client-side terminals, run
```
ros2 launch nav2_bringup tb3_simulation_launch.py
```
In the remaining client-side terminal, run
```
ros2 launch rdb_client client.launch.py
```

Connection should be started between the server and client. The server should confirm that it received an init message, that the init message is matching, and that the receiving topics have connected.

On the available server-side terminal, run ```ros2 topic list```. You should see '/tester/scan' and '/tester/odom'. These are the topics received from the client.

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[APACHE LICENSE, VERSION 2.0](https://www.apache.org/licenses/LICENSE-2.0)
