Quick-start guide
=================

This guide will let you quickly start using ros2\_domain\_bridge. To start off, we need to select our configuration based on the intention of using this package.

If you are intending on using this as a domain bridge, the most proficient way would be to transfer topics locally on your computer. That would base both the client and server running on the same computer, but on different ROS\_DOMAIN\_ID's. In this case, the server\_ip setting would be set to '127.0.0.1'.

If you are intending on transferring between computers, set the IP of the server computer as something memorable and static. This will later be the setting for server\_ip. If you are using a VPN tunnel or other means of transferring data across the internet, make sure the server computer's IP is static.

If you are planning on transferring messages using Bluetooth, you first need to connect the server and client devices. Once this is done, find the MAC address of the Bluetooth attached to your server device. This is done in Linux by running 'hciconfig' in the terminal. Alternatively, just run the 'check\_bluetooth\_MAC\_and\_ports.py' script located inside the project folder. This will give you the MAC address and busy ports on the device. The MAC address will later be used as the server\_ip variable.

Server setup
^^^^^^^^^^^^
To configure the server, simply head into 'src/rsb\_server'. 

First off, we will configure the 'bringup.yaml' file, located inside the config folder. Simply set the server\_ip to be either an IP address or MAC address, based on your intention.

Navigate back into 'rsb\_server' and enter 'launch'. Locate the 'server.launch.py' file. First, we need to set the name of the process. This name has to match with the client, and can be used as a prefix to the topics received. Secondly, set the server\_port variable. This variable has to be an open port, best suited to any number between 49152 and 65535. If you are using Bluetooth, the ports to be used should not be listed when running the 'check\_bluetooth\_MAC\_and\_ports.py' script. Next, we select whether or not to use the name as a prefix for incoming topics. As an example, if we set the name to be 'robot1', and we are to receive a topic '/goal\_pose', the name of the incoming topic would be '/robot1/goal\_pose'. We also have to select if we want to use encryption or not. The encryption key could be any 32 url-safe base64-encoded bytes object, which can easily be generated using the 'generate\_key.py' script in the main folder. This has to match with the client.

That's it for server setup. The topics to be transferred and their settings are all defined client-side.


Client setup
^^^^^^^^^^^^
Head to 'src/rsb\_client'

First, we will configure the 'bringup.yaml' file inside the 'config' folder. As with the server, we need to set the server\_ip and server\_port variables. We then need to select which topics to transmit and receive. Every setting exept for \*\_ports should be inside a string. The receive\_\* settings are for topics which are received on the client, and transmit\_\* are for topics which are sent from the client to the server. Each topic has its belonging message type, added to the \*\_msg\_types variable. The port to be used is in the \*\_ports list, the protocol for transmission is in the \*\_protocols list, and finally the QoS to be used by the publisher is in the \*\_qos list. The first inquiry in each list is related. As an example, here is a configuration which sends '/scan', '/odom' and '/shutdown' topics, and receives '/initialpose', '/goal\_pose' and '/shutdown'.

.. code-block:: rts

    receive_topics: ['scan', 'odom', 'shutdown']
    receive_msg_types: ['LaserScan', 'Odometry', 'String']
    receive_ports: [12004, 12005, 12006]
    receive_protocols: ['UDP', 'UDP', 'TCP']
    receive_qos: ['qos_profile_sensor_data', 'qos_profile_sensor_data', '10']

    transmit_topics: ['initialpose', 'goal_pose', 'shutdown']
    transmit_msg_types: ['PoseWithCovarianceStamped', 'PoseStamped', 'String']
    transmit_ports: [12002, 12012]
    transmit_protocols: ['UDP', 'UDP', 'TCP']
    transmit_qos: ['qos_profile_system_default', 'qos_profile_system_default', '10']
    
If you are going to use Bluetooth to communicate the topics, you need to set all \*\_protocols to 'BLUETOOTH'.
    
Once the configuration file is finalized, we need to set up the launch file.
Head to 'src/rsb\_client/launch/client.launch.py'. We first need to set the name which must match with the name used in the server node. Additionally, we need to set whether or not to use name as a prefix, to use encryption, and the encryption key which must match with the server.

Once this is done, both server and client should be ready to go.
