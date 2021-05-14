Internal workings
=================

The only meaningful difference between the server and client is that the ports are bound server-side, and that the server-client relationship is configured on the client. Both server and client reads from a configuration file named 'bringup.yaml', and additional settings such as the encryption key is set in the launch files. 

Initialization
^^^^^^^^^^^^^^

The configuration file of the client contains all settings for transmitting and receiving messages from the server. Topics that are set to be received or transmitted are added to a string, and sent as a byte object to the server using the main communication line. From here, both server and client creates BridgeObjects for each topic to be sent or received. These BridgeObjects contain information about the connection such as direction, encryption key, topic name, message type, port, protocol, qos and encryption setting. When starting connections, socket and connection objects are also attached to the BridgeObject. In addition, a callback function exists so that the BridgeObject is referenced when creating subscribers for outgoing topics.

.. image:: images/data_pathway.png
  :width: 600
  :alt: Alternative text

Topics that are to be received are running on threads which contain a while-loop, receiving function, deserializer, decryptor and publisher. Once a serialized and encrypted message is received, it is first decrypted, deserialized and then published to its correct topic.

Topics that are to be sent is only started as a subscriber with the callback function being inside the BridgeObject belonging to said topic. The callback function serializes, encrypts and sends based on protocol.


UDP sockets
^^^^^^^^^^^
When using UDP as transmission protocol, sockets are set up with the following settings;

.. code-block:: python

    obj.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    obj.soc.settimeout(15)
    obj.soc.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.BUFFER_SIZE)

It is an UDP (SOCK_DGRAM) socket using IPv4 (AF_INET), with a buffer-size of self.BUFFER_SIZE. The timeout is set to 15 seconds. Every 15 seconds of no data, the user receives a warning that the topic is stale. Once the warning has been given 5 times, the warnings stop.


TCP sockets
^^^^^^^^^^^
The TCP sockets use the following settings;

.. code-block:: python

    obj.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    obj.soc.settimeout(15)
    obj.soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

SOCK_STREAM indicates TCP, and it is set to SO_REUSEADDR. This allows reconnection to said socket, which is useful in cases where the socket needs to be rebooted.

TCP sockets are streaming sockets, meaning that they continually sends data. To ensure that the data we receive is split in the correct manner, a b'_split_' is added to the end of each message when sent. On the receiving end, a buffer continually checks to see if '_split_' is in the buffer. If it is, the buffer is split into two parts. The first part is out message, and the second part is the start of a new message. The first part is decrypted, deserialized and published, while the second part is re-added to the now empty buffer.

Bluetooth sockets
^^^^^^^^^^^^^^^^^
As with TCP, SOCK_STREAM is used. This is due to the low reliability of Bluetooth, ensuring that the messages we receive are intact. In addition, BTPROTO_RFCOMM ensure that the address we pass in is (bdaddr, channel), where as it normally is (ip, port).

.. code-block:: python

    obj.soc = socket.socket(socket.AF_BLUETOOTH,socket.SOCK_STREAM,socket.BTPROTO_RFCOMM)
    obj.soc.settimeout(15)
    obj.soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

Bluetooth sockets also use a buffer, just as TCP sockets do.