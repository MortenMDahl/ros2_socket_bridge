Testing and benchmarking
========================
As this software was made in relation to a masters thesis, testing and benchmarking was done. Time it took to serialize and encrypt, deserialize and decrypt, and maximum topic publishing rate using different protocols and connection modes was tested.

Details about the two computers used during testing can be seen here;

.. image:: images/computers.png
  :width: 600

Serialization and encryption
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. image:: images/se.png
  :width: 600

Decryption and deserialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: images/ddes.png
  :width: 600

Maximum publishing rate
^^^^^^^^^^^^^^^^^^^^^^^
The maximum publishing rate was tested in relation to message size.

Internal transmission using UDP:

.. image:: images/rate_local_UDP.png
  :width: 600

Internal transmission using TCP:

.. image:: images/rate_local_TCP.png
  :width: 600

External (between computers on local network) transmission using UDP:

.. image:: images/rate_external_UDP.png
  :width: 600

External transmission using TCP:

.. image:: images/rate_external_TCP.png
  :width: 600

External transmission using Bluetooth:

.. image:: images/blu_rate.png
  :width: 600