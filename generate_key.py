#!/usr/bin/env python3

from cryptography.fernet import Fernet
from datetime import datetime

key = Fernet.generate_key().decode('utf-8')

now = datetime.now()
dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

with open('key.txt', 'w') as file:
	file.write('Key generated at ' + dt_string + ':\n\n')
	file.write(key)