#!/usr/bin/env python3

# Copyright 2021 Morten Melby Dahl.
# Copyright 2021 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from cryptography.fernet import Fernet
from datetime import datetime

key = Fernet.generate_key().decode('utf-8')

now = datetime.now()
dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

with open('key.txt', 'w') as file:
	file.write('Key generated at ' + dt_string + ':\n\n')
	file.write(key)