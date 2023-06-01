import requests
import random
from datetime import datetime
import json

def random_tree():
    return json.dumps({ 'color': random.choice([ 'B', 'R' ]) })

payload = { 
    'date': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    'lat': random.uniform(-90, 90), 
    'lon': random.uniform(-180, 180), 
    'trees': [random_tree() for i in range(random.randrange(0, 3))]    
}
files = { 'photo': open('test.png', 'rb') }

r = requests.post('http://drone.bimbur.art/api', data=payload, files=files)

print(r.text)
