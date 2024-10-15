import requests

url = 'http://192.168.232.23:5000/data'  # Replace with Pi's IP

data = {
    'message': 'Hello, Pi!',
    'value': 123
}

response = requests.post(url, json=data)
print(f"Response: {response.text}")
