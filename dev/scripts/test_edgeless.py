import requests

url = "http://127.0.0.1:7035/"
headers = {
    "Host": "demo.edgeless.com",
    "Content-Type": "application/json"
}
data = {
    "message": "I am here from outside!"
}

response = requests.post(url, headers=headers, json=data)

print(f"Status Code: {response.status_code}")
print(f"Response Text: {response.text}")
