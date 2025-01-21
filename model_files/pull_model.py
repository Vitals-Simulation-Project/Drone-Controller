import requests
import json

URL = "http://localhost:11434/api/pull"


# downloads the requested model from Ollama or attempts to update it if it already exists
# returns attribute "status" as "success" if successful
def pull_model(model):
    data = {
        "model": model,
        "stream": False
    }
    response = requests.post(URL, json=data)
    response_json = json.loads(response.text)
    return response_json["status"]