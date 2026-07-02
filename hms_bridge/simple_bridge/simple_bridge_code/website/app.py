from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

@app.post("/api/get_sensor_data")
def get_sensor_data():
    # sensor_id = request.json["sensor_id"]
    if request.json["sensor_id"] == "00000001":
        ret = jsonify({
            "temperature": [2745],
            "humidity": [4544],
            "timestamp": ["2024-06-01T12:00:00Z"]
        })
    else:
        ret = jsonify({
            "temperature": [2445],
            "humidity": [4544],
            "pressure": [1234566],
            "timestamp": ["2024-06-01T12:00:00Z"]
        })

    return ret

@app.post("/api/get_sensor_info")
def get_sensor_info():
    return jsonify({
        "sensor_id":request.json["sensor_id"], 
        "soft_ver":"xxxxxxxx", 
        "hard_ver":"xxxxxxxx", 
        "sensor_type":"00010060", 
        "dataPMaxCount":"xx",
        "calibPMaxCount":"xx", 
        "radio_addr":"xxxxxxxxxx", 
        "initTime":"xxxxxxxxxxxxxxxx"
    })

@app.post("/api/get_sensors_count")
def get_sensors_count():
    return jsonify({
        "sensors_count":1
    })

@app.post("/api/get_sensors_ids")
def get_sensors_ids():
   return jsonify({
      "sensors_ids":["00000001", "00000002"]
   })

@app.route("/")
def home():
  i = open("index.html", "r")
  return i.read()