from flask import Flask, request

app = Flask(__name__)


@app.route('/data', methods=['POST'])
def receive_data():
    data = request.json  # Assuming JSON data is sent
    print(f"Received: {data}")
    return "Data received", 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # Run on port 5000
